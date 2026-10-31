// =====================================================================
//  Null Island Space Program  --  the logo (see logo.h)
// =====================================================================

#include "ui/logo.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "graceloader.h"
#include "pax_codecs.h"
#include "se_direct565.h"  // direct_565_pack, and the display geometry it assumes
#include "se_run.h"        // se_display_info

static char const TAG[] = "logo";

// The ball at its on-screen size, built once (logo.h), in one of two
// shapes. Which one is decided in logo_load() from what the engine says
// its own framebuffers are, so the frame path never has to choose.
//
// FAST (what the badge gets): the artwork pre-packed into the
// framebuffer's own RGB565 halfwords, stored column by column in the
// order the raw buffer stores them -- under PAX_O_ROT_CW a run of
// logical y at a fixed logical x is a run of adjacent halfwords, so a
// column is one contiguous sweep of the framebuffer. Drawing it is then
// plain halfword stores: no per-pixel getter/setter/col2buf/buf2col
// function pointers, no blend for the ~79% of the artwork that is fully
// opaque, and nothing read or written at all for the ~21% that is fully
// transparent. That is the whole difference between a blit and the 59 ms
// the title screen used to spend here.
//
// FALLBACK (any other display): the ARGB8888 layer, drawn with
// pax_draw_image() the way it was before. Slower, but it works whatever
// the panel turns out to be.
//
// The layer carries alpha because the ball is round: the artwork's
// corners are transparent and the framebuffer's format is not, so the
// "_op" variants would paint them as a black square on the starfield.
typedef struct {
    int       w, h;  // size in logical pixels
    uint16_t* rgb;   // w*h, column-major: rgb[x * h + y], packed for the framebuffer
    uint8_t*  alpha;  // w*h, same order; 0 = leave the screen pixel alone
    // Per column: the rows the artwork touches, and the longest run of
    // fully opaque rows inside that span. Everything outside the opaque
    // run goes through the blend path.
    int16_t* y0;
    int16_t* y1;  // one past the last row touched
    int16_t* o0;
    int16_t* o1;  // one past the opaque run
} layer_565_t;

static layer_565_t s_fast;
static pax_buf_t   s_layer;  // the fallback representation
static bool        s_fast_ok;
static bool        s_ready;

// --- The one resample ----------------------------------------------------

// Average the source over each destination pixel's footprint, weighting
// the colours by their alpha (logo.h). Run at load and never again, so it
// is written to be obvious rather than fast: ~880k source reads.
static void build_layer(pax_buf_t const* src, pax_buf_t* dst) {
    int const sw = pax_buf_get_width(src), sh = pax_buf_get_height(src);
    int const dw = pax_buf_get_width(dst), dh = pax_buf_get_height(dst);

    pax_col_t const* s = src->buf_32bpp;
    pax_col_t*       d = dst->buf_32bpp;

    for (int y = 0; y < dh; y++) {
        int y0 = (int)((int64_t)y * sh / dh);
        int y1 = (int)((int64_t)(y + 1) * sh / dh);
        if (y1 <= y0) y1 = y0 + 1;
        if (y1 > sh) y1 = sh;

        for (int x = 0; x < dw; x++) {
            int x0 = (int)((int64_t)x * sw / dw);
            int x1 = (int)((int64_t)(x + 1) * sw / dw);
            if (x1 <= x0) x1 = x0 + 1;
            if (x1 > sw) x1 = sw;

            uint32_t n = 0, alpha = 0, r = 0, g = 0, b = 0;
            for (int sy = y0; sy < y1; sy++) {
                pax_col_t const* row = s + sy * sw;
                for (int sx = x0; sx < x1; sx++) {
                    uint32_t const c = row[sx];
                    uint32_t const a = (c >> 24) & 0xffu;
                    n++;
                    alpha += a;
                    r += a * ((c >> 16) & 0xffu);
                    g += a * ((c >> 8) & 0xffu);
                    b += a * (c & 0xffu);
                }
            }

            uint32_t const a = alpha / n;
            uint32_t out = a << 24;
            if (a != 0) {
                out |= (r / alpha) << 16;
                out |= (g / alpha) << 8;
                out |= b / alpha;
            }
            d[y * dw + x] = out;
        }
    }
}

// --- Packing it for the blit ---------------------------------------------

// Turn the resampled ARGB layer into the column-major 565 form above.
// Column-major is the point: it is the order the framebuffer itself keeps
// a column in, so the draw loop walks both arrays in one direction with
// no multiply per pixel.
static bool pack_layer(pax_buf_t const* argb, layer_565_t* out, bool reversed) {
    int const w = pax_buf_get_width(argb), h = pax_buf_get_height(argb);
    if (w <= 0 || h <= 0 || h > INT16_MAX) return false;

    size_t const px  = (size_t)w * (size_t)h;
    out->w     = w;
    out->h     = h;
    out->rgb   = heap_caps_malloc(px * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    out->alpha = heap_caps_malloc(px * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    out->y0    = heap_caps_malloc((size_t)w * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    out->y1    = heap_caps_malloc((size_t)w * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    out->o0    = heap_caps_malloc((size_t)w * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    out->o1    = heap_caps_malloc((size_t)w * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (out->rgb == NULL || out->alpha == NULL || out->y0 == NULL || out->y1 == NULL || out->o0 == NULL ||
        out->o1 == NULL) {
        return false;
    }

    pax_col_t const* s = argb->buf_32bpp;

    for (int x = 0; x < w; x++) {
        int      touched_lo = h, touched_hi = -1;
        int      run_lo     = h, run_hi = -1;  // longest all-opaque run so far
        int      best_len   = 0;
        uint16_t* dst       = out->rgb + (size_t)x * h;
        uint8_t*  dst_a     = out->alpha + (size_t)x * h;

        for (int y = 0; y < h; y++) {
            uint32_t const c = s[(size_t)y * w + x];
            uint8_t const  a = (uint8_t)((c >> 24) & 0xffu);
            dst[y]           = direct_565_pack(c, reversed);
            dst_a[y]         = a;
            if (a != 0) {
                if (y < touched_lo) touched_lo = y;
                touched_hi = y;
            }
        }

        for (int y = 0; y < h;) {
            if (dst_a[y] != 255) {
                y++;
                continue;
            }
            int const start = y;
            while (y < h && dst_a[y] == 255) y++;
            if (y - start > best_len) {
                best_len = y - start;
                run_lo   = start;
                run_hi   = y - 1;
            }
        }

        out->y0[x] = (int16_t)touched_lo;
        out->y1[x] = (int16_t)(touched_hi + 1);
        out->o0[x] = (int16_t)run_lo;
        out->o1[x] = (int16_t)(run_hi + 1);
    }
    return true;
}

static void pack_free(layer_565_t* l) {
    heap_caps_free(l->rgb);
    heap_caps_free(l->alpha);
    heap_caps_free(l->y0);
    heap_caps_free(l->y1);
    heap_caps_free(l->o0);
    heap_caps_free(l->o1);
    memset(l, 0, sizeof(*l));
}

bool logo_load(void) {
    if (s_ready) return true;

    char const* base = graceloader_get_install_basepath();
    if (base == NULL || base[0] == '\0') {
        ESP_LOGW(TAG, "no install directory from graceloader; running without the logo");
        return false;
    }

    char path[256];
    snprintf(path, sizeof(path), "%s/" LOGO_FILE, base);

    FILE* f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "no %s -- the title will do without it", path);
        return false;
    }

    // Decoded to ARGB8888 whatever the display is: the resample below
    // needs the alpha to weight the averages, and 565 has no alpha to
    // carry it. Whether the layer stays that shape is decided further
    // down.
    pax_buf_t src;
    if (!pax_decode_png_fd(&src, f, PAX_BUF_32_8888ARGB, 0)) {
        ESP_LOGW(TAG, "could not decode %s", path);
        fclose(f);
        return false;
    }
    fclose(f);

    int const sw = pax_buf_get_width(&src), sh = pax_buf_get_height(&src);
    if (sw <= 0 || sh <= 0) {
        pax_buf_destroy(&src);
        ESP_LOGW(TAG, "%s has no size", path);
        return false;
    }

    // Scaled here, once, so every frame after this is a blit. Resampling
    // 940x937 into the screen 60 times a second is what made the title
    // crawl, and nothing in the half-resolution setting can reach it: the
    // logo is drawn after the upscale, at full resolution (main.c).
    int const dh = LOGO_DRAW_H;
    int const dw = (int)(((int64_t)sw * dh + sh / 2) / sh);

    // Which shape the frame path will use, settled here from the layout
    // the engine says its own framebuffers have. The direct blit below
    // hardcodes RGB565 + PAX_O_ROT_CW + the geometry in se_config.h, so
    // it is only allowed when all three actually match; anything else
    // keeps the ARGB layer and pax_draw_image().
    se_display_info_t di;
    se_display_info(&di);
    s_fast_ok = di.pax_format == PAX_BUF_16_565RGB && di.orientation == PAX_O_ROT_CW &&
                di.width == (size_t)DISPLAY_RAW_W && di.height == (size_t)DISPLAY_RAW_H;

    int64_t const t0 = esp_timer_get_time();
    if (!pax_buf_init(&s_layer, NULL, dw, dh, PAX_BUF_32_8888ARGB)) {
        pax_buf_destroy(&src);
        ESP_LOGW(TAG, "no room for a %dx%d logo layer", dw, dh);
        return false;
    }
    build_layer(&src, &s_layer);
    pax_buf_destroy(&src);  // the 3.5 MB original is done with

    if (s_fast_ok) {
        if (pack_layer(&s_layer, &s_fast, di.reversed)) {
            pax_buf_destroy(&s_layer);  // the ARGB layer was scaffolding
            memset(&s_layer, 0, sizeof(s_layer));
        } else {
            pack_free(&s_fast);
            s_fast_ok = false;
            ESP_LOGW(TAG, "no room for the packed logo; falling back to pax_draw_image");
        }
    }

    s_ready = true;
    ESP_LOGI(TAG, "%s: %dx%d -> %dx%d %s layer in %lld ms", path, sw, sh, dw, dh,
             s_fast_ok ? "RGB565 column" : "ARGB8888", (esp_timer_get_time() - t0) / 1000);
    return true;
}

void logo_free(void) {
    if (!s_ready) return;
    if (s_layer.buf != NULL) pax_buf_destroy(&s_layer);  // gone already when the packed layer won
    memset(&s_layer, 0, sizeof(s_layer));
    pack_free(&s_fast);
    s_ready = false;
}

bool logo_ready(void) {
    return s_ready;
}

// Blend one already-packed 565 source pixel over one already-packed 565
// destination pixel. Only the artwork's edge touches this -- a couple of
// thousand pixels a frame, not a hundred thousand -- so it is written for
// correctness rather than speed: unpack both, mix on the alpha, repack.
static inline uint16_t blend_565(uint16_t src, uint16_t dst, uint32_t a) {
    uint32_t const ia = 255u - a;
    uint32_t const r  = ((((src >> 11) & 0x1fu) * a + ((dst >> 11) & 0x1fu) * ia + 127u) / 255u) & 0x1fu;
    uint32_t const g  = ((((src >> 5) & 0x3fu) * a + ((dst >> 5) & 0x3fu) * ia + 127u) / 255u) & 0x3fu;
    uint32_t const b  = (((src & 0x1fu) * a + (dst & 0x1fu) * ia + 127u) / 255u) & 0x1fu;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

// The fast blit. One column of the artwork per iteration, walked down the
// framebuffer's own memory: direct_565_logical_index keeps a column
// contiguous, so each of the three runs below is a straight sweep.
static void draw_565(pax_buf_t* fb, int x_off, int y_off) {
    uint16_t* const px  = (uint16_t*)pax_buf_get_pixels_rw(fb);
    bool const      rev = fb->reverse_endianness;
    int const       w   = s_fast.w, h = s_fast.h;

    int lx0 = x_off, lx1 = x_off + w;
    if (lx0 < 0) lx0 = 0;
    if (lx1 > DISPLAY_LOG_W) lx1 = DISPLAY_LOG_W;

    for (int lx = lx0; lx < lx1; lx++) {
        int       const col  = lx - x_off;
        int       const base = col * h;
        int               a0 = s_fast.y0[col], a1 = s_fast.y1[col];
        if (a0 >= a1) continue;

        // Clip the column to the screen, moving the layer's own row range
        // with it so the arrays below stay lined up with the pixels.
        if (a0 < -y_off) a0 = -y_off;
        if (a1 > DISPLAY_LOG_H - y_off) a1 = DISPLAY_LOG_H - y_off;
        if (a0 >= a1) continue;

        uint16_t const* const src = s_fast.rgb + base;
        uint8_t const* const  alp = s_fast.alpha + base;

        // Split the clipped span into [head | opaque | tail]. The opaque
        // run is the artwork's own, clipped to the span; when the span
        // holds none of it, everything is a blend pixel.
        int m0 = s_fast.o0[col] > a0 ? s_fast.o0[col] : a0;
        int m1 = s_fast.o1[col] < a1 ? s_fast.o1[col] : a1;
        if (m1 < m0) m0 = m1 = a0;

        uint16_t* d = px + direct_565_logical_index(lx, y_off + a0);
        for (int y = a0; y < m0; y++, d--) {
            uint32_t const a = alp[y];
            if (a == 0) continue;
            uint16_t s = src[y], v = *d;
            if (rev) {
                s = (uint16_t)__builtin_bswap16(s);
                v = (uint16_t)__builtin_bswap16(v);
            }
            uint16_t const mixed = blend_565(s, v, a);
            *d                   = rev ? (uint16_t)__builtin_bswap16(mixed) : mixed;
        }

        d = px + direct_565_logical_index(lx, y_off + m0);
        for (uint16_t const* s = src + m0; m0 < m1; m0++, s++, d--) *d = *s;

        d = px + direct_565_logical_index(lx, y_off + m1);
        for (int y = m1; y < a1; y++, d--) {
            uint32_t const a = alp[y];
            if (a == 0) continue;
            uint16_t s = src[y], v = *d;
            if (rev) {
                s = (uint16_t)__builtin_bswap16(s);
                v = (uint16_t)__builtin_bswap16(v);
            }
            uint16_t const mixed = blend_565(s, v, a);
            *d                   = rev ? (uint16_t)__builtin_bswap16(mixed) : mixed;
        }
    }
}

void logo_draw(pax_buf_t* fb, float cx, float cy) {
    if (fb == NULL || !s_ready) return;

    if (s_fast_ok) {
        int const x_off = (int)lroundf(cx) - s_fast.w / 2;
        int const y_off = (int)lroundf(cy) - s_fast.h / 2;
        draw_565(fb, x_off, y_off);
        return;
    }

    // 1:1, which is what pax_draw_image is fast for: at the buffer's own
    // size and with no transform on the buffer it becomes a sprite blit
    // rather than a textured rectangle with per-pixel sampling.
    pax_draw_image(fb, &s_layer, cx - (float)pax_buf_get_width(&s_layer) * 0.5f,
                   cy - (float)pax_buf_get_height(&s_layer) * 0.5f);
}

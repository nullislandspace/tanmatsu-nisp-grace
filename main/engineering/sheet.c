// =====================================================================
//  Null Island Space Program  --  the blueprint sheet (see sheet.h)
// =====================================================================

#include "engineering/sheet.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rocket/parts.h"
#include "se_direct565.h"
#include "se_text.h"

// --- The pen ------------------------------------------------------------

static uint16_t pen(pax_buf_t const* fb, uint32_t argb) {
    return direct_565_pack_for(fb, argb);
}

static void line(pax_buf_t* fb, uint16_t c, int x0, int y0, int x1, int y1) {
    direct_565_line((uint16_t*)pax_buf_get_pixels(fb), x0, y0, x1, y1, c);
}

static void rect(pax_buf_t* fb, uint16_t c, int x, int y, int w, int h) {
    if (w < 2 || h < 2) return;
    line(fb, c, x, y, x + w - 1, y);
    line(fb, c, x, y + h - 1, x + w - 1, y + h - 1);
    line(fb, c, x, y, x, y + h - 1);
    line(fb, c, x + w - 1, y, x + w - 1, y + h - 1);
}

static void label(pax_buf_t* fb, uint32_t col, float h, int x, int y, char const* s) {
    rendertext_draw(fb, col, NULL, h, (float)x, (float)y, s);
}

static int span_of(nisp_stack_t const* s, int stage, float* out0, float* out1) {
    float lo = 1e30f, hi = -1e30f;
    int found = 0;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        if (p->slot == NISP_PART_KIND_LEGS) continue;
        if (stage < 0 ? p->stage >= 0 : p->stage != stage) continue;
        if (p->x0 < lo) lo = p->x0;
        if (p->x1 > hi) hi = p->x1;
        found = 1;
    }
    if (!found) {
        lo = 0.0f;
        hi = 0.0f;
    }
    *out0 = lo;
    *out1 = hi;
    return found;
}

// --- Fit and scroll -------------------------------------------------------

nisp_sheet_view_t nisp_sheet_fit(nisp_stack_t const* s, nisp_rect_t r, int cursor_stage) {
    nisp_sheet_view_t v = {0};

    float total = 0.0f;
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].x1 > total) total = s->part[i].x1;
    }
    v.total_m = total;

    // The sheet's own margins: the title block lives at the bottom, and a
    // rocket that touches it stops looking like a drawing.
    float const pad = 14.0f;
    float const avail = (float)r.h - 2.0f * pad;

    float ppm = total > 0.0f ? avail / total : NISP_SHEET_MAX_PX_PER_M;
    if (ppm > NISP_SHEET_MAX_PX_PER_M) ppm = NISP_SHEET_MAX_PX_PER_M;
    if (ppm < NISP_SHEET_MIN_PX_PER_M) ppm = NISP_SHEET_MIN_PX_PER_M;
    v.px_per_m = ppm;
    v.shown_m = avail / ppm;

    // Past the clamp the sheet scrolls, and it scrolls to the stage the
    // cursor is on: what you are editing is what you can see (Part B1).
    float top = 0.0f;
    if (v.shown_m < total) {
        float c0 = 0.0f, c1 = 0.0f;
        span_of(s, cursor_stage, &c0, &c1);
        top = 0.5f * (c0 + c1) - 0.5f * v.shown_m;
        if (top > total - v.shown_m) top = total - v.shown_m;
        if (top < 0.0f) top = 0.0f;
    }
    v.view_top_m = top;

    // A little left of centre: the diameter callout needs the left margin,
    // the length callout and the title block need the right one.
    v.axis_x = r.x + r.w / 2 - 12;
    v.top_y = r.y + (int)pad;
    return v;
}

static int y_of(nisp_sheet_view_t const* v, float m) {
    return v->top_y + (int)lroundf((m - v->view_top_m) * v->px_per_m);
}

// --- The grid -------------------------------------------------------------

// Aligned to the screen rather than to the column, so the paper reads as
// one sheet even though the columns are drawn by three different files.
static void draw_grid(pax_buf_t* fb, nisp_rect_t r) {
    uint16_t const minor = pen(fb, NISP_GRID_MINOR_ARGB);
    uint16_t const major = pen(fb, NISP_GRID_MAJOR_ARGB);
    int const x_end = r.x + r.w - 1, y_end = r.y + r.h - 1;

    for (int x = (r.x / 20) * 20; x <= x_end; x += 20) {
        if (x < r.x) continue;
        line(fb, (x % 100 == 0) ? major : minor, x, r.y, x, y_end);
    }
    for (int y = (r.y / 20) * 20; y <= y_end; y += 20) {
        if (y < r.y) continue;
        line(fb, (y % 100 == 0) ? major : minor, r.x, y, x_end, y);
    }
}

// --- The rocket -----------------------------------------------------------

// A cylinder seen from the side: two verticals and the two ends. Everything
// on a blueprint is this, so everything on a blueprint is one function.
static void cylinder(pax_buf_t* fb, uint16_t c, int cx, int half_w, int y0, int y1) {
    if (y1 <= y0) y1 = y0 + 1;
    if (half_w < 1) half_w = 1;
    line(fb, c, cx - half_w, y0, cx - half_w, y1);
    line(fb, c, cx + half_w, y0, cx + half_w, y1);
    line(fb, c, cx - half_w, y0, cx + half_w, y0);
    line(fb, c, cx - half_w, y1, cx + half_w, y1);
}

// The bell: a trapezoid opening downward from the stage's tail. Drawn for
// every engine a stage has, one bell per engine, which is how a real
// drawing shows a cluster.
static void bell(pax_buf_t* fb, uint16_t c, int cx, int y, int half_w, int n) {
    int const hw = n > 1 ? half_w / n : half_w;
    if (hw < 2) return;
    int const depth = hw + 3;
    for (int i = 0; i < n; i++) {
        int const bx = cx - half_w + hw + i * (2 * hw);
        line(fb, c, bx - hw / 2, y, bx - hw, y + depth);
        line(fb, c, bx + hw / 2, y, bx + hw, y + depth);
        line(fb, c, bx - hw, y + depth, bx + hw, y + depth);
    }
}

// The capsule: a nose that curves to its shoulders and a heat shield across
// its tail. The shield is drawn wider than the body because that is what
// makes it read as a shield (Part V1).
static void capsule(pax_buf_t* fb, uint16_t c, int cx, int half_w, int y0, int y1) {
    int const shoulder = y0 + (int)lroundf(0.55f * (float)(y1 - y0));
    line(fb, c, cx, y0, cx - half_w, shoulder);
    line(fb, c, cx, y0, cx + half_w, shoulder);
    line(fb, c, cx - half_w, shoulder, cx - half_w, y1);
    line(fb, c, cx + half_w, shoulder, cx + half_w, y1);
    line(fb, c, cx - half_w - 2, y1, cx + half_w + 2, y1);
    // The window, because a capsule without a window is a tank.
    line(fb, c, cx - half_w / 2, y0 + 4, cx + half_w / 2, y0 + 4);
}

// A decoupler: a short band with a hatch across it, which is the drafting
// shorthand for "this comes off here" (D-21).
static void separator(pax_buf_t* fb, uint16_t c, int cx, int half_w, int y0, int y1) {
    line(fb, c, cx - half_w, y0, cx + half_w, y0);
    line(fb, c, cx - half_w, y1, cx + half_w, y1);
    for (int x = cx - half_w; x < cx + half_w; x += 5) {
        line(fb, c, x, y1, x + 3, y0);
    }
}

// Side boosters. The model treats a booster group as one fat cylinder
// (rocket/stack.c); the drawing shows the n separate motors the player
// picked, each narrower than the core, because that is what a side booster
// looks like and what the icon draws too (Part B3).
static void boosters(pax_buf_t* fb, uint16_t c, int cx, int half_w, int y0, int y1, int n) {
    int const bw = (half_w * 3) / 5;
    if (bw < 2) return;
    int const gap = 2;
    int const side = (n + 1) / 2;
    for (int i = 0; i < side; i++) {
        int const off = half_w + gap + bw + i * (2 * bw + gap);
        cylinder(fb, c, cx - off, bw, y0, y1);
        cylinder(fb, c, cx + off, bw, y0, y1);
    }
    // The nose cone on each outermost booster.
    int const off = half_w + gap + bw + (side - 1) * (2 * bw + gap);
    line(fb, c, cx - off - bw, y0, cx - off, y0 - 4);
    line(fb, c, cx - off, y0 - 4, cx - off + bw, y0);
    line(fb, c, cx + off - bw, y0, cx + off, y0 - 4);
    line(fb, c, cx + off, y0 - 4, cx + off + bw, y0);
}

// Legs, stowed against the tank. The one parameter the sheet draws rather
// than only calls out (Part V7): a stage with legs looks different, and
// that is the point of drawing them.
static void legs(pax_buf_t* fb, uint16_t c, int cx, int half_w, int y0, int y1, int n) {
    int const reach = half_w + 8;
    int const top = y1 - (y1 - y0) / 3;
    int const sets = n / 2 > 0 ? n / 2 : 1;
    for (int i = 0; i < sets; i++) {
        int const dy = i * 4;
        line(fb, c, cx - half_w, top + dy, cx - reach, y1 + 2 + dy);
        line(fb, c, cx - reach, y1 + 2 + dy, cx - reach + 4, y1 + 2 + dy);
        line(fb, c, cx + half_w, top + dy, cx + reach, y1 + 2 + dy);
        line(fb, c, cx + reach, y1 + 2 + dy, cx + reach - 4, y1 + 2 + dy);
    }
}

// The canopies a pod carries, drawn as packs stowed in its aft end: one
// shallow arc each, nested outward (D-51). Only when the pod is tall enough
// for the arcs to be separate lines rather than one smudge.
static void chutes(pax_buf_t* fb, uint16_t c, int cx, int half_w, int y1, int n) {
    if (n < 2 || half_w < 3) return;
    for (int i = 0; i < n - 1; i++) {
        int const y = y1 - 4 - 3 * i;
        line(fb, c, cx - half_w, y, cx, y - 3);
        line(fb, c, cx, y - 3, cx + half_w, y);
    }
}

// RCS pods at the shoulders of the stage they belong to.
static void rcs(pax_buf_t* fb, uint16_t c, int cx, int half_w, int y0) {
    for (int s = -1; s <= 1; s += 2) {
        int const x = cx + s * (half_w + 3);
        rect(fb, c, x - 2, y0 + 1, 5, 5);
    }
}

static void draw_stack(pax_buf_t* fb, nisp_stack_t const* s, nisp_sheet_view_t const* v, int cursor_stage,
                       uint32_t base, uint32_t hot) {
    uint16_t const pencil = pen(fb, base);
    uint16_t const mark = pen(fb, hot);
    int const y_lo = v->top_y - 24;
    int const y_hi = v->top_y + (int)(v->shown_m * v->px_per_m) + 24;

    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        if (p->slot == NISP_PART_KIND_LEGS) continue;  // drawn with its stage
        int const y0 = y_of(v, p->x0), y1 = y_of(v, p->x1);
        if (y1 < y_lo || y0 > y_hi) continue;

        int half = (int)lroundf(0.5f * p->diameter_m * v->px_per_m);
        if (half < 2) half = 2;
        int const cx = v->axis_x;
        uint16_t const c = (p->stage == cursor_stage) ? mark : pencil;

        switch (p->slot) {
            case NISP_PART_KIND_SEPARATOR:
                separator(fb, c, cx, half, y0, y1);
                break;
            case NISP_PART_KIND_BOOSTER:
                boosters(fb, c, cx, half, y0, y1, p->engine_count);
                break;
            case NISP_PART_KIND_BODY:
            default:
                if (p->stage < 0) {
                    capsule(fb, c, cx, half, y0, y1);
                    // The pod's two options, on the drawing (D-51): pods at
                    // its shoulders, canopy packs in its aft end.
                    if (s->pod_rcs) rcs(fb, c, cx, half, y0 + (2 * (y1 - y0)) / 5);
                    chutes(fb, c, cx, half, y1, s->chutes);
                } else {
                    cylinder(fb, c, cx, half, y0, y1);
                    if (p->engine_count > 0) bell(fb, c, cx, y1, half, p->engine_count);
                    if (p->stage < s->stage_count && s->rcs[p->stage]) rcs(fb, c, cx, half, y0);
                    if (p->stage < s->stage_count && s->legs_id[p->stage] != NISP_LEGS_NONE) {
                        legs(fb, c, cx, half, y0, y1, nisp_legs_row(s->legs_id[p->stage])->count);
                    }
                }
                break;
        }

        // The part's name, in the left margin, which is where a drawing
        // puts them and where the diameter callout will not reach.
        if (p->slot == NISP_PART_KIND_BODY) {
            char const* const abbr = nisp_part_row(p->kind)->abbr;
            int const w = (int)rendertext_size(NULL, 11.0f, abbr).x;
            label(fb, (p->stage == cursor_stage) ? NISP_MARK_ARGB : NISP_PENCIL_DIM_ARGB, 11.0f,
                  cx - half - 10 - w, y0 + 1, abbr);
        }
    }
}

// --- Dimension callouts ---------------------------------------------------

static void arrow_v(pax_buf_t* fb, uint16_t c, int x, int y, int dir) {
    line(fb, c, x - 3, y + dir * 5, x, y);
    line(fb, c, x + 3, y + dir * 5, x, y);
}

static void arrow_h(pax_buf_t* fb, uint16_t c, int x, int y, int dir) {
    line(fb, c, x + dir * 5, y - 3, x, y);
    line(fb, c, x + dir * 5, y + 3, x, y);
}

static void draw_callouts(pax_buf_t* fb, nisp_stack_t const* s, nisp_sheet_view_t const* v, int stage) {
    nisp_part_t const* body = NULL;
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].slot == NISP_PART_KIND_BODY && s->part[i].stage == stage) {
            body = &s->part[i];
            break;
        }
    }
    if (body == NULL) return;

    uint16_t const c = pen(fb, NISP_MARK_ARGB);
    int const half = (int)lroundf(0.5f * body->diameter_m * v->px_per_m);
    if (half < 2) return;
    int const cx = v->axis_x;
    int const y0 = y_of(v, body->x0), y1 = y_of(v, body->x1);
    int const ym = (y0 + y1) / 2;

    // Diameter: an arrow across the cylinder, its number in the margin.
    line(fb, c, cx - half, ym, cx + half, ym);
    arrow_h(fb, c, cx - half, ym, -1);
    arrow_h(fb, c, cx + half, ym, 1);
    char txt[24];
    snprintf(txt, sizeof(txt), "%.2f m", (double)body->diameter_m);
    int const dw = (int)rendertext_size(NULL, 12.0f, txt).x;
    label(fb, NISP_MARK_ARGB, 12.0f, cx - half - 8 - dw, ym - 6, txt);

    // Length: an extension line out to the right, a dimension line, and the
    // number beside it. This is the number the ladder is changing, drawn.
    int const dx = cx + half + 26;
    line(fb, c, cx + half, y0, dx + 4, y0);
    line(fb, c, cx + half, y1, dx + 4, y1);
    line(fb, c, dx, y0, dx, y1);
    arrow_v(fb, c, dx, y0, 1);
    arrow_v(fb, c, dx, y1, -1);

    snprintf(txt, sizeof(txt), "%.2f m", (double)(body->x1 - body->x0));
    label(fb, NISP_MARK_ARGB, 12.0f, dx + 8, ym - 6, txt);
}

// --- Title block ----------------------------------------------------------

// Bottom right, like a real drawing's, and the one place the unnamed state
// is visible: "Launch" is refused while the name is empty (D-11), and this
// is where the sheet says why.
static void draw_title_block(pax_buf_t* fb, nisp_rect_t r, nisp_sheet_view_t const* v, char const* name,
                             nisp_stack_t const* s) {
    uint16_t const c = pen(fb, NISP_PENCIL_ARGB);
    int const bw = 176, bh = 62;
    int const bx = r.x + r.w - bw - 4, by = r.y + r.h - bh - 8;

    // A clear area first (F-37). A drawing's title block is never something
    // the object is drawn over, and filling it with the paper's own blue is
    // how this renderer keeps that promise without clipping the rocket: a
    // wide stage's tail may reach here, and it stops at the block's edge
    // rather than running through the numbers.
    uint16_t* const px = (uint16_t*)pax_buf_get_pixels(fb);
    uint16_t const paper = pen(fb, NISP_BLUEPRINT_ARGB);
    direct_565_tri(px, (float)bx, (float)by, (float)(bx + bw), (float)by, (float)(bx + bw),
                   (float)(by + bh), paper);
    direct_565_tri(px, (float)bx, (float)by, (float)(bx + bw), (float)(by + bh), (float)bx,
                   (float)(by + bh), paper);

    rect(fb, c, bx, by, bw, bh);
    line(fb, c, bx, by + 20, bx + bw - 1, by + 20);
    line(fb, c, bx, by + 40, bx + bw - 1, by + 40);

    char const* const shown = (name != NULL && name[0] != '\0') ? name : "UNNAMED";
    label(fb, (name != NULL && name[0] != '\0') ? NISP_PENCIL_ARGB : NISP_MARK_ARGB, 14.0f, bx + 6, by + 4, shown);

    char txt[48];
    snprintf(txt, sizeof(txt), "STAGES %d   PARTS %d", (int)(s->stage_count), s->count);
    label(fb, NISP_PENCIL_DIM_ARGB, 12.0f, bx + 6, by + 24, txt);

    // The scale as a ratio, which is what a drawing says rather than what a
    // renderer says: 4 px per metre is 1:250.
    snprintf(txt, sizeof(txt), "SCALE 1:%d   HEIGHT %.1f m", (int)lroundf(1000.0f / v->px_per_m),
             (double)v->total_m);
    label(fb, NISP_PENCIL_DIM_ARGB, 12.0f, bx + 6, by + 44, txt);
}

// --- The sheet ------------------------------------------------------------

void nisp_sheet_draw(pax_buf_t* fb, nisp_rect_t r, nisp_stack_t const* s, nisp_sheet_view_t view,
                     char const* name, int cursor_stage, int callout_stage) {
    if (fb == NULL || s == NULL || r.w <= 0 || r.h <= 0) return;

    draw_grid(fb, r);
    draw_stack(fb, s, &view, cursor_stage, NISP_PENCIL_ARGB, NISP_MARK_ARGB);
    if (callout_stage >= 0) draw_callouts(fb, s, &view, callout_stage);
    draw_title_block(fb, r, &view, name, s);
}

void nisp_sheet_thumb(pax_buf_t* fb, nisp_rect_t r, nisp_design_t const* d) {
    if (fb == NULL || d == NULL || r.w <= 0 || r.h <= 0) return;
    nisp_stack_t stack;
    nisp_stack_build(d, &stack);
    nisp_sheet_view_t const view = nisp_sheet_fit(&stack, r, -1);
    draw_grid(fb, r);
    draw_stack(fb, &stack, &view, -1, NISP_PENCIL_ARGB, NISP_PENCIL_ARGB);
}

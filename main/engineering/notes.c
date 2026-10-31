// =====================================================================
//  Null Island Space Program  --  the post-it notes (see notes.h)
// =====================================================================

#include "engineering/notes.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rocket/parts.h"
#include "se_direct565.h"
#include "se_text.h"

// The only warm colours on a blue screen, on purpose: the sheet is the
// work, the notes are the verdict (Part B4).
#define CARD_MANILA  0xffefd7a6u
#define CARD_PINK    0xfff0b6c0u
#define CARD_YELLOW  0xfff6e27cu
#define CARD_INK     0xff2b2418u

#define CARD_X_PAD 8
#define CARD_TITLE_H 18
#define CARD_ROW_H 15

typedef struct {
    int      x, y, w, h;
    int      tilt;  // how many pixels the far edge sits lower: the card is stuck on crooked
    uint32_t fill, corner;
} card_t;

static float top_at(card_t const* c, int x) {
    return (float)c->y + (float)c->tilt * (float)(x - c->x) / (float)(c->w > 0 ? c->w : 1);
}

// A sheared rectangle: two flat triangles, a folded darker corner, and a
// pencil-thin border that follows the same shear.
static void draw_card(pax_buf_t* fb, card_t const* c) {
    uint16_t* const px = (uint16_t*)pax_buf_get_pixels(fb);
    int const x0 = c->x, x1 = c->x + c->w;
    float const t0 = top_at(c, x0), t1 = top_at(c, x1);

    direct_565_tri(px, (float)x0, t0, (float)x1, t1, (float)x1, t1 + (float)c->h,
                   direct_565_pack_for(fb, c->fill));
    direct_565_tri(px, (float)x0, t0, (float)x1, t1 + (float)c->h, (float)x0, t0 + (float)c->h,
                   direct_565_pack_for(fb, c->fill));

    // The folded corner, bottom right: the thing that makes it read as a
    // stuck-on square of paper rather than a panel.
    int const k = 16;
    direct_565_tri(px, (float)x1 - k, t1 + (float)c->h, (float)x1, t1 + (float)c->h,
                   (float)x1, t1 + (float)c->h - k, direct_565_pack_for(fb, c->corner));

    uint16_t const ink = direct_565_pack_for(fb, CARD_INK);
    direct_565_line(px, x0, (int)t0, x1, (int)t1, ink);
    direct_565_line(px, x0, (int)(t0 + c->h), x1, (int)(t1 + c->h), ink);
    direct_565_line(px, x0, (int)t0, x0, (int)(t0 + c->h), ink);
    direct_565_line(px, x1, (int)t1, x1, (int)(t1 + c->h), ink);
}

static void card_title(pax_buf_t* fb, card_t const* c, char const* s) {
    rendertext_draw(fb, CARD_INK, NULL, 15.0f, (float)(c->x + CARD_X_PAD), (float)c->y + 3.0f, s);
    uint16_t* const px = (uint16_t*)pax_buf_get_pixels(fb);
    int const y = c->y + CARD_TITLE_H;
    direct_565_line(px, c->x + 4, y, c->x + c->w - 4, y + c->tilt, direct_565_pack_for(fb, CARD_INK));
}

// One row: the thing being measured on the left, its number against the
// right edge. Both on the same sheared line, so the card stays one piece
// of paper.
static void card_row(pax_buf_t* fb, card_t const* c, int row, char const* label, char const* value) {
    float const y = (float)c->y + CARD_TITLE_H + 3.0f + CARD_ROW_H * (float)row;
    rendertext_draw(fb, CARD_INK, NULL, 13.0f, (float)(c->x + CARD_X_PAD), y, label);
    if (value == NULL) return;
    float const vw = rendertext_size(NULL, 13.0f, value).x;
    rendertext_draw(fb, CARD_INK, NULL, 13.0f, (float)(c->x + c->w - CARD_X_PAD) - vw, y, value);
}

// A delta-v is four digits and a comma makes it readable: 3,289 m/s.
static void group_thousands(char* buf, int cap, float v) {
    int const whole = (int)lroundf(fabsf(v));
    int const thousands = whole / 1000, rest = whole % 1000;
    if (thousands <= 0) {
        snprintf(buf, (size_t)cap, "%d", rest);
        return;
    }
    snprintf(buf, (size_t)cap, "%d,%03d", thousands, rest);
}

// --- The selected stage -------------------------------------------------

static int stage_card(pax_buf_t* fb, nisp_rect_t r, nisp_budget_t const* b, int stage, int* y_out) {
    card_t c = {0};
    c.x = r.x + 6;
    c.y = *y_out;
    c.w = r.w - 12;
    c.h = CARD_TITLE_H + CARD_ROW_H * 10 + 6;
    c.tilt = 3;
    c.fill = CARD_MANILA;
    c.corner = 0xffc9a86au;
    draw_card(fb, &c);

    nisp_stage_budget_t const* s = &b->stage[stage];
    char title[48];
    snprintf(title, sizeof(title), "STAGE %d  %s", b->stage_count - stage, s->name);
    card_title(fb, &c, title);

    char dv[24], val[32];
    group_thousands(dv, sizeof(dv), s->delta_v_ms);
    snprintf(val, sizeof(val), "%s m/s", dv);
    card_row(fb, &c, 0, "delta-v", val);

    snprintf(val, sizeof(val), "%.2f", (double)s->twr);
    card_row(fb, &c, 1, "TWR at Bob", val);

    snprintf(val, sizeof(val), "%.0f s", (double)s->burn_s);
    card_row(fb, &c, 2, "burn", val);

    snprintf(val, sizeof(val), "%.1f t", (double)s->wet_t);
    card_row(fb, &c, 3, "wet", val);
    snprintf(val, sizeof(val), "%.1f t", (double)s->dry_t);
    card_row(fb, &c, 4, "dry", val);
    snprintf(val, sizeof(val), "%.1f t", (double)s->prop_t);
    card_row(fb, &c, 5, "propellant", val);

    snprintf(val, sizeof(val), "%s x%d", s->engine, s->engine_count);
    card_row(fb, &c, 6, "engine", val);

    card_row(fb, &c, 7, "RCS", s->rcs ? "yes" : "no");
    card_row(fb, &c, 8, "boosters", s->boosters > 0 ? (s->boosters == 4 ? "4" : "2") : "none");

    nisp_legs_row_t const* legs = nisp_legs_row(s->legs);
    if (legs->count > 0) {
        snprintf(val, sizeof(val), "%d @ %.0f m/s", legs->count, (double)legs->rated_v_ms);
        card_row(fb, &c, 9, "legs rated", val);
    } else {
        card_row(fb, &c, 9, "legs", "none");
    }

    *y_out = c.y + c.h + 10;
    return 1;
}

// --- The capsule, which is not a stage ----------------------------------

static int pod_card(pax_buf_t* fb, nisp_rect_t r, nisp_stack_t const* s, int* y_out) {
    card_t c = {0};
    c.x = r.x + 6;
    c.y = *y_out;
    c.w = r.w - 12;
    c.h = CARD_TITLE_H + CARD_ROW_H * 4 + 6;
    c.tilt = 3;
    c.fill = CARD_MANILA;
    c.corner = 0xffc9a86au;
    draw_card(fb, &c);
    card_title(fb, &c, "COMMAND POD");

    nisp_part_t const* pod = NULL;
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].stage == -1) {
            pod = &s->part[i];
            break;
        }
    }

    char val[32];
    if (pod != NULL) {
        snprintf(val, sizeof(val), "%.1f t", (double)pod->mass_t);
        card_row(fb, &c, 0, "mass", val);
        snprintf(val, sizeof(val), "%.1f t", (double)pod->dry_t);
        card_row(fb, &c, 1, "dry", val);
        snprintf(val, sizeof(val), "%.0f %% facing", (double)pod->shield * 100.0);
        card_row(fb, &c, 2, "heat shield", val);
    }
    card_row(fb, &c, 3, "parachute", "integrated");

    *y_out = c.y + c.h + 10;
    return 1;
}

// --- The whole rocket ---------------------------------------------------

static int rocket_card(pax_buf_t* fb, nisp_rect_t r, nisp_budget_t const* b, int* y_out) {
    card_t c = {0};
    c.x = r.x + 6;
    c.y = *y_out;
    c.w = r.w - 12;
    c.h = CARD_TITLE_H + CARD_ROW_H * 5 + 6;
    c.tilt = -2;
    c.fill = CARD_PINK;
    c.corner = 0xffcf8893u;
    draw_card(fb, &c);
    card_title(fb, &c, "WHOLE ROCKET");

    char dv[24], val[40];
    snprintf(val, sizeof(val), "%.1f t", (double)b->total_mass_t);
    card_row(fb, &c, 0, "mass at pad", val);

    group_thousands(dv, sizeof(dv), b->total_delta_v_ms);
    snprintf(val, sizeof(val), "%s m/s", dv);
    card_row(fb, &c, 1, "total delta-v", val);

    snprintf(val, sizeof(val), "%.2f", (double)b->liftoff_twr);
    card_row(fb, &c, 2, "liftoff TWR", val);

    snprintf(val, sizeof(val), "%d / %d", b->stage_count, b->part_count);
    card_row(fb, &c, 3, "stages / parts", val);

    group_thousands(dv, sizeof(dv), b->reach_orbit_ms);
    snprintf(val, sizeof(val), "%s m/s", dv);
    card_row(fb, &c, 4, "to 100 km orbit", val);

    *y_out = c.y + c.h + 10;
    return 1;
}

// --- The warning, which only exists when it is true ---------------------

// Two of these are the trap the budget exists to catch (F-12): a rocket
// with plenty of delta-v and a bottom stage that cannot lift. The third is
// the same trap seen from the other end. None of them is a prediction
// about the air or about reentry (D-37).
static int warning_card(pax_buf_t* fb, nisp_rect_t r, nisp_budget_t const* b, int* y_out) {
    char lines[3][48];
    int n = 0;

    if (!b->first_stage_ok) {
        snprintf(lines[n++], sizeof(lines[0]), "Bottom stage TWR under 1.0:");
        if (n < 3) snprintf(lines[n++], sizeof(lines[0]), "it will not leave the pad.");
    }
    if (b->stage_count > 0 && b->stage[b->stage_count - 1].thrust_kn <= 0.0f) {
        if (n < 3) snprintf(lines[n++], sizeof(lines[0]), "Bottom stage has no engine.");
    }
    if (!b->reach_orbit_ok) {
        snprintf(lines[n++], sizeof(lines[0]), "Not enough delta-v for orbit.");
    }
    if (n == 0) return 0;

    card_t c = {0};
    c.x = r.x + 6;
    c.y = *y_out;
    c.w = r.w - 12;
    c.h = CARD_TITLE_H + 14 * n + 6;
    c.tilt = 2;
    c.fill = CARD_YELLOW;
    c.corner = 0xffd3bd45u;
    draw_card(fb, &c);
    card_title(fb, &c, "READ THIS");

    for (int i = 0; i < n; i++) {
        rendertext_draw(fb, CARD_INK, NULL, 13.0f, (float)(c.x + CARD_X_PAD),
                        (float)c.y + CARD_TITLE_H + 3.0f + 14.0f * (float)i, lines[i]);
    }

    *y_out = c.y + c.h + 10;
    return 1;
}

// --- The column ---------------------------------------------------------

void nisp_notes_draw(pax_buf_t* fb, nisp_rect_t r, nisp_stack_t const* s, nisp_budget_t const* b,
                     int cursor_stage) {
    if (fb == NULL || s == NULL || b == NULL || r.w <= 0 || r.h <= 0) return;

    // The cards are drawn top-down and the column is tall enough for all
    // three, so no card has to be dropped -- but the loop stops rather than
    // draw past the bottom if a later part of the game makes them taller.
    int y = r.y + 4;
    if (cursor_stage >= 0 && cursor_stage < b->stage_count) stage_card(fb, r, b, cursor_stage, &y);
    else pod_card(fb, r, s, &y);
    rocket_card(fb, r, b, &y);
    warning_card(fb, r, b, &y);
}

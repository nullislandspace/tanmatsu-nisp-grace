// =====================================================================
//  Null Island Space Program  --  the HUD (see hud.h)
// =====================================================================

#include "game/hud.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "game/scene.h"
#include "rocket/stack.h"
#include "se_config.h"  // DISPLAY_LOG_W / DISPLAY_LOG_H
#include "se_scene.h"
#include "se_text.h"
#include "sim/bodies.h"
#include "ui/settings.h"

// Text is drawn twice, dark under light, because the scene behind it can
// be anything from basalt to a lit cap. The offset is a hard shadow, not
// a blur: cheap, and it reads at 800x480.
#define SHADOW 1.5f

static void text(pax_buf_t* fb, uint32_t col, float h, float x, float y, char const* s) {
    if (s == NULL || s[0] == '\0') return;
    rendertext_draw(fb, 0xff000000u, NULL, h, x + SHADOW, y + SHADOW, s);
    rendertext_draw(fb, col, NULL, h, x, y, s);
}

// A label and its value, the value always at the same offset so a column
// of numbers lines up on the colon rather than on the label. Labels stay
// short on purpose: the offset is fixed, and a long label would run into
// the number it introduces.
#define LABEL_W 58.0f

static void kv(pax_buf_t* fb, float x, float y, char const* k, char const* v, uint32_t col) {
    text(fb, 0xff7f8f98u, 14.0f, x, y, k);
    text(fb, col, 14.0f, x + LABEL_W, y, v);
}

// --- Number formatting -------------------------------------------------
//
// One unit is not enough for a flight: a launch is measured in metres and
// a hundred metres a second, an orbit in hundreds of kilometres and
// thousands of metres a second. Each of these picks the unit that keeps
// the digits readable, which is the difference between "122.05 km" and
// "122050 m" at a glance.

static void fmt_alt(char* buf, int cap, float km) {
    float const a = fabsf(km);
    if (a < 1.0f) {
        snprintf(buf, cap, "%.0f m", (double)(km * 1000.0f));
    } else if (a < 1000.0f) {
        snprintf(buf, cap, "%.2f km", (double)km);
    } else {
        snprintf(buf, cap, "%.0f km", (double)km);
    }
}

static void fmt_speed(char* buf, int cap, float ms) {
    float const v = fabsf(ms);
    if (v < 1000.0f) {
        snprintf(buf, cap, "%.0f m/s", (double)ms);
    } else if (v < 10000.0f) {
        snprintf(buf, cap, "%.2f km/s", (double)(ms / 1000.0f));
    } else {
        snprintf(buf, cap, "%.1f km/s", (double)(ms / 1000.0f));
    }
}

static void fmt_period(char* buf, int cap, float s) {
    if (!(s > 0.0f)) {
        snprintf(buf, cap, "-");
        return;
    }
    int const total = (int)(s + 0.5f);
    int const h = total / 3600, m = (total % 3600) / 60, sec = total % 60;
    if (h > 0) snprintf(buf, cap, "%dh %02dm", h, m);
    else if (m > 0) snprintf(buf, cap, "%dm %02ds", m, sec);
    else snprintf(buf, cap, "%ds", sec);
}

// T+ on the mission clock. Three digits of seconds and the minutes over
// it: a launch is minutes long, and a clock that rolls over at 999 s is a
// clock nobody can read during a burn.
static void fmt_clock(char* buf, int cap, double t) {
    if (t < 0.0) t = 0.0;
    int const total = (int)(t + 0.5);
    if (total < 1000) {
        snprintf(buf, cap, "T+%d s", total);
    } else {
        snprintf(buf, cap, "T+%d m %03d s", total / 600, total % 600);
    }
}

void hud_reticle(pax_buf_t* fb) {
    if (fb == NULL) return;
    float const cx = (float)DISPLAY_LOG_W * 0.5f, cy = (float)DISPLAY_LOG_H * 0.5f;
    uint32_t const col = 0xff9fe8e8u;
    pax_draw_line(fb, col, cx - 14.0f, cy, cx - 5.0f, cy);
    pax_draw_line(fb, col, cx + 5.0f, cy, cx + 14.0f, cy);
    pax_draw_line(fb, col, cx, cy - 14.0f, cx, cy - 5.0f);
    pax_draw_line(fb, col, cx, cy + 5.0f, cx, cy + 14.0f);
}

void hud_flash(pax_buf_t* fb, char const* msg) {
    if (fb == NULL || msg == NULL || msg[0] == '\0') return;
    pax_vec2f const sz = rendertext_size(NULL, 18.0f, msg);
    float const x = ((float)DISPLAY_LOG_W - sz.x) * 0.5f;
    float const y = (float)DISPLAY_LOG_H - 44.0f;
    text(fb, 0xfffff0c0u, 18.0f, x, y, msg);
}

// A horizontal bar with a name on it: throttle and heat, the two numbers
// that are a range rather than a value and read as a length far faster
// than as digits. The frame is always drawn -- an empty bar still says
// "nothing", which is a reading of its own.
static void bar(pax_buf_t* fb, float x, float y, float w, float h, float frac, uint32_t col) {
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    pax_draw_rect(fb, 0xff2a323au, x, y, w, h);
    if (frac > 0.0f) pax_draw_rect(fb, col, x, y, w * frac, h);
    pax_draw_line(fb, 0xff8fa0a8u, x, y, x + w, y);
    pax_draw_line(fb, 0xff8fa0a8u, x, y + h, x + w, y + h);
    pax_draw_line(fb, 0xff8fa0a8u, x, y, x, y + h);
    pax_draw_line(fb, 0xff8fa0a8u, x + w, y, x + w, y + h);
}

// The two columns. Left is the here: where the ship is and how it is
// moving through whatever it is moving through. Right is the there: the
// orbit it is on, which is the only prediction the game makes.
#define COL_L   14.0f
#define COL_R   628.0f
#define COL_Y0  206.0f
#define COL_DY  19.0f

static void draw_columns(pax_buf_t* fb, nisp_mission_t const* m, nisp_readout_t const* r) {
    char line[96];
    float y = COL_Y0;

    fmt_alt(line, sizeof(line), r->alt_km);
    kv(fb, COL_L, y, "ALT", line, 0xffffffffu);
    y += COL_DY;

    fmt_speed(line, sizeof(line), r->speed_ms);
    kv(fb, COL_L, y, "SPEED", line, 0xffd8e0e8u);
    y += COL_DY;

    // Through the air, or over the ground. They are different questions
    // and only one of them is asked at a time: what matters at 200 km is
    // the ground speed a landing has, what matters at 20 km is the speed
    // the heat and the drag are computed from.
    if (r->in_air) {
        fmt_speed(line, sizeof(line), r->v_rel_ms);
        kv(fb, COL_L, y, "V AIR", line, 0xffe8c8a0u);
    } else {
        fmt_speed(line, sizeof(line), r->v_rel_ms);
        kv(fb, COL_L, y, "V GND", line, 0xffa8c8b0u);
    }
    y += COL_DY;

    if (r->has_air) {
        if (r->q_dyn_pa < 1000.0f) snprintf(line, sizeof(line), "%.0f Pa", (double)r->q_dyn_pa);
        else snprintf(line, sizeof(line), "%.2f kPa", (double)(r->q_dyn_pa / 1000.0f));
        kv(fb, COL_L, y, "Q", line, 0xffc8d8e0u);
    } else {
        kv(fb, COL_L, y, "Q", "no air", 0xff5f6f78u);
    }
    y += COL_DY;

    snprintf(line, sizeof(line), "%.2f t", (double)r->mass_t);
    kv(fb, COL_L, y, "MASS", line, 0xffd8e0e8u);
    y += COL_DY;

    if (r->twr > 0.0f) snprintf(line, sizeof(line), "%.2f", (double)r->twr);
    else snprintf(line, sizeof(line), "-");
    kv(fb, COL_L, y, "TWR", line, r->twr >= 1.0f ? 0xffb0e8b0u : 0xffe8b0b0u);
    y += COL_DY;

    // The one aerodynamic number the model offers (Part A5): how far the
    // centre of pressure is behind the centre of mass. Positive means the
    // air will hold the nose into the airstream by itself; negative means
    // it will not, and the rocket is a dart with the dart's point in the
    // wrong end (D-47).
    if (r->in_air) {
        snprintf(line, sizeof(line), "%+.2f m", (double)r->static_margin_m);
        kv(fb, COL_L, y, "MARGIN", line, r->static_margin_m >= 0.0f ? 0xffb0e8b0u : 0xffe8b0b0u);
    }

    y = COL_Y0;
    if (r->escaped) {
        kv(fb, COL_R, y, "APO", "escape", 0xffc8d8e0u);
    } else {
        fmt_alt(line, sizeof(line), r->apoapsis_km);
        kv(fb, COL_R, y, "APO", line, 0xffb0d8f0u);
    }
    y += COL_DY;

    fmt_alt(line, sizeof(line), r->periapsis_km);
    kv(fb, COL_R, y, "PERI", line, r->periapsis_km < 70.0f ? 0xffe8c8a0u : 0xffb0d8f0u);
    y += COL_DY;

    if (r->escaped) {
        kv(fb, COL_R, y, "ORBIT", "open", 0xffc8d8e0u);
    } else {
        fmt_period(line, sizeof(line), r->period_s);
        kv(fb, COL_R, y, "PERIOD", line, 0xffc8d8e0u);
    }
    y += COL_DY;

    snprintf(line, sizeof(line), "%.1f deg", (double)r->incl_deg);
    kv(fb, COL_R, y, "INCL", line, 0xffc8d8e0u);
    y += COL_DY;

    nisp_body_t const* const b = nisp_body(m->ship.body);
    kv(fb, COL_R, y, "SOI", (b != NULL) ? b->name : "?", 0xff9fb0b8u);
    y += COL_DY;

    snprintf(line, sizeof(line), "x%.0f", (double)m->last_warp.warp);
    kv(fb, COL_R, y, "WARP", line, m->last_warp.warp > 1.0f ? 0xffffe0a0u : 0xff9fb0b8u);
}

// A named bar: the label on the left, the bar in the middle, the number on
// the right. Fixed positions, because two bars stacked under the reticle
// have to line up with each other or they read as two different things.
static void bar_row(pax_buf_t* fb, float y, char const* label, float frac, uint32_t col, char const* value) {
    text(fb, 0xff7f8f98u, 14.0f, 246.0f, y - 2.0f, label);
    bar(fb, 310.0f, y, 180.0f, 10.0f, frac, col);
    text(fb, col, 14.0f, 500.0f, y - 2.0f, value);
}

// The bottom centre: the engine and the control system, the two things a
// player is actually doing, next to each other and under the reticle the
// ship is pointed along.
static void draw_controls(pax_buf_t* fb, nisp_mission_t const* m, nisp_readout_t const* r) {
    nisp_stack_t const* const s = &m->ship.stack;
    char line[96];

    snprintf(line, sizeof(line), "%.0f%%  %.0f kN", (double)(s->throttle * 100.0f), (double)r->thrust_kn);
    bar_row(fb, 378.0f, "THRUST", s->throttle, 0xffffc25au, line);

    // The heat bar is the only warning this game gives (Part A4): a part at
    // its limit is gone, and there is no repair. It stays off the screen
    // until there is heat worth watching, so a coast is not decorated with
    // an empty bar.
    if (r->heat_max > 0.02f) {
        snprintf(line, sizeof(line), "%.0f%%", (double)(r->heat_max * 100.0f));
        uint32_t const col =
            r->heat_max > 0.75f ? 0xffe85050u : (r->heat_max > 0.4f ? 0xffe8a050u : 0xffe8d070u);
        bar_row(fb, 358.0f, "HEAT", r->heat_max, col, line);
    }

    // One line of state: which stage is armed, what the control system is
    // doing, and what has already been let go. All of it is a word rather
    // than a number, because it is read as "is this what I meant".
    char status[128];
    snprintf(status, sizeof(status), "stage %d/%d   SAS %s   %s%s%s", s->live_stage + 1, s->stage_count,
             nisp_sas_name(m->sas), m->rcs ? "RCS on" : "RCS off", s->parachute ? "   chute" : "",
             nisp_stack_legs_deployed(s, s->live_stage) ? "   legs" : "");
    pax_vec2f const sz = rendertext_size(NULL, 14.0f, status);
    text(fb, 0xffc8d8e0u, 14.0f, ((float)DISPLAY_LOG_W - sz.x) * 0.5f, 398.0f, status);

    // Warp refusal, in the warp's own words (flight/warp.c): the reason a
    // key did nothing is the thing a player wants to know at that moment.
    if (m->last_warp.reason[0] != '\0') {
        pax_vec2f const s2 = rendertext_size(NULL, 13.0f, m->last_warp.reason);
        text(fb, 0xffffe0a0u, 13.0f, ((float)DISPLAY_LOG_W - s2.x) * 0.5f, 418.0f, m->last_warp.reason);
    }
}

void hud_draw(pax_buf_t* fb, nisp_game_t const* game, nisp_mission_t const* mission,
              nisp_readout_t const* readout, bool info, hud_stats_t const* stats) {
    if (fb == NULL || game == NULL) return;

    char line[96];

    // Top left: which world this is.
    text(fb, 0xffffffffu, 20.0f, 14.0f, 12.0f, game->name);
    snprintf(line, sizeof(line), "seed %u", (unsigned)game->seed);
    text(fb, 0xff9fb0b8u, 13.0f, 14.0f, 36.0f, line);

    if (mission == NULL || readout == NULL) return;

    // Top right: the mission clock and where the flight is.
    fmt_clock(line, sizeof(line), mission->t);
    pax_vec2f sz = rendertext_size(NULL, 16.0f, line);
    text(fb, 0xffd8e0e8u, 16.0f, (float)DISPLAY_LOG_W - sz.x - 14.0f, 12.0f, line);

    char const* state = nisp_mission_state_name(mission->state);
    if (mission->state == NISP_MISSION_LANDED) {
        snprintf(line, sizeof(line), "%s  %.1f m/s  tilt %.0f deg", state, (double)mission->ship.impact_ms,
                 (double)mission->ship.impact_tilt_deg);
    } else if (mission->state == NISP_MISSION_WRECKED) {
        snprintf(line, sizeof(line), "%s  %s stage %d", state,
                 mission->ship.wreck_kind >= 0 ? "part" : "impact", mission->ship.wreck_stage + 1);
    } else {
        snprintf(line, sizeof(line), "%s", state);
    }
    sz = rendertext_size(NULL, 14.0f, line);
    uint32_t const state_col = (mission->state == NISP_MISSION_WRECKED) ? 0xffe87070u
                               : (mission->state == NISP_MISSION_LANDED) ? 0xffb0e8b0u : 0xff9fb0b8u;
    text(fb, state_col, 14.0f, (float)DISPLAY_LOG_W - sz.x - 14.0f, 34.0f, line);

    draw_columns(fb, mission, readout);
    draw_controls(fb, mission, readout);

    if (!info) return;

    // The diagnostic block: everything a bug report needs quoted, in one
    // place, on the key that means "show me what this is".
    int tri = 0, line_n = 0, pt = 0;
    int64_t tri_us = 0, line_us = 0, pt_us = 0;
    scene_raster_stats(&tri, &line_n, &tri_us, &line_us);
    scene_point_stats(&pt, &pt_us);

    static char const* const view_name[] = {"near", "medium", "far"};
    int const v = settings_view();
    snprintf(line, sizeof(line), "slot %d  view %s (%.0f km)  %s-res", stats != NULL ? stats->slot : -1,
             (v >= 0 && v < NISP_VIEW_COUNT) ? view_name[v] : "?", (double)nisp_view_range(v),
             (stats != NULL && stats->half) ? "half" : "full");
    text(fb, 0xffc0d0d8u, 13.0f, 14.0f, 60.0f, line);
    snprintf(line, sizeof(line), "tris %d  lines %d  points %d  rast %.1f ms", tri, line_n, pt,
             (double)((stats != NULL ? (float)stats->rast_us : (float)tri_us) / 1000.0f));
    text(fb, 0xffc0d0d8u, 13.0f, 14.0f, 78.0f, line);
    snprintf(line, sizeof(line), "sim %d substeps  %s%s", mission->last_warp.substeps,
             mission->last_warp.coasting ? "coast" : "ticked", mission->last_warp.capped ? " (capped)" : "");
    text(fb, 0xffc0d0d8u, 13.0f, 14.0f, 96.0f, line);
}

// =====================================================================
//  Null Island Space Program  --  the HUD (see hud.h)
// =====================================================================

#include "game/hud.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "game/input.h"
#include "game/scene.h"
#include "rocket/parts.h"
#include "rocket/stack.h"
#include "se_config.h"  // DISPLAY_LOG_W / DISPLAY_LOG_H
#include "se_scene.h"
#include "se_text.h"
#include "sim/bodies.h"
#include "sim/ephemeris.h"
#include "sim/surface.h"
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

// --- The attitude indicator (plan 6.2) ------------------------------------
//
// Prograde, retrograde and the orbit normal are directions in the world, not
// parts of the ship, so where they land on screen is a question about the
// camera and nothing else. They are projected through the same pinhole the
// scene rasterizes with: a direction is a point one unit out along it from
// the eye, and the engine's projection of that point is where the marker
// goes. A direction behind the eye has no screen position and is not drawn.
//
// A marker whose projection lands off the sky is pulled in along its own
// bearing, because "it is that way, further out" is the reading a pilot needs
// -- and it is pulled in to the rectangle no readout is using, so a marker
// never lands on a number.
#define SKY_MIN_X 130.0f
#define SKY_MAX_X 670.0f
#define SKY_MIN_Y 56.0f
#define SKY_MAX_Y 344.0f
#define MARK_R    8.0f

typedef struct {
    // Frame axes at the camera: local up, north, east.
    double l_up[3], l_north[3], l_east[3];
    // Engine-space camera axes, rebuilt from the camera's own yaw / pitch /
    // roll with the formulas se_scene.c builds its basis from, so a marker
    // lands on the scene that was drawn with them.
    float cam_right[3], cam_up[3], cam_fwd[3];
} view_t;

static void view_at(nisp_camera_t const* cam, view_t* v) {
    nisp_local_frame(cam->frame_body, cam->pos_km, v->l_up, v->l_north, v->l_east);
    float const cy = cosf(cam->yaw), sy = sinf(cam->yaw);
    float const cp = cosf(cam->pitch), sp = sinf(cam->pitch);
    float const cr = cosf(cam->roll), sr = sinf(cam->roll);
    v->cam_right[0] = cy * cr + sy * sp * sr;
    v->cam_right[1] = cp * sr;
    v->cam_right[2] = -sy * cr + cy * sp * sr;
    v->cam_up[0] = -cy * sr + sy * sp * cr;
    v->cam_up[1] = cp * cr;
    v->cam_up[2] = sy * sr + cy * sp * cr;
    v->cam_fwd[0] = sy * cp;
    v->cam_fwd[1] = -sp;
    v->cam_fwd[2] = cy * cp;
}

static float dot_df(double const a[3], float const b[3]) {
    return (float)(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}
static float dot_ff(float const a[3], float const b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// A frame-axis unit direction to a screen position. False when it is behind
// the eye, where there is nothing to mark.
static bool project_dir(view_t const* v, float const d[3], float* out_x, float* out_y) {
    // The local frame maps onto the engine's space as (east, up, -north):
    // the same mapping game/scene.c submits every vertex with.
    float const e[3] = {dot_df(v->l_east, d), dot_df(v->l_up, d), -dot_df(v->l_north, d)};
    float const cx = dot_ff(v->cam_right, e), cy = dot_ff(v->cam_up, e), cz = dot_ff(v->cam_fwd, e);
    if (cz < -1e-4f) return false;  // behind the eye
    if (cz <= 1e-4f) {
        // Exactly to the side: the pinhole has no finite answer for it, but
        // the bearing on screen is still (cx, -cy), and that is what is worth
        // showing. Placed far out so the clamp below puts it on the edge.
        float const m = sqrtf(cx * cx + cy * cy);
        if (m < 1e-4f) return false;  // ahead or behind along the axis: nothing to say
        *out_x = RENDER_HALF_W + 1000.0f * cx / m;
        *out_y = RENDER_HORIZON_Y - 1000.0f * cy / m;
        return true;
    }
    float const inv = 1.0f / cz;
    *out_x = RENDER_HALF_W + RENDER_FOCAL_LEN * cx * inv;
    *out_y = RENDER_HORIZON_Y - RENDER_FOCAL_LEN * cy * inv;
    return true;
}

// Pull a marker in to the edge of the sky area, along its own bearing from
// the projection centre, so the bearing is what survives the clamp. True when
// it moved.
static bool clamp_marker(float* x, float* y) {
    float const ox = RENDER_HALF_W, oy = RENDER_HORIZON_Y;
    float dx = *x - ox, dy = *y - oy;
    float const hx = SKY_MAX_X - ox;  // the area is symmetric left to right
    float const hy_up = oy - SKY_MIN_Y, hy_dn = SKY_MAX_Y - oy;
    float k = 1.0f;
    if (fabsf(dx) > hx) k = fminf(k, hx / fabsf(dx));
    if (dy < 0.0f && -dy > hy_up) k = fminf(k, hy_up / -dy);
    if (dy > 0.0f && dy > hy_dn) k = fminf(k, hy_dn / dy);
    if (k >= 1.0f) return false;
    *x = ox + dx * k;
    *y = oy + dy * k;
    return true;
}

static uint32_t dimmed(uint32_t c) {
    uint32_t const r = ((c >> 16) & 0xffu) / 2u, g = ((c >> 8) & 0xffu) / 2u, b = (c & 0xffu) / 2u;
    return 0xff000000u | (r << 16) | (g << 8) | b;
}

// A marker: a triangle whose point faces away from the middle of the screen,
// because the point is the direction, with a short label on the inside of it.
// A clamped marker is drawn dimmer -- it is a bearing, not a place.
static void marker(pax_buf_t* fb, view_t const* v, float const d[3], char const* label, uint32_t col) {
    float x, y;
    if (!project_dir(v, d, &x, &y)) return;
    uint32_t const c = clamp_marker(&x, &y) ? dimmed(col) : col;

    float dx = x - RENDER_HALF_W, dy = y - RENDER_HORIZON_Y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1.0f) {
        dx = 0.0f;
        dy = -1.0f;
    } else {
        dx /= len;
        dy /= len;
    }
    float const px = -dy, py = dx;  // across the bearing
    float const tx = x + dx * MARK_R, ty = y + dy * MARK_R;
    float const b1x = x - dx * MARK_R + px * MARK_R * 0.7f, b1y = y - dy * MARK_R + py * MARK_R * 0.7f;
    float const b2x = x - dx * MARK_R - px * MARK_R * 0.7f, b2y = y - dy * MARK_R - py * MARK_R * 0.7f;
    pax_draw_line(fb, c, tx, ty, b1x, b1y);
    pax_draw_line(fb, c, b1x, b1y, b2x, b2y);
    pax_draw_line(fb, c, b2x, b2y, tx, ty);

    pax_vec2f const sz = rendertext_size(NULL, 12.0f, label);
    text(fb, c, 12.0f, x - dx * (MARK_R + 9.0f) - sz.x * 0.5f, y - dy * (MARK_R + 9.0f) - sz.y * 0.5f, label);
}

static void draw_attitude(pax_buf_t* fb, nisp_mission_t const* m, nisp_camera_t const* cam) {
    view_t v;
    view_at(cam, &v);

    float d[3];
    nisp_vessel_prograde(&m->ship, d);
    marker(fb, &v, d, "PRO", 0xff70e870u);
    for (int i = 0; i < 3; i++) d[i] = -d[i];
    marker(fb, &v, d, "RET", 0xffe87070u);

    nisp_vessel_normal(&m->ship, d);
    marker(fb, &v, d, "NRM", 0xff9fb0f0u);
}

// The stability readout as a picture (plan 6.6, D-37): the stack nose to tail
// as a line, with the centre of mass, the drag centre and the centre of
// thrust ticked on it. The signed number in the left column is the same fact;
// this is the form a player watches change, which is how the dart case gets
// taught instead of being explained -- the drag mark crossing the mass mark
// as a stage empties is the dart being assembled in front of them.
#define STRIP_X  14.0f
#define STRIP_W  176.0f
#define STRIP_Y  372.0f

// `shape` is 0 for the mass square, +1 for a mark pointing down, -1 for one
// pointing up. The three marks have to be tellable apart at a glance, and they
// are often close enough together that colour alone will not do it.
static void strip_mark(pax_buf_t* fb, float along, float x0, float span, uint32_t col, int shape) {
    float t = (along - x0) / span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float const x = STRIP_X + STRIP_W * t;
    if (shape == 0) {
        pax_draw_rect(fb, col, x - 2.5f, STRIP_Y - 2.5f, 5.0f, 5.0f);
        return;
    }
    float const s = (float)shape;
    pax_draw_line(fb, col, x - 4.0f, STRIP_Y - 6.0f * s, x + 4.0f, STRIP_Y - 6.0f * s);
    pax_draw_line(fb, col, x - 4.0f, STRIP_Y - 6.0f * s, x, STRIP_Y + 2.0f * s);
    pax_draw_line(fb, col, x + 4.0f, STRIP_Y - 6.0f * s, x, STRIP_Y + 2.0f * s);
}

static void draw_stability(pax_buf_t* fb, nisp_stack_t const* s) {
    if (s->count <= 0) return;
    float const x0 = s->part[0].x0, x1 = s->part[s->count - 1].x1;
    float const span = x1 - x0;
    if (!(span > 0.0f)) return;

    text(fb, 0xff7f8f98u, 14.0f, STRIP_X, STRIP_Y - 20.0f, "STABILITY");
    pax_draw_line(fb, 0xff8fa0a8u, STRIP_X, STRIP_Y, STRIP_X + STRIP_W, STRIP_Y);
    pax_draw_line(fb, 0xff8fa0a8u, STRIP_X, STRIP_Y - 9.0f, STRIP_X, STRIP_Y + 9.0f);
    pax_draw_line(fb, 0xff8fa0a8u, STRIP_X + STRIP_W, STRIP_Y - 9.0f, STRIP_X + STRIP_W, STRIP_Y + 9.0f);

    strip_mark(fb, s->com_x, x0, span, 0xffffffffu, 0);
    strip_mark(fb, s->cp_x, x0, span, 0xffe8a050u, 1);
    if (s->cot_x > 0.0f) strip_mark(fb, s->cot_x, x0, span, 0xff8fd8f0u, -1);

    // The legend is three words, each in the colour of its mark: short enough
    // to read at a glance, and it says which end of the stack is which.
    text(fb, 0xffffffffu, 12.0f, STRIP_X, STRIP_Y + 12.0f, "MASS");
    text(fb, 0xffe8a050u, 12.0f, STRIP_X + 42.0f, STRIP_Y + 12.0f, "DRAG");
    text(fb, 0xff8fd8f0u, 12.0f, STRIP_X + 86.0f, STRIP_Y + 12.0f, "THRUST");
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

    // The clock the sky is on above the ship (plan 6.1). It is lit or it is
    // not, and the row is coloured by the sun's height rather than by the
    // hour, so a ship over the terminator reads as what it is.
    int mins = (int)(nisp_solar_time(m->ship.body, m->t, m->ship.pos_km) * 60.0f + 0.5f) % (24 * 60);
    if (mins < 0) mins += 24 * 60;
    snprintf(line, sizeof(line), "%02d:%02d", mins / 60, mins % 60);
    kv(fb, COL_L, y, "LOCAL", line,
       nisp_sun_elevation(m->ship.body, m->t, m->ship.pos_km) > 0.0f ? 0xffe8d8a0u : 0xff9fb0b8u);
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

    // Seconds to the next apoapsis, under the apoapsis it counts down to.
    // A hyperbola has no apoapsis to reach, and says so twice rather than
    // showing a number that means nothing (plan 6.1).
    if (r->escaped || r->time_to_ap_s < 0.0f) {
        kv(fb, COL_R, y, "T+AP", "-", 0xff5f6f78u);
    } else {
        fmt_period(line, sizeof(line), r->time_to_ap_s);
        kv(fb, COL_R, y, "T+AP", line, 0xff9fb0b8u);
    }
    y += COL_DY;

    fmt_alt(line, sizeof(line), r->periapsis_km);
    kv(fb, COL_R, y, "PERI", line, r->periapsis_km < 70.0f ? 0xffe8c8a0u : 0xffb0d8f0u);
    y += COL_DY;

    if (r->time_to_pe_s < 0.0f) {
        kv(fb, COL_R, y, "T+PE", "-", 0xff5f6f78u);
    } else {
        fmt_period(line, sizeof(line), r->time_to_pe_s);
        kv(fb, COL_R, y, "T+PE", line, 0xff9fb0b8u);
    }
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
// Propellant, the way a player asks for it: how much is left of what the
// armed stage can burn. A stage that has already burned out reads as an
// empty bar, which is the correct answer and the one that says "stage it"
// without a word being spent on it.
static void draw_fuel(pax_buf_t* fb, nisp_mission_t const* m) {
    nisp_stack_t const* const s = &m->ship.stack;
    float left = 0.0f, full = 0.0f;
    nisp_stack_propellant(s, s->live_stage >= 0 ? s->live_stage : -1, &left, &full);

    float const frac = full > 0.0f ? left / full : 0.0f;
    uint32_t const col = frac < 0.10f ? 0xffe8b0b0u : 0xffa8d8b0u;
    char line[48];
    snprintf(line, sizeof(line), "%.1f / %.1f t", (double)left, (double)full);
    bar_row(fb, 338.0f, "FUEL", frac, col, line);
}

// The key legend (plan 6.3): what the keys do, spelled with the keys this
// player actually has -- built from the bindings rather than written out, so
// a remapped control says the remapped thing (D-58). Two small lines at the
// very bottom, to be glanced at once and then ignored.
static void legend_keys(char* buf, int cap, nisp_action_t const* acts, int n) {
    buf[0] = '\0';
    char key[16];
    for (int i = 0; i < n; i++) {
        input_key_name(input_key(acts[i]), key, sizeof(key));
        int len = (int)strlen(buf);
        if (len + (int)strlen(key) + 2 >= cap) break;
        if (len > 0) {
            buf[len] = '/';
            buf[len + 1] = '\0';
            len++;
        }
        snprintf(buf + len, cap - len, "%s", key);
    }
}

static void legend_add(char* line, int cap, char const* label, nisp_action_t const* acts, int n) {
    char keys[48];
    legend_keys(keys, sizeof(keys), acts, n);
    int const len = (int)strlen(line);
    if (len >= cap) return;
    if (len > 0) {
        if (cap - len < 4) return;
        line[len] = ' ';
        line[len + 1] = ' ';
        line[len + 2] = '\0';
    }
    snprintf(line + strlen(line), cap - (int)strlen(line), "%s %s", keys, label);
}

static void draw_legend(pax_buf_t* fb) {
    static nisp_action_t const stage[] = {NISP_STAGE};
    static nisp_action_t const throttle[] = {NISP_UP, NISP_DOWN};
    static nisp_action_t const cut[] = {NISP_CUT};
    static nisp_action_t const sas[] = {NISP_SAS};
    static nisp_action_t const rcs[] = {NISP_RCS};
    static nisp_action_t const steer[] = {NISP_FORWARD, NISP_BACK, NISP_LEFT, NISP_RIGHT};
    static nisp_action_t const roll[] = {NISP_ROLL_LEFT, NISP_ROLL_RIGHT};
    static nisp_action_t const legs[] = {NISP_LEGS};
    static nisp_action_t const chute[] = {NISP_CHUTE};
    static nisp_action_t const look[] = {NISP_LOOK_UP, NISP_LOOK_DOWN, NISP_LOOK_LEFT, NISP_LOOK_RIGHT};
    static nisp_action_t const map_key[] = {NISP_MAP};

    char one[128] = "", two[128] = "";
    legend_add(one, sizeof(one), "stage", stage, 1);
    legend_add(one, sizeof(one), "throttle", throttle, 2);
    legend_add(one, sizeof(one), "cut", cut, 1);
    legend_add(one, sizeof(one), "SAS", sas, 1);
    legend_add(one, sizeof(one), "RCS", rcs, 1);
    legend_add(two, sizeof(two), "steer", steer, 4);
    legend_add(two, sizeof(two), "roll", roll, 2);
    legend_add(two, sizeof(two), "legs", legs, 1);
    legend_add(two, sizeof(two), "chute", chute, 1);
    legend_add(two, sizeof(two), "look", look, 4);
    legend_add(two, sizeof(two), "map", map_key, 1);

    pax_vec2f const a = rendertext_size(NULL, 12.0f, one);
    pax_vec2f const b = rendertext_size(NULL, 12.0f, two);
    text(fb, 0xff9fb0b8u, 12.0f, ((float)DISPLAY_LOG_W - a.x) * 0.5f, 456.0f, one);
    text(fb, 0xff9fb0b8u, 12.0f, ((float)DISPLAY_LOG_W - b.x) * 0.5f, 469.0f, two);
}

static void draw_controls(pax_buf_t* fb, nisp_mission_t const* m, nisp_readout_t const* r) {
    nisp_stack_t const* const s = &m->ship.stack;
    char line[96];

    draw_fuel(fb, m);

    // The bar is the DIAL and the number is what the engines are actually
    // making, deliberately not the same quantity (D-56): a full bar over
    // "0 kN" is what a rocket on the pad with cold engines looks like, and
    // that mismatch is the whole answer to "is it firing".
    snprintf(line, sizeof(line), "%.0f%%  %.0f kN", (double)(m->throttle * 100.0f), (double)r->thrust_kn);
    uint32_t const thrust_col = m->ignited ? 0xffffc25au : 0xff6f7f88u;
    bar_row(fb, 378.0f, "THRUST", m->throttle, thrust_col, line);

    // The heat bar is the only warning this game gives (Part A4): a part at
    // its limit is gone, and there is no repair. It stays off the screen
    // until there is heat worth watching, so a coast is not decorated with
    // an empty bar.
    if (r->heat_max > 0.02f) {
        uint32_t const col =
            r->heat_max > 0.75f ? 0xffe85050u : (r->heat_max > 0.4f ? 0xffe8a050u : 0xffe8d070u);
        // Past 70 % the bar stops being enough: which part is going is the
        // thing a player can still act on, and the name is the whole point of
        // showing it at all (plan 6.5).
        char const* who = "";
        if (r->heat_max > 0.70f && r->heat_part >= 0 && r->heat_part < s->count) {
            who = nisp_part_row(s->part[r->heat_part].kind)->name;
        }
        snprintf(line, sizeof(line), "%.0f%% %s", (double)(r->heat_max * 100.0f), who);
        bar_row(fb, 358.0f, "HEAT", r->heat_max, col, line);
    }

    // One line of state: which stage is armed, what the control system is
    // doing, and what has already been let go. All of it is a word rather
    // than a number, because it is read as "is this what I meant".
    char pieces[32] = "";
    if (m->debris_count > 0) {
        snprintf(pieces, sizeof(pieces), "   %d loose piece%s", m->debris_count,
                 m->debris_count == 1 ? "" : "s");
    }
    char const* const ignition =
        m->ignited ? (m->throttle > 0.0f ? "engines lit" : "engines idle") : "engines cold";
    char status[160];
    // Seven conversions, seven arguments. The tail is four optional words
    // that are either a short label or "", which is why the format string
    // reads as a sentence rather than as a list.
    snprintf(status, sizeof(status), "stage %d/%d   %s   SAS %s   %s%s%s%s", s->live_stage + 1, s->stage_count,
             ignition, nisp_sas_name(m->sas), m->rcs ? "RCS on" : "RCS off", s->parachute ? "   chute" : "",
             nisp_stack_legs_deployed(s, s->live_stage) ? "   legs" : "", pieces);
    pax_vec2f const sz = rendertext_size(NULL, 14.0f, status);
    text(fb, 0xffc8d8e0u, 14.0f, ((float)DISPLAY_LOG_W - sz.x) * 0.5f, 398.0f, status);

    // Warp refusal, in the warp's own words (flight/warp.c): the reason a
    // key did nothing is the thing a player wants to know at that moment.
    if (m->last_warp.reason[0] != '\0') {
        pax_vec2f const s2 = rendertext_size(NULL, 13.0f, m->last_warp.reason);
        text(fb, 0xffffe0a0u, 13.0f, ((float)DISPLAY_LOG_W - s2.x) * 0.5f, 418.0f, m->last_warp.reason);
    }

    draw_legend(fb);
}

void hud_draw(pax_buf_t* fb, nisp_game_t const* game, nisp_mission_t const* mission,
              nisp_readout_t const* readout, nisp_camera_t const* cam, bool info,
              hud_stats_t const* stats) {
    if (fb == NULL || game == NULL) return;

    char line[96];

    // Top left: what the ship is called, which is the name the player gave it
    // in Engineering (plan 6.1). A mission with no name -- a scripted case, a
    // slot from before names existed -- falls back to the world's name, which
    // is also what a menu over a frozen view has to show.
    char const* name = game->name;
    if (mission != NULL && mission->vessel_name[0] != '\0') name = mission->vessel_name;
    text(fb, 0xffffffffu, 20.0f, 14.0f, 12.0f, name);

    if (mission == NULL || readout == NULL) return;

    // Top right: the mission clock -- T+ from the moment this flight started,
    // not the absolute sim clock, which is a number no flight is measured by.
    fmt_clock(line, sizeof(line), mission->t - mission->t_launch);
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

    // The attitude markers need this frame's camera, and a caller that has
    // none (a menu over a frozen view) gets the numbers without them.
    if (cam != NULL) draw_attitude(fb, mission, cam);
    // The stability strip is an aerodynamic picture, so it only means
    // something while there is air to be aerodynamic in (plan 6.6).
    if (readout->in_air) draw_stability(fb, &mission->ship.stack);

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

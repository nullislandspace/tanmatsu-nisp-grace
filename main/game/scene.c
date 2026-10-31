// =====================================================================
//  Null Island Space Program  --  the sky, on the badge (see scene.h)
// =====================================================================

#include "game/scene.h"

#include <math.h>
#include <string.h>

#include "common/rng.h"
#include "esp_timer.h"
#include "se_config.h"
#include "se_light.h"
#include "se_scene.h"
#include "testkit/profile.h"

#include "sim/bodies.h"
#include "sim/ephemeris.h"
#include "sim/surface.h"
#include "sim/units.h"

#define TAU 6.2831853071795865f

// --- The shell -----------------------------------------------------------

// Stars, and anything further out than that, sit on a shell this far from
// the camera (metres). It has to be beyond Bob's own radius (600 km) so a
// camera standing on the planet is inside it, and small enough that its
// coordinates stay exact in float32. 20,000 km is both.
#define SKY_R       2.0e7f
#define SKY_INNER   1.6e7f  // distant bodies are pulled in to here, at their true angular size

// SE_SCENE_POINT_CAP is 1,024; 600 leaves room for sparks and markers.
#define STAR_N 600

// Draw ranges for the three settings presets, in km. 3,000 km is "the
// planet I am standing on"; 40,000 km puts Cheese (12,000 km out) in
// range from a parking orbit; the last one is the whole system.
static float const VIEW_KM[3] = {3000.0f, 40000.0f, 2000000.0f};

// --- What the bodies look like -----------------------------------------

// Base colours; the light does the rest. Bob is land-and-sea green, Cheese
// is the grey of a rock, Frodo is drawn emissive because it is the light.
#define BOB_COL     0xff3f7a52u
#define BOB_CAP     0xffc8d8e0u
#define CHEESE_COL  0xff8d8f96u
#define FRODO_COL   0xffffd27au
#define PAD_COL     0xff7fe0ffu

// The sun's drawn angular radius, in radians. Frodo's true one, 2,616 km
// at 13.6 million km, is 0.00019 rad -- a tenth of a pixel. This is the
// number that makes it a disc you can navigate by, and it is a drawing
// choice, not a physical one.
#define SUN_ANG_R 0.030f

// --- LOD ladder (scene.h) ----------------------------------------------

// bands x segments of quads, picked by how many pixels across the disc is.
// The top rung is 48x24 = 2,304 quads before the facing test halves it,
// which is the largest single object the renderer is willing to spend a
// frame on (SE_SCENE_TRI_CAP is 4,096).
#define LOD_RUNGS 6
static int const LOD_RINGS[LOD_RUNGS] = {40, 28, 20, 14, 10, 6};
static int const LOD_SEGS[LOD_RUNGS]  = {28, 20, 14, 10, 8, 6};
#define LOD_MAX_RINGS 40
#define LOD_MAX_SEGS  28

static int lod_for(float projected_px) {
    if (projected_px >= 220.0f) return 0;
    if (projected_px >= 110.0f) return 1;
    if (projected_px >= 55.0f) return 2;
    if (projected_px >= 28.0f) return 3;
    if (projected_px >= 14.0f) return 4;
    return 5;
}

// --- The starfield -------------------------------------------------------

static float s_star_dir[STAR_N][3];
static uint32_t s_star_col[STAR_N];

float nisp_view_range(int preset) {
    if (preset < 0) preset = 0;
    if (preset >= (int)(sizeof(VIEW_KM) / sizeof(VIEW_KM[0]))) preset = (int)(sizeof(VIEW_KM) / sizeof(VIEW_KM[0])) - 1;
    return VIEW_KM[preset];
}

void nisp_scene_init(uint32_t seed) {
    rng_init(seed);
    for (int i = 0; i < STAR_N; i++) {
        // A uniform direction on the shell: z in [-1,1] and an angle, the
        // standard construction. The shell rides with the camera, so the
        // stars parallax exactly as much as stars should: not at all.
        float const u = rng_unit() * 2.0f - 1.0f;
        float const a = rng_unit() * TAU;
        float const s = sqrtf(1.0f - u * u);
        s_star_dir[i][0] = s * cosf(a);
        s_star_dir[i][1] = u;
        s_star_dir[i][2] = s * sinf(a);

        uint32_t const r = rng_u32();
        int const b = 140 + (int)(r % 116);
        switch (r >> 30) {
            case 0: s_star_col[i] = 0xff000000u | (uint32_t)b << 16 | (uint32_t)b << 8 | 0xffu; break;
            case 1: s_star_col[i] = 0xff000000u | 0xffu << 16 | (uint32_t)b << 8 | (uint32_t)b; break;
            default: s_star_col[i] = 0xff000000u | (uint32_t)b << 16 | (uint32_t)b << 8 | (uint32_t)b; break;
        }
    }
}

// --- The camera basis ----------------------------------------------------

// The engine's space is right-handed with +y up and +z forward, so the
// local frame maps onto it as (east, up, -north): east x up = -north, which
// is the engine's +z. Anything else mirrors the world, and a mirrored world
// turns the wrong way.
typedef struct {
    double east[3], up[3], north[3];
} cam_basis_t;

static void basis_at(nisp_camera_t const* cam, cam_basis_t* b) {
    nisp_local_frame(cam->frame_body, cam->pos_km, b->up, b->north, b->east);
}

// A frame-axis OFFSET in km to engine metres.
static void pt_engine(cam_basis_t const* b, double const d[3], float out[3]) {
    out[0] = (float)(d[0] * b->east[0] + d[1] * b->east[1] + d[2] * b->east[2]) * NISP_M_PER_KM;
    out[1] = (float)(d[0] * b->up[0] + d[1] * b->up[1] + d[2] * b->up[2]) * NISP_M_PER_KM;
    out[2] = -(float)(d[0] * b->north[0] + d[1] * b->north[1] + d[2] * b->north[2]) * NISP_M_PER_KM;
}

// A frame-axis DIRECTION, unchanged in length.
static void dir_engine(cam_basis_t const* b, double const d[3], float out[3]) {
    out[0] = (float)(d[0] * b->east[0] + d[1] * b->east[1] + d[2] * b->east[2]);
    out[1] = (float)(d[0] * b->up[0] + d[1] * b->up[1] + d[2] * b->up[2]);
    out[2] = -(float)(d[0] * b->north[0] + d[1] * b->north[1] + d[2] * b->north[2]);
}

// --- Where the camera is, and where it looks ---------------------------

void nisp_camera_aim(nisp_camera_t* cam, float const fwd[3], float const up[3]) {
    cam_basis_t b;
    basis_at(cam, &b);

    double const f[3] = {fwd[0], fwd[1], fwd[2]};
    double const u[3] = {up[0], up[1], up[2]};
    float e[3], s[3];
    dir_engine(&b, f, e);
    dir_engine(&b, u, s);

    float const len = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    if (len < 1e-9f) {
        cam->yaw = cam->pitch = cam->roll = 0.0f;
        return;
    }
    for (int i = 0; i < 3; i++) e[i] /= len;

    // The engine's forward at (yaw, pitch) is (sin yaw cos pitch, -sin
    // pitch, cos yaw cos pitch) (se_scene.c), so pitch is minus asin of the
    // up component -- positive pitch looks DOWN.
    float const ey = e[1] > 1.0f ? 1.0f : (e[1] < -1.0f ? -1.0f : e[1]);
    cam->pitch = asinf(-ey);
    cam->yaw = atan2f(e[0], e[2]);

    // Roll is what is left: the camera's right and up before it, and the
    // angle the wanted up vector makes with them.
    float const cy = cosf(cam->yaw), sy = sinf(cam->yaw);
    float const cp = cosf(cam->pitch), sp = sinf(cam->pitch);
    float const R[3] = {cy, 0.0f, -sy};
    float const F[3] = {sy * cp, -sp, cy * cp};
    float const U[3] = {F[1] * R[2] - F[2] * R[1], F[2] * R[0] - F[0] * R[2], F[0] * R[1] - F[1] * R[0]};
    cam->roll = atan2f(-(s[0] * R[0] + s[1] * R[1] + s[2] * R[2]), s[0] * U[0] + s[1] * U[1] + s[2] * U[2]);
}

void nisp_camera_chase(nisp_camera_t* cam, nisp_vessel_t const* v, double t, float back_m, float up_m) {
    (void)t;  // the ship is the clock; the camera is wherever the ship is

    float nose[3];
    nisp_vessel_nose(v, nose);

    double up[3], north[3], east[3];
    nisp_local_frame(v->body, v->pos_km, up, north, east);

    // The camera's up is the local vertical with the nose component taken
    // out, so the horizon stays level however the ship is pointed. Straight
    // up or straight down, that degenerates, and north takes over.
    double u[3];
    double dn = up[0] * nose[0] + up[1] * nose[1] + up[2] * nose[2];
    for (int i = 0; i < 3; i++) u[i] = up[i] - nose[i] * dn;
    if (sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]) < 0.1) {
        dn = north[0] * nose[0] + north[1] * nose[1] + north[2] * nose[2];
        for (int i = 0; i < 3; i++) u[i] = north[i] - nose[i] * dn;
    }
    double ul = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (ul < 1e-9) ul = 1.0;
    for (int i = 0; i < 3; i++) u[i] /= ul;

    cam->frame_body = v->body;
    for (int i = 0; i < 3; i++) {
        cam->pos_km[i] = v->pos_km[i] - nose[i] * (back_m * NISP_KM_PER_M) + u[i] * (up_m * NISP_KM_PER_M);
    }
    float const uf[3] = {(float)u[0], (float)u[1], (float)u[2]};
    nisp_camera_aim(cam, nose, uf);
}

// The title's standoff from Bob's centre, km. 2,600 km puts Bob's disc at
// 208 pixels across on an 800x480 screen -- big enough to read as a
// planet, small enough to leave the sky around it.
#define TITLE_R    2600.0
#define TITLE_SWING 0.61f  // radians off the Bob->Cheese line: puts Cheese in frame
#define TITLE_WOBBLE (TAU / 90.0f)  // one slow sway a minute and a half

void nisp_camera_title(nisp_camera_t* cam, double t) {
    double rel[3];
    nisp_body_relative(BODY_CHEESE, BODY_BOB, t, rel);
    double rl = sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);
    if (rl < 1.0) rl = 1.0;
    for (int i = 0; i < 3; i++) rel[i] /= rl;

    // Turn aside from the line to Cheese, about Bob's spin axis, so the
    // planet sits left of centre and its moon somewhere in the frame.
    float const a = TITLE_SWING;
    double p[3] = {cosf(a) * rel[0] + sinf(a) * rel[2], rel[1], -sinf(a) * rel[0] + cosf(a) * rel[2]};

    // A sway in the plane and out of it, so the terminator and the stars
    // move even though nothing here is actually in a hurry.
    double n[3] = {p[2], 0.0, -p[0]};
    double nl = sqrt(n[0] * n[0] + n[2] * n[2]);
    if (nl < 1e-9) {
        n[0] = 1.0; n[2] = 0.0; nl = 1.0;
    }
    n[0] /= nl; n[2] /= nl;
    double k[3] = {p[1] * n[2] - p[2] * n[1], p[2] * n[0] - p[0] * n[2], p[0] * n[1] - p[1] * n[0]};

    float const w = (float)t * TITLE_WOBBLE;
    double pos[3];
    for (int i = 0; i < 3; i++) {
        pos[i] = TITLE_R * (p[i] + 0.09 * sinf(w) * n[i] + 0.06 * cosf(w) * k[i]);
    }

    cam->frame_body = BODY_BOB;
    cam->pos_km[0] = pos[0];
    cam->pos_km[1] = pos[1];
    cam->pos_km[2] = pos[2];

    double fl = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
    if (fl < 1.0) fl = 1.0;
    float const fwd[3] = {(float)(-pos[0] / fl), (float)(-pos[1] / fl), (float)(-pos[2] / fl)};

    // Up is the ecliptic normal, squared off against the line of sight:
    // the planet's pole stays up, whatever the sway does.
    double u[3] = {0.0, 1.0, 0.0};
    double const d = u[1];
    u[1] -= d * fwd[1];
    double ul = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (ul < 1e-6) {
        u[0] = 0.0; u[1] = 0.0; u[2] = 1.0;
        ul = 1.0;
    }
    float const up[3] = {(float)(u[0] / ul), (float)(u[1] / ul), (float)(u[2] / ul)};

    nisp_camera_aim(cam, fwd, up);
}

// --- Spheres -------------------------------------------------------------

// Trig tables per rung: bands+1 latitudes and segs+1 longitudes, filled
// once per body per frame. Without them a 48x24 sphere is 2,304 quads of
// four trig calls each, which is the whole frame budget spent on sinf.
// Ring sines/cosines and segment cosines/sines, filled once per body per
// frame. A quad is then 8 multiplies and no trig at all, which matters on a
// badge where sinf is a function call and a full-disc Bob is a thousand quads.
static float s_ring_s[LOD_MAX_RINGS + 1], s_ring_d[LOD_MAX_RINGS + 1];
static float s_seg_c[LOD_MAX_SEGS], s_seg_s[LOD_MAX_SEGS];

static void fill_tables(int rings, int segs) {
    for (int k = 0; k <= rings; k++) {
        float const t = (float)k / (float)rings;
        float const th = 1.5707963f * t * t;
        s_ring_s[k] = sinf(th);
        s_ring_d[k] = cosf(th);
    }
    for (int j = 0; j < segs; j++) {
        float const f = (float)j / (float)segs * TAU;
        s_seg_c[j] = cosf(f);
        s_seg_s[j] = sinf(f);
    }
}

// A sphere at engine-space `c`, radius `r`, tessellated in rings around
// the point nearest the camera rather than in latitude bands. That is the
// tessellation a camera actually wants: the quads under you are metres
// across, the ones at the limb are hundreds of kilometres, and only the
// visible hemisphere is generated at all -- no facing test, no wasted
// submission slots, and a silhouette that is a `segs`-gon.
//
// Ring k sits at angle theta_k = (pi/2) * (k/rings)^2 from the near point,
// so the spacing grows quadratically outward: 40 rings over Bob put the
// first one 5.9 km wide and the last one at the limb.
static void submit_sphere(float const c[3], float r, int rings, int segs, uint32_t col, uint32_t flags) {
    float const dc = sqrtf(c[0] * c[0] + c[1] * c[1] * c[2] * c[2]);
    float const inv = 1.0f / (dc > 1e-3f ? dc : 1e-3f);
    float const w[3] = {-c[0] * inv, -c[1] * inv, -c[2] * inv};

    // Two axes spanning the plane the rings turn in. Whichever engine axis
    // is least like `w` becomes the reference, so the basis never degenerates.
    float ref[3] = {0.0f, 1.0f, 0.0f};
    if (fabsf(w[1]) > 0.9f) { ref[0] = 1.0f; ref[1] = 0.0f; }
    float a[3] = {w[1] * ref[2] - w[2] * ref[1], w[2] * ref[0] - w[0] * ref[2], w[0] * ref[1] - w[1] * ref[0]};
    float al = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (al < 1e-6f) al = 1.0f;
    a[0] /= al; a[1] /= al; a[2] /= al;
    float b[3] = {w[1] * a[2] - w[2] * a[1], w[2] * a[0] - w[0] * a[2], w[0] * a[1] - w[1] * a[0]};

    fill_tables(rings, segs);

    for (int k = 0; k < rings; k++) {
        float const s0 = s_ring_s[k], d0 = s_ring_d[k];
        float const s1 = s_ring_s[k + 1], d1 = s_ring_d[k + 1];

        for (int j = 0; j < segs; j++) {
            int const jn = (j + 1 == segs) ? 0 : j + 1;
            float const ca0 = s_seg_c[j] * s0, sa0 = s_seg_s[j] * s0;
            float const ca1 = s_seg_c[jn] * s0, sa1 = s_seg_s[jn] * s0;
            float const cb0 = s_seg_c[j] * s1, sb0 = s_seg_s[j] * s1;
            float const cb1 = s_seg_c[jn] * s1, sb1 = s_seg_s[jn] * s1;

            float const x10 = c[0] + r * (w[0] * d0 + a[0] * ca0 + b[0] * sa0);
            float const y10 = c[1] + r * (w[1] * d0 + a[1] * ca0 + b[1] * sa0);
            float const z10 = c[2] + r * (w[2] * d0 + a[2] * ca0 + b[2] * sa0);
            float const x11 = c[0] + r * (w[0] * d0 + a[0] * ca1 + b[0] * sa1);
            float const y11 = c[1] + r * (w[1] * d0 + a[1] * ca1 + b[1] * sa1);
            float const z11 = c[2] + r * (w[2] * d0 + a[2] * ca1 + b[2] * sa1);
            float const x01 = c[0] + r * (w[0] * d1 + a[0] * cb0 + b[0] * sb0);
            float const y01 = c[1] + r * (w[1] * d1 + a[1] * cb0 + b[1] * sb0);
            float const z01 = c[2] + r * (w[2] * d1 + a[2] * cb0 + b[2] * sb0);
            float const x12 = c[0] + r * (w[0] * d1 + a[0] * cb1 + b[0] * sb1);
            float const y12 = c[1] + r * (w[1] * d1 + a[1] * cb1 + b[1] * sb1);
            float const z12 = c[2] + r * (w[2] * d1 + a[2] * cb1 + b[2] * sb1);

            if (k == 0) {
                // The first ring is a point: a fan, not a quad.
                scene_tri(c[0] + r * w[0], c[1] + r * w[1], c[2] + r * w[2], x10, y10, z10, x11, y11, z11, col, flags);
                continue;
            }
            scene_tri(x10, y10, z10, x11, y11, z11, x01, y01, z01, col, flags);
            scene_tri(x11, y11, z11, x12, y12, z12, x01, y01, z01, col, flags);
        }
    }
}


// --- Stars, planets, the sun, the pad -----------------------------------

static void submit_stars(void) {
    for (int i = 0; i < STAR_N; i++) {
        scene_point(s_star_dir[i][0] * SKY_R, s_star_dir[i][1] * SKY_R, s_star_dir[i][2] * SKY_R, s_star_col[i]);
    }
}

static void submit_planet(cam_basis_t const* b, nisp_camera_t const* cam, int id, double t, float range_km) {
    nisp_body_t const* body = nisp_body(id);

    double rel[3];
    nisp_body_relative(id, cam->frame_body, t, rel);
    double d[3] = {rel[0] - cam->pos_km[0], rel[1] - cam->pos_km[1], rel[2] - cam->pos_km[2]};

    float c[3];
    pt_engine(b, d, c);
    float dist = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    float r = body->radius_km * NISP_M_PER_KM;

    if (dist - r > range_km * NISP_M_PER_KM) return;  // out of the settings' draw range
    if (dist <= r) return;                            // the camera is on or in it: nothing to draw

    // Further out than the shell: pull it in to the shell and shrink it by
    // the same factor. Same angular size, same pixels, no 1e10-metre
    // vertices for a float32 projection to chew on.
    if (dist > SKY_INNER) {
        float const k = SKY_INNER / dist;
        for (int i = 0; i < 3; i++) c[i] *= k;
        r *= k;
        dist = SKY_INNER;
    }

    float const px = RENDER_FOCAL_LEN * r / dist;
    if (px < 0.6f) return;  // less than a pixel of disc: a point would lie about it

    int const lod = lod_for(px);
    submit_sphere(c, r, LOD_RINGS[lod], LOD_SEGS[lod], (id == BODY_BOB) ? BOB_COL : CHEESE_COL, 0u);
}

static void submit_sun(cam_basis_t const* b, double const sun[3]) {
    float e[3];
    dir_engine(b, sun, e);
    float const len = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    if (len < 1e-6f) return;
    float const dist = SKY_INNER * 0.98f;
    float const c[3] = {e[0] / len * dist, e[1] / len * dist, e[2] / len * dist};
    submit_sphere(c, dist * SUN_ANG_R, LOD_RINGS[5], LOD_SEGS[5], FRODO_COL, SE_TRI_EMISSIVE);
}

// The pad: a ring and a mast, drawn on the surface it sits on. It is the
// only landmark on a featureless sphere, and it is what a launch is
// measured against, so it stays visible long after it is small.
static void submit_pad(cam_basis_t const* b, nisp_camera_t const* cam, double t) {
    nisp_body_t const* body = nisp_body(cam->frame_body);
    if (body->radius_km <= 0.0f) return;

    double pad[3];
    nisp_surface_point(cam->frame_body, t, NISP_PAD_LAT, NISP_PAD_LON, pad);
    double d[3] = {pad[0] - cam->pos_km[0], pad[1] - cam->pos_km[1], pad[2] - cam->pos_km[2]};
    float c[3];
    pt_engine(b, d, c);
    float const dist = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    if (dist > 200000.0f) return;  // 200 km: beyond that it is a pixel and not a landmark

    double pu[3], pn[3], pe[3];
    nisp_local_frame(cam->frame_body, pad, pu, pn, pe);
    float up[3], north[3], east[3];
    dir_engine(b, pu, up);
    dir_engine(b, pn, north);
    dir_engine(b, pe, east);

    float const R = 60.0f;
    for (int j = 0; j < 16; j++) {
        float const f0 = (float)j / 16.0f * TAU, f1 = (float)(j + 1) / 16.0f * TAU;
        float const x0 = c[0] + (east[0] * cosf(f0) + north[0] * sinf(f0)) * R + up[0];
        float const z0 = c[2] + (east[2] * cosf(f0) + north[2] * sinf(f0)) * R + up[2];
        float const y0 = c[1] + (east[1] * cosf(f0) + north[1] * sinf(f0)) * R + up[1];
        float const x1 = c[0] + (east[0] * cosf(f1) + north[0] * sinf(f1)) * R + up[0];
        float const y1 = c[1] + (east[1] * cosf(f1) + north[1] * sinf(f1)) * R + up[1];
        float const z1 = c[2] + (east[2] * cosf(f1) + north[2] * sinf(f1)) * R + up[2];
        scene_line(x0, y0, z0, x1, y1, z1, PAD_COL);
    }
    scene_line(c[0], c[1] + 2.0f, c[2], c[0] + up[0] * 40.0f, c[1] + up[1] * 40.0f + 2.0f, c[2] + up[2] * 40.0f, PAD_COL);
}

// --- The ship ----------------------------------------------------------

// Drawn with its TAIL at v->pos_km rather than its centre of mass.
// sim/surface.c treats a landed ship as a point ON the sphere, so this is
// the convention that makes a rocket on the pad stand on the pad instead of
// halfway through it; everywhere else the difference is a few metres on a
// screen that is measuring thousands of kilometres.
//
// One tube per stack row, six to eight segments each: a launcher is a
// hundred triangles, which is nothing next to the planet behind it. The
// rows come straight out of rocket/stack, so what is drawn is what the
// flight model is carrying -- a stage that has separated is not drawn
// because it is not in the stack any more.
#define SHIP_SEGS  8
#define SHIP_HULL  0xffb4bcc4u  // tank sides
#define SHIP_BOOST 0xff8c9298u  // solid boosters, a shade darker
#define SHIP_SEP   0xff545a62u  // the stage boundaries
#define SHIP_POD   0xffe2c892u  // the capsule, the part the mission is about
#define SHIP_BELL  0xff3f454du  // engine bells
#define FLAME_COL  0xffffc25au  // emissive: the light is not asked about it
#define CHUTE_COL  0xffe8f0f8u
#define AXIS_COL   0xff9fe8e8u

static void ship_ring(float const base[3], float const ax[3], float const u1[3], float const u2[3], float dist,
                      float r, float out[SHIP_SEGS][3]) {
    for (int j = 0; j < SHIP_SEGS; j++) {
        float const f    = (float)j / (float)SHIP_SEGS * TAU;
        float const c    = cosf(f) * r;
        float const s    = sinf(f) * r;
        float const along = dist;
        out[j][0] = base[0] + ax[0] * along + u1[0] * c + u2[0] * s;
        out[j][1] = base[1] + ax[1] * along + u1[1] * c + u2[1] * s;
        out[j][2] = base[2] + ax[2] * along + u1[2] * c + u2[2] * s;
    }
}

// A frustum between two rings measured along `ax` from `base`. A zero
// radius on either end is a fan, which is how a cone (a bell, a flame, a
// nose) gets drawn without a second function.
static void submit_tube(float const base[3], float const ax[3], float const u1[3], float const u2[3], float dA,
                        float rA, float dB, float rB, uint32_t col) {
    float ra[SHIP_SEGS][3], rb[SHIP_SEGS][3];
    ship_ring(base, ax, u1, u2, dA, rA, ra);
    ship_ring(base, ax, u1, u2, dB, rB, rb);

    for (int j = 0; j < SHIP_SEGS; j++) {
        int const k = (j + 1 == SHIP_SEGS) ? 0 : j + 1;
        if (rA <= 1e-4f) {
            scene_tri(base[0] + ax[0] * dA, base[1] + ax[1] * dA, base[2] + ax[2] * dA, rb[j][0], rb[j][1],
                      rb[j][2], rb[k][0], rb[k][1], rb[k][2], col, 0u);
            continue;
        }
        if (rB <= 1e-4f) {
            scene_tri(ra[j][0], ra[j][1], ra[j][2], ra[k][0], ra[k][1], ra[k][2], base[0] + ax[0] * dB,
                      base[1] + ax[1] * dB, base[2] + ax[2] * dB, col, 0u);
            continue;
        }
        scene_tri(ra[j][0], ra[j][1], ra[j][2], ra[k][0], ra[k][1], ra[k][2], rb[j][0], rb[j][1], rb[j][2], col, 0u);
        scene_tri(ra[k][0], ra[k][1], ra[k][2], rb[k][0], rb[k][1], rb[k][2], rb[j][0], rb[j][1], rb[j][2], col, 0u);
    }
}

static void submit_ship(cam_basis_t const* b, nisp_camera_t const* cam, nisp_vessel_t const* v) {
    nisp_stack_t const* s = &v->stack;
    if (s->count <= 0) return;

    // Vehicle axes into frame axes, then into engine axes. +z is the nose,
    // and the stack axis is local -z (flight/physics.h), so a part at stack
    // coordinate x sits at -x along the nose -- which is what `dist` below
    // unwinds: distance from the TAIL, measured toward the nose.
    float nose_f[3], nx_f[3], ny_f[3];
    nisp_vessel_nose(v, nose_f);
    float const ex[3] = {1.0f, 0.0f, 0.0f}, ey[3] = {0.0f, 1.0f, 0.0f};
    nisp_quat_rotate(&v->q, ex, nx_f);
    nisp_quat_rotate(&v->q, ey, ny_f);

    // dir_engine works in the sim's doubles; the attitude is a float
    // quaternion, so the three directions are widened on the way in.
    double const nose[3] = {nose_f[0], nose_f[1], nose_f[2]};
    double const nx[3] = {nx_f[0], nx_f[1], nx_f[2]};
    double const ny[3] = {ny_f[0], ny_f[1], ny_f[2]};

    float ax[3], u1[3], u2[3];
    dir_engine(b, nose, ax);
    dir_engine(b, nx, u1);
    dir_engine(b, ny, u2);

    float L = 0.0f, rmax = 0.5f;
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].x1 > L) L = s->part[i].x1;
        float const r = s->part[i].diameter_m * 0.5f;
        if (r > rmax) rmax = r;
    }

    double const d0[3] = {v->pos_km[0] - cam->pos_km[0], v->pos_km[1] - cam->pos_km[1],
                          v->pos_km[2] - cam->pos_km[2]};
    float base[3];
    pt_engine(b, d0, base);

    float const dist = sqrtf(base[0] * base[0] + base[1] * base[1] + base[2] * base[2]);
    if (RENDER_FOCAL_LEN * (rmax + L) < 0.7f * dist) return;  // under a pixel: a dot would lie

    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        float const r  = p->diameter_m * 0.5f;
        float const dA = L - p->x1;  // its tail end
        float const dB = L - p->x0;  // its nose end

        if (p->slot == NISP_PART_KIND_SEPARATOR) {
            submit_tube(base, ax, u1, u2, dA - 0.02f, r * 1.06f, dB + 0.02f, r * 1.06f, SHIP_SEP);
            continue;
        }

        if (p->slot == NISP_PART_KIND_BOOSTER) {
            // One row stands for n identical boosters (F-25), so it draws n
            // cylinders, hung on the shoulder of the stage they belong to.
            float shoulder = r;
            for (int j = 0; j < s->count; j++) {
                nisp_part_t const* q = &s->part[j];
                if (q->slot == NISP_PART_KIND_BODY && q->stage == p->stage) {
                    shoulder = q->diameter_m * 0.5f;
                    break;
                }
            }
            int const n = (p->engine_count > 0) ? p->engine_count : 2;
            for (int j = 0; j < n; j++) {
                float const f = (float)j / (float)n * TAU + TAU / (float)(2 * n);
                float off[3] = {u1[0] * cosf(f) * (shoulder + r) + u2[0] * sinf(f) * (shoulder + r),
                                u1[1] * cosf(f) * (shoulder + r) + u2[1] * sinf(f) * (shoulder + r),
                                u1[2] * cosf(f) * (shoulder + r) + u2[2] * sinf(f) * (shoulder + r)};
                float bo[3] = {base[0] + off[0], base[1] + off[1], base[2] + off[2]};
                submit_tube(bo, ax, u1, u2, dA, r, dB, r, SHIP_BOOST);
                submit_tube(bo, ax, u1, u2, dA, r * 0.85f, dA - r * 1.4f, 0.0f, SHIP_BELL);
                if (p->igniting)
                    submit_tube(bo, ax, u1, u2, dA - r * 1.4f, r * 0.6f, dA - r * 6.0f, 0.0f, FLAME_COL);
            }
            continue;
        }

        if (p->slot == NISP_PART_KIND_LEGS) {
            if (!nisp_stack_legs_deployed(s, p->stage)) continue;
            int const n = nisp_legs_row(s->legs_id[p->stage])->count;
            for (int j = 0; j < n; j++) {
                float const f = (float)j / (float)n * TAU;
                float const cx = u1[0] * cosf(f) + u2[0] * sinf(f);
                float const cy = u1[1] * cosf(f) + u2[1] * sinf(f);
                float const cz = u1[2] * cosf(f) + u2[2] * sinf(f);
                float const foot = r * 2.0f;
                scene_line(base[0] + ax[0] * dA + cx * r, base[1] + ax[1] * dA + cy * r,
                           base[2] + ax[2] * dA + cz * r, base[0] + ax[0] * (dA - 0.3f) + cx * foot,
                           base[1] + ax[1] * (dA - 0.3f) + cy * foot, base[2] + ax[2] * (dA - 0.3f) + cz * foot,
                           0xffc8d0d8u);
            }
            continue;
        }

        // A body: the capsule tapers to a nose, everything else is a tank.
        if (p->kind == NISP_KIND_CAPSULE) {
            submit_tube(base, ax, u1, u2, dA, r, dB, r, SHIP_POD);
            submit_tube(base, ax, u1, u2, dB, r, dB + r * 1.1f, 0.0f, SHIP_POD);
        } else {
            submit_tube(base, ax, u1, u2, dA, r, dB, r, SHIP_HULL);
        }

        if (p->engine != NISP_ENGINE_NONE && p->engine_count > 0) {
            int const n = p->engine_count;
            for (int j = 0; j < n; j++) {
                float off[3] = {0.0f, 0.0f, 0.0f};
                if (n > 1) {
                    float const f    = (float)j / (float)n * TAU;
                    float const lean = r * 0.55f;
                    off[0]           = u1[0] * cosf(f) * lean + u2[0] * sinf(f) * lean;
                    off[1]           = u1[1] * cosf(f) * lean + u2[1] * sinf(f) * lean;
                    off[2]           = u1[2] * cosf(f) * lean + u2[2] * sinf(f) * lean;
                }
                float be[3] = {base[0] + off[0], base[1] + off[1], base[2] + off[2]};
                float const bell = r * 1.2f;
                submit_tube(be, ax, u1, u2, dA, r * 0.30f, dA - bell, r * 0.62f, SHIP_BELL);
                if (p->igniting)
                    submit_tube(be, ax, u1, u2, dA - bell, r * 0.55f, dA - bell - r * 5.0f, 0.0f, FLAME_COL);
            }
        }
    }

    // The nose, and a stub past it: the one thing on this model that says
    // "that end". A line inside the hull would be depth-tested away, so it
    // starts at the tip.
    scene_line(base[0] + ax[0] * L, base[1] + ax[1] * L, base[2] + ax[2] * L, base[0] + ax[0] * (L + rmax * 2.0f),
               base[1] + ax[1] * (L + rmax * 2.0f), base[2] + ax[2] * (L + rmax * 2.0f), AXIS_COL);

    if (s->parachute) {
        float const rim = L + 4.0f + rmax * 2.0f;
        float const canopy = rmax * 4.0f + 6.0f;
        submit_tube(base, ax, u1, u2, rim, canopy, rim + canopy * 0.7f, 0.0f, CHUTE_COL);
        for (int j = 0; j < 4; j++) {
            float const f    = (float)j / 4.0f * TAU + TAU / 8.0f;
            float const cx   = u1[0] * cosf(f) + u2[0] * sinf(f);
            float const cy   = u1[1] * cosf(f) + u2[1] * sinf(f);
            float const cz   = u1[2] * cosf(f) + u2[2] * sinf(f);
            scene_line(base[0] + ax[0] * L, base[1] + ax[1] * L, base[2] + ax[2] * L, base[0] + ax[0] * rim + cx * canopy,
                       base[1] + ax[1] * rim + cy * canopy, base[2] + ax[2] * rim + cz * canopy, CHUTE_COL);
        }
    }
}

// --- The frame ---------------------------------------------------------

// The light is parked far along the sun's direction -- further out than the
// sky shell, so every face in the scene sees the same direction and the
// shading does not depend on how big the scene currently is. It is a point
// light with no falloff (se_light.h), which is exactly a sun at this scale.
#define SUN_LAMP_R 3.0e7f
#define SUN_BRIGHTNESS 0.85f

int64_t nisp_scene_render(pax_buf_t* fb, nisp_camera_t const* cam, nisp_vessel_t const* ship, int view_preset,
                          double t) {
    cam_basis_t b;
    basis_at(cam, &b);

    // Frodo's direction, from where the camera is. nisp_sun_direction()
    // would give the same vector in inertial axes; this is the same
    // subtraction the planets are drawn with, in the frame the camera is
    // already standing in.
    double rel[3];
    nisp_body_relative(BODY_FRODO, cam->frame_body, t, rel);
    double d[3] = {rel[0] - cam->pos_km[0], rel[1] - cam->pos_km[1], rel[2] - cam->pos_km[2]};
    double dl   = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (dl < 1e-6) dl = 1e-6;
    for (int i = 0; i < 3; i++) d[i] /= dl;

    float sun[3];
    dir_engine(&b, d, sun);
    se_light_set(&(se_light_t){.x = sun[0] * SUN_LAMP_R, .y = sun[1] * SUN_LAMP_R, .z = sun[2] * SUN_LAMP_R,
                               .brightness = SUN_BRIGHTNESS, .two_sided = true});

    // The camera is the origin: everything submitted is camera-relative
    // (scene.h), so the projection never sees a number the size of the
    // solar system.
    render_set_camera_6dof(0.0f, 0.0f, 0.0f, cam->yaw, cam->pitch, cam->roll);

    float const range = nisp_view_range(view_preset);

    prof_begin(PROF_SUBMIT);
    scene_begin(fb);
    submit_stars();
    submit_sun(&b, d);
    for (int id = 0; id < BODY_COUNT; id++) {
        if (id == BODY_FRODO) continue;  // the disc above is Frodo
        submit_planet(&b, cam, id, t, range);
    }
    submit_pad(&b, cam, t);
    if (ship != NULL) submit_ship(&b, cam, ship);
    prof_end(PROF_SUBMIT);

    prof_begin(PROF_PREPARE);
    scene_prepare(SE_RENDER_ZBUFFER);
    prof_end(PROF_PREPARE);

    prof_begin(PROF_RASTER);
    int64_t const t0 = esp_timer_get_time();
    scene_rasterize(SE_RENDER_ZBUFFER);
    int64_t const rast_us = esp_timer_get_time() - t0;
    prof_end(PROF_RASTER);

    return rast_us;
}

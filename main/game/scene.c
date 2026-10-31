// =====================================================================
//  Null Island Space Program  --  the island (see scene.h)
// =====================================================================

#include "game/scene.h"

#include <math.h>
#include <string.h>

#include "common/rng.h"
#include "esp_timer.h"
#include "se_light.h"
#include "se_scene.h"
#include "testkit/profile.h"

// --- The island's shape ---------------------------------------------------

// One quad of the heightmap, in world units. 4 m is coarse enough to read
// as terrain at the distances this is flown at and fine enough to stay
// well under the engine's triangle cap.
#define CELL      4.0f
// Half the span, in cells: the grid runs -12..12, so 25 samples an axis
// and 24x24 quads -- 1152 triangles, against SE_SCENE_TRI_CAP of 4096.
#define HALF_CELLS 12
#define GRID_N     (HALF_CELLS * 2 + 1)
#define ISLAND_R   ((float)HALF_CELLS * CELL)  // 48 m

// The flat pad the ship starts on, and the height the hills are scaled by.
#define PAD_R      9.0f
#define HILL_H     3.2f

// Stars: directions on a unit shell, drawn far out so they parallax like
// they should. SE_SCENE_POINT_CAP is 1024; 600 leaves room for sparks.
#define STAR_N 600
#define STAR_R 20000.0f

// The survey markers. Placed by the seed, well clear of the pad.
#define BEACON_N 6

// The sun: a direction, not a position. The engine's light is directional
// with no falloff, so it is carried along with the camera (se_light.h).
#define SUN_X (-0.42f)
#define SUN_Y (0.80f)
#define SUN_Z (-0.50f)
#define SUN_BRIGHTNESS 0.55f

// math.h's M_PI is a GNU-ism rather than C, and this is the only place
// the constant is wanted.
#define TAU 6.2831853071795865f

static float s_h[GRID_N][GRID_N];
static float s_star_dir[STAR_N][3];
static uint32_t s_star_col[STAR_N];
static float s_beacon[BEACON_N][3];
static uint32_t s_beacon_col[BEACON_N];
static uint32_t s_seed;

static float grid_x(int i) {
    return (float)(i - HALF_CELLS) * CELL;
}

void nisp_scene_init(uint32_t seed) {
    s_seed = seed;
    rng_init(seed);

    // Raw value noise, then a box blur: two passes is enough for hills
    // that read as ground rather than as noise, and it runs once a world.
    for (int i = 0; i < GRID_N; i++) {
        for (int j = 0; j < GRID_N; j++) {
            s_h[i][j] = rng_unit();
        }
    }
    static float blurred[GRID_N][GRID_N];
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < GRID_N; i++) {
            for (int j = 0; j < GRID_N; j++) {
                float sum = 0.0f;
                int n = 0;
                for (int di = -1; di <= 1; di++) {
                    for (int dj = -1; dj <= 1; dj++) {
                        int const a = i + di, b = j + dj;
                        if (a < 0 || a >= GRID_N || b < 0 || b >= GRID_N) continue;
                        sum += s_h[a][b];
                        n++;
                    }
                }
                blurred[i][j] = sum / (float)n;
            }
        }
        memcpy(s_h, blurred, sizeof(s_h));
    }
    // Scale to metres, and flatten the pad: the ship starts on it and the
    // obelisk stands on it, so neither may be on a slope.
    for (int i = 0; i < GRID_N; i++) {
        for (int j = 0; j < GRID_N; j++) {
            float const x = grid_x(i), z = grid_x(j);
            float const r = sqrtf(x * x + z * z);
            float h = (s_h[i][j] - 0.5f) * HILL_H;
            if (r <= PAD_R) {
                h = 0.0f;
            } else if (r < PAD_R + CELL) {
                float const k = (r - PAD_R) / CELL;
                h *= k;
            }
            s_h[i][j] = h;
        }
    }

    for (int i = 0; i < STAR_N; i++) {
        // A uniform direction on the shell: z in [-1,1] and an angle, the
        // standard construction.
        float const u = rng_unit() * 2.0f - 1.0f;
        float const a = rng_unit() * 2.0f * TAU;
        float const s = sqrtf(1.0f - u * u);
        s_star_dir[i][0] = s * cosf(a);
        s_star_dir[i][1] = u;
        s_star_dir[i][2] = s * sinf(a);
        // Mostly white, a few warm and a few cold; the brightness carries
        // the magnitude, so a dim star is a dim pixel rather than absent.
        uint32_t const r = rng_u32();
        int const b = 140 + (int)(r % 116);
        uint32_t col;
        switch (r >> 30) {
            case 0: col = 0xff000000u | (uint32_t)b << 16 | (uint32_t)b << 8 | 0xffu; break;  // cold
            case 1: col = 0xff000000u | 0xffu << 16 | (uint32_t)b << 8 | (uint32_t)b; break;  // warm
            default: col = 0xff000000u | (uint32_t)b << 16 | (uint32_t)b << 8 | (uint32_t)b; break;
        }
        s_star_col[i] = col;
    }

    for (int i = 0; i < BEACON_N; i++) {
        // On a ring round the pad, jittered, so the markers are spread
        // about the island rather than bunched on one side of it.
        float const a = ((float)i + rng_unit() * 0.6f) * (2.0f * TAU / (float)BEACON_N);
        float const r = PAD_R + 8.0f + rng_unit() * (ISLAND_R - PAD_R - 12.0f);
        float const x = cosf(a) * r, z = sinf(a) * r;
        s_beacon[i][0] = x;
        s_beacon[i][1] = nisp_scene_ground(x, z);
        s_beacon[i][2] = z;
        s_beacon_col[i] = (i & 1) ? 0xff00d0ffu : 0xffffc040u;
    }
}

float nisp_view_range(int preset) {
    switch (preset) {
        case 0: return 36.0f;   // near
        case 2: return 120.0f;  // far
        default: return 70.0f;  // medium
    }
}

float nisp_scene_ground(float x, float z) {
    // Bilinear over the heightmap, clamped at the edge: outside the island
    // there is no ground, and the last row is what the edge looks like.
    float const fi = (x / CELL) + (float)HALF_CELLS;
    float const fj = (z / CELL) + (float)HALF_CELLS;
    int i0 = (int)floorf(fi), j0 = (int)floorf(fj);
    if (i0 < 0) i0 = 0;
    if (j0 < 0) j0 = 0;
    if (i0 > GRID_N - 2) i0 = GRID_N - 2;
    if (j0 > GRID_N - 2) j0 = GRID_N - 2;
    float tx = fi - (float)i0, tz = fj - (float)j0;
    if (tx < 0.0f) tx = 0.0f;
    if (tz < 0.0f) tz = 0.0f;
    if (tx > 1.0f) tx = 1.0f;
    if (tz > 1.0f) tz = 1.0f;
    float const h00 = s_h[i0][j0], h10 = s_h[i0 + 1][j0];
    float const h01 = s_h[i0][j0 + 1], h11 = s_h[i0 + 1][j0 + 1];
    return (h00 * (1.0f - tx) + h10 * tx) * (1.0f - tz) + (h01 * (1.0f - tx) + h11 * tx) * tz;
}

float nisp_scene_mark_range(float x, float z) {
    return sqrtf(x * x + z * z);
}

// --- Submitting a frame ---------------------------------------------------

// The island's colour from how high a quad sits: dark basalt in the low
// ground, pale rock on the ridges. Chosen per quad, not per vertex -- a
// triangle is flat by construction (se_scene.h).
static uint32_t ground_col(float h) {
    float const k = (h + HILL_H * 0.5f) / HILL_H;
    int const g = 70 + (int)(k * 90.0f);
    if (g < 40) return 0xff2a3a3au;
    if (g > 160) return 0xffa8b0b8u;
    return 0xff000000u | (uint32_t)(g - 20) << 16 | (uint32_t)g << 8 | (uint32_t)(g + 10);
}

static void submit_stars(nisp_pose_t const* pose) {
    // A star "at infinity" is the camera position plus a fixed direction
    // times a large distance (se_scene.h), so the shell moves with the ship.
    for (int i = 0; i < STAR_N; i++) {
        scene_point(pose->x + s_star_dir[i][0] * STAR_R, pose->y + s_star_dir[i][1] * STAR_R,
                    pose->z + s_star_dir[i][2] * STAR_R, s_star_col[i]);
    }
}

static void submit_ground(nisp_pose_t const* pose, float range) {
    float const range2 = range * range;
    for (int i = 0; i < GRID_N - 1; i++) {
        float const x0 = grid_x(i), x1 = grid_x(i + 1);
        if ((x0 - pose->x) * (x0 - pose->x) > range2 && (x1 - pose->x) * (x1 - pose->x) > range2) continue;
        for (int j = 0; j < GRID_N - 1; j++) {
            float const z0 = grid_x(j), z1 = grid_x(j + 1);
            float const dz = (z0 + z1) * 0.5f - pose->z;
            float const dx = (x0 + x1) * 0.5f - pose->x;
            if (dx * dx + dz * dz > range2) continue;

            float const h00 = s_h[i][j], h10 = s_h[i + 1][j], h01 = s_h[i][j + 1], h11 = s_h[i + 1][j + 1];
            uint32_t const col = ground_col((h00 + h10 + h01 + h11) * 0.25f);
            // Both windings are not needed: the light is two-sided, so
            // whichever face the camera sees is the face that gets lit.
            scene_tri(x0, h00, z0, x1, h10, z1, x0, h01, z1, col, 0u);
            scene_tri(x0, h00, z0, x1, h10, z0, x1, h10, z1, col, 0u);
        }
    }
}

// The survey grid, drawn on the pad's plane. An edge is never lit and
// never writes depth, so it sits over whatever it crosses (se_scene.h).
static void submit_grid(void) {
    uint32_t const col = 0xff1f8f8fu;
    for (int k = -HALF_CELLS; k <= HALF_CELLS; k += 2) {
        float const c = (float)k * CELL;
        scene_line(c, 0.02f, -ISLAND_R, c, 0.02f, ISLAND_R, col);
        scene_line(-ISLAND_R, 0.02f, c, ISLAND_R, 0.02f, c, col);
    }
}

// A box pylon: four sides and a lit cap. `half` is its half-width, `top`
// its height above `base`.
static void submit_pylon(float x, float base, float z, float half, float top, uint32_t col, uint32_t cap) {
    float const x0 = x - half, x1 = x + half, z0 = z - half, z1 = z + half;
    float const y0 = base, y1 = base + top;
    scene_tri(x0, y0, z0, x1, y0, z1, x0, y0, z1, col, 0u);
    scene_tri(x0, y0, z0, x1, y0, z0, x1, y0, z1, col, 0u);
    scene_tri(x1, y0, z1, x0, y0, z0, x0, y1, z0, col, 0u);
    scene_tri(x1, y0, z1, x1, y1, z1, x0, y1, z0, col, 0u);
    scene_tri(x1, y0, z0, x0, y0, z1, x1, y1, z1, col, 0u);
    scene_tri(x1, y0, z0, x0, y0, z0, x0, y1, z0, col, 0u);
    scene_tri(x0, y0, z1, x1, y0, z0, x1, y1, z1, col, 0u);
    scene_tri(x0, y0, z1, x0, y1, z1, x1, y1, z1, col, 0u);
    scene_tri(x0, y1, z0, x1, y1, z0, x1, y1, z1, cap, SE_TRI_EMISSIVE);
    scene_tri(x0, y1, z0, x1, y1, z1, x0, y1, z1, cap, SE_TRI_EMISSIVE);
}

static void submit_markers(void) {
    for (int i = 0; i < BEACON_N; i++) {
        submit_pylon(s_beacon[i][0], s_beacon[i][1], s_beacon[i][2], 0.35f, 3.0f, 0xff505860u, s_beacon_col[i]);
        // The lamp itself, one bright pixel above the cap: a point is
        // unlit, so it stays the colour of the marker at any distance.
        scene_point(s_beacon[i][0], s_beacon[i][1] + 3.4f, s_beacon[i][2], s_beacon_col[i]);
    }
}

// The reason the program came here. Dark slab, lit top.
static void submit_obelisk(void) {
    submit_pylon(0.0f, nisp_scene_ground(0.0f, 0.0f), 0.0f, 1.4f, 14.0f, 0xff202428u, 0xff7fe0ffu);
    scene_line(0.0f, 14.2f, 0.0f, 0.0f, 20.0f, 0.0f, 0xff7fe0ffu);
}

int64_t nisp_scene_render(pax_buf_t* fb, nisp_pose_t const* pose, int view_preset, double t) {
    render_set_camera_6dof(pose->x, pose->y, pose->z, pose->yaw, pose->pitch, 0.0f);

    // Directional, no falloff: carried along with the camera so it is
    // always "the sun", never a lamp the ship can fly past (se_light.h).
    se_light_set(&(se_light_t){
        .x = pose->x + SUN_X * 5000.0f, .y = pose->y + SUN_Y * 5000.0f, .z = pose->z + SUN_Z * 5000.0f,
        .brightness = SUN_BRIGHTNESS, .two_sided = true,
    });

    float const range = nisp_view_range(view_preset);

    prof_begin(PROF_SUBMIT);
    scene_begin(fb);
    submit_stars(pose);
    submit_ground(pose, range);
    submit_grid();
    submit_markers();
    submit_obelisk();
    (void)t;  // nothing here moves yet; the clock is threaded through for when it does
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

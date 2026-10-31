// =====================================================================
//  Null Island Space Program  --  the HUD (see hud.h)
// =====================================================================

#include "game/hud.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "game/scene.h"
#include "se_config.h"  // DISPLAY_LOG_W / DISPLAY_LOG_H
#include "se_scene.h"
#include "se_text.h"
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

// Compass heading from the yaw, in degrees, 0 = +z. The engine's yaw
// grows towards the camera's right (se_scene.c), so the reading turns the
// same way a compass card does.
static int heading_deg(float yaw) {
    float const TAU = 6.2831853071795865f;
    float d = yaw * 180.0f / TAU;
    d = fmodf(d, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return (int)lroundf(d);
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

void hud_draw(pax_buf_t* fb, nisp_game_t const* game, nisp_pose_t const* pose, bool info, hud_stats_t const* stats) {
    if (fb == NULL || game == NULL || pose == NULL) return;

    char line[96];

    // Top left: which world this is.
    text(fb, 0xffffffffu, 20.0f, 14.0f, 12.0f, game->name);
    snprintf(line, sizeof(line), "seed %u", (unsigned)game->seed);
    text(fb, 0xff9fb0b8u, 13.0f, 14.0f, 36.0f, line);

    // Bottom left: the flight.
    float y = (float)DISPLAY_LOG_H - 96.0f;
    snprintf(line, sizeof(line), "x %+.1f  y %+.1f  z %+.1f", (double)pose->x, (double)pose->y, (double)pose->z);
    text(fb, 0xffd8e0e8u, 14.0f, 14.0f, y, line);
    snprintf(line, sizeof(line), "heading %d deg   alt %+.1f m", heading_deg(pose->yaw),
             (double)(pose->y - nisp_scene_ground(pose->x, pose->z)));
    text(fb, 0xffd8e0e8u, 14.0f, 14.0f, y + 20.0f, line);
    snprintf(line, sizeof(line), "mark %+.0f m", (double)nisp_scene_mark_range(pose->x, pose->z));
    text(fb, 0xff9fe8e8u, 14.0f, 14.0f, y + 40.0f, line);

    // Bottom right: how the ship and the frame are doing.
    snprintf(line, sizeof(line), "%.0f m flown", (double)game->flown);
    pax_vec2f sz = rendertext_size(NULL, 14.0f, line);
    text(fb, 0xffd8e0e8u, 14.0f, (float)DISPLAY_LOG_W - sz.x - 14.0f, y, line);
    if (stats != NULL) {
        snprintf(line, sizeof(line), "%.0f fps", (double)stats->fps);
        sz = rendertext_size(NULL, 14.0f, line);
        text(fb, 0xff9fb0b8u, 14.0f, (float)DISPLAY_LOG_W - sz.x - 14.0f, y + 20.0f, line);
    }

    if (!info) return;

    // The diagnostic block: everything a bug report needs quoted, in one
    // place, on the key that means "show me what this is".
    int tri = 0, line_n = 0, pt = 0;
    int64_t tri_us = 0, line_us = 0, pt_us = 0;
    scene_raster_stats(&tri, &line_n, &tri_us, &line_us);
    scene_point_stats(&pt, &pt_us);

    static char const* const view_name[] = {"near", "medium", "far"};
    int const v = settings_view();
    snprintf(line, sizeof(line), "slot %d  view %s (%.0f m)  %s-res", stats != NULL ? stats->slot : -1,
             (v >= 0 && v < 3) ? view_name[v] : "?", (double)nisp_view_range(v),
             (stats != NULL && stats->half) ? "half" : "full");
    text(fb, 0xffc0d0d8u, 13.0f, 14.0f, 60.0f, line);
    snprintf(line, sizeof(line), "tris %d  lines %d  points %d  rast %.1f ms", tri, line_n, pt,
             (double)((stats != NULL ? (float)stats->rast_us : (float)tri_us) / 1000.0f));
    text(fb, 0xffc0d0d8u, 13.0f, 14.0f, 78.0f, line);
}

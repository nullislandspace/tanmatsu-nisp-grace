// =====================================================================
//  Null Island Space Program  --  the title screen (see title.h)
// =====================================================================

#include "ui/title.h"

#include <math.h>
#include <stdio.h>

#include "common/build_id.h"
#include "se_config.h"
#include "se_text.h"
#include "ui/logo.h"

static float const TAU = 6.2831853071795865f;

// The orbit: slow enough to read the island, wide enough that the obelisk
// never leaves the frame, low enough that the ground fills the bottom of
// it. One revolution a little under two minutes.
#define ORBIT_R      34.0f
#define ORBIT_H       9.0f
#define ORBIT_BOB     2.0f
#define ORBIT_RATE  (TAU / 110.0)

void title_pose(double t, nisp_pose_t* out) {
    if (out == NULL) return;
    float const a = (float)t * ORBIT_RATE;
    float const r = ORBIT_R;
    float const h = ORBIT_H + ORBIT_BOB * sinf(a * 0.5f);

    out->x = cosf(a) * r;
    out->y = h;
    out->z = sinf(a) * r;

    // Looking at the origin. The engine's forward is
    // (sin yaw cos pitch, -sin pitch, cos yaw cos pitch) (se_scene.c), so
    // the yaw that points the horizontal part at (-cos a, -sin a) is
    // atan2 of those, and the pitch that drops the line of sight by the
    // camera's own height is atan(h / r) -- positive, because positive
    // pitch looks DOWN.
    out->yaw = atan2f(-cosf(a), -sinf(a));
    out->pitch = atan2f(h, r);
}

static void word(pax_buf_t* fb, uint32_t col, float h, float x, float y, char const* s) {
    rendertext_draw(fb, 0xff000000u, NULL, h, x + 2.0f, y + 2.0f, s);
    rendertext_draw(fb, col, NULL, h, x, y, s);
}

// Text centred on a vertical line through `x` rather than on the whole
// screen: the title's text block sits in the space left beside the logo,
// which is not the middle of the screen.
static void at(pax_buf_t* fb, uint32_t col, float h, float x, float y, char const* s) {
    pax_vec2f const sz = rendertext_size(NULL, h, s);
    word(fb, col, h, x - sz.x * 0.5f, y, s);
}

// Where the ball sits, and where the words have to make room for it.
#define LOGO_X 200.0f
#define LOGO_Y 215.0f

void title_logo(pax_buf_t* fb) {
    if (fb == NULL || !logo_ready()) return;
    logo_draw(fb, LOGO_X, LOGO_Y);
}

void title_draw(pax_buf_t* fb, double t) {
    if (fb == NULL) return;

    // The ball on the left, the name on the right. The logo is the app's
    // own artwork off the card (ui/logo.h); when it is missing the words
    // move over and fill the screen instead, so a card without the file
    // still looks like a title screen and not like a broken one.
    float const words_x = logo_ready() ? 590.0f : (float)DISPLAY_LOG_W * 0.5f;

    at(fb, 0xffffffffu, 52.0f, words_x, 120.0f, "NULL ISLAND");
    at(fb, 0xff7fe0ffu, 24.0f, words_x, 178.0f, "SPACE PROGRAM");
    at(fb, 0xff9fe8e8u, 19.0f, words_x, 218.0f, "To nowhere, and beyond!");

    if (APP_VERSION[0] != '\0') {
        char line[64];
        snprintf(line, sizeof(line), "version %s", APP_VERSION);
        at(fb, 0xff9fb0b8u, 14.0f, words_x, 258.0f, line);
    }

    // The build, in a corner, so a screenshot says which build made it
    // without taking room the menu bar needs along the bottom.
    char line[96];
    snprintf(line, sizeof(line), "%s  %s", APP_GIT_HASH, APP_BUILD_TIME);
    word(fb, 0xff5f6f78u, 12.0f, 14.0f, 12.0f, line);

    // What to press. The bar along the bottom (ui/menu.c) is what is up
    // here, and it shows where the cursor is; this only says what a key
    // does. It blinks, and a blink is a function of the clock like
    // everything else on this screen.
    if ((int)(t * 1.4) & 1) at(fb, 0xffd8e0e8u, 18.0f, words_x, 300.0f, "press ENTER");
}

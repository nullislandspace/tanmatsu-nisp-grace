// =====================================================================
//  Null Island Space Program  --  controls (see input.h)
// =====================================================================

#include "game/input.h"

#include <stdio.h>

#include "bsp/input.h"
#include "esp_log.h"
#include "gl_input.h"
#include "se_bindings.h"
#include "synthengine3d.h"
#include "ui/settings.h"

// How fast the cursor keys turn the view, radians a second. Brisk without
// being impossible to aim with. A mouse or a gyroscope will not use it.
#define LOOK_RATE 7.0f

static char const TAG[] = "nisp-input";

// The declared control set. The third column is the action's stable id,
// what settings.txt keys its binding by: it must never change once a
// build has shipped or everyone's bindings move.
//
// Nothing is bound to F1. The run loop answers F1 itself as "back to the
// launcher" (se_run.h, f1_exits) and would never let a binding see it.
static se_binding_def_t const BINDINGS[NISP_ACTION_COUNT] = {
    // The ids are the ones the template shipped with, and they stay: a
    // saved binding is keyed by id, so re-meaning a key is free and moving
    // an id is not (input.h).
    [NISP_FORWARD]     = {NISP_FORWARD, "Pitch down", "fwd", BSP_INPUT_SCANCODE_W},
    [NISP_BACK]        = {NISP_BACK, "Pitch up", "back", BSP_INPUT_SCANCODE_S},
    [NISP_LEFT]        = {NISP_LEFT, "Yaw left", "left", BSP_INPUT_SCANCODE_A},
    [NISP_RIGHT]       = {NISP_RIGHT, "Yaw right", "right", BSP_INPUT_SCANCODE_D},
    [NISP_UP]          = {NISP_UP, "Throttle up", "up", BSP_INPUT_SCANCODE_LEFTSHIFT},
    [NISP_DOWN]        = {NISP_DOWN, "Throttle down", "down", BSP_INPUT_SCANCODE_LEFTCTRL},
    [NISP_BOOST]       = {NISP_BOOST, "Big step", "boost", BSP_INPUT_SCANCODE_RIGHTSHIFT},
    [NISP_LOOK_UP]     = {NISP_LOOK_UP, "Look up", "lookup", BSP_INPUT_SCANCODE_ESCAPED_GREY_UP},
    [NISP_LOOK_DOWN]   = {NISP_LOOK_DOWN, "Look down", "lookdown", BSP_INPUT_SCANCODE_ESCAPED_GREY_DOWN},
    [NISP_LOOK_LEFT]   = {NISP_LOOK_LEFT, "Look left", "lookleft", BSP_INPUT_SCANCODE_ESCAPED_GREY_LEFT},
    [NISP_LOOK_RIGHT]  = {NISP_LOOK_RIGHT, "Look right", "lookright", BSP_INPUT_SCANCODE_ESCAPED_GREY_RIGHT},
    [NISP_PAUSE]       = {NISP_PAUSE, "Pause", "pause", BSP_INPUT_SCANCODE_ESC},
    [NISP_SAVE]        = {NISP_SAVE, "Save", "save", BSP_INPUT_SCANCODE_F2},
    [NISP_INFO]        = {NISP_INFO, "Show position", "info", BSP_INPUT_SCANCODE_BACKSPACE},
    [NISP_SCREENSHOT]  = {NISP_SCREENSHOT, "Screenshot", "screenshot", BSP_INPUT_SCANCODE_0},
    [NISP_ROLL_LEFT]   = {NISP_ROLL_LEFT, "Roll left", "rollleft", BSP_INPUT_SCANCODE_Q},
    [NISP_ROLL_RIGHT]  = {NISP_ROLL_RIGHT, "Roll right", "rollright", BSP_INPUT_SCANCODE_E},
    [NISP_STAGE]       = {NISP_STAGE, "Stage", "stage", BSP_INPUT_SCANCODE_SPACE},
    [NISP_CUT]         = {NISP_CUT, "Cut engines", "cut", BSP_INPUT_SCANCODE_X},
    [NISP_CHUTE]       = {NISP_CHUTE, "Parachute", "chute", BSP_INPUT_SCANCODE_C},
    [NISP_LEGS]        = {NISP_LEGS, "Landing legs", "legs", BSP_INPUT_SCANCODE_G},
    [NISP_SAS]         = {NISP_SAS, "Stability assist", "sas", BSP_INPUT_SCANCODE_T},
    [NISP_RCS]         = {NISP_RCS, "RCS", "rcs", BSP_INPUT_SCANCODE_R},
    [NISP_WARP_UP]     = {NISP_WARP_UP, "Warp faster", "warpup", BSP_INPUT_SCANCODE_F3},
    [NISP_WARP_DOWN]   = {NISP_WARP_DOWN, "Warp slower", "warpdown", BSP_INPUT_SCANCODE_F4},
    [NISP_MAP]         = {NISP_MAP, "Map mode", "map", BSP_INPUT_SCANCODE_M},
    [NISP_CONFIRM]     = {NISP_CONFIRM, "Confirm", "confirm", BSP_INPUT_SCANCODE_ENTER},
    [NISP_FOCUS]       = {NISP_FOCUS, "Cycle focus", "focus", BSP_INPUT_SCANCODE_TAB},
    [NISP_TIME_FASTER] = {NISP_TIME_FASTER, "Time forward", "timeup", BSP_INPUT_SCANCODE_ESCAPED_GREY_PGUP},
    [NISP_TIME_SLOWER] = {NISP_TIME_SLOWER, "Time back", "timedn", BSP_INPUT_SCANCODE_ESCAPED_GREY_PGDN},
};

// The navigation key a scancode ALSO arrives as, where there is one.
// Which of the two a keyboard produces depends on the keyboard -- the
// built-in one and a USB one through the graceloader do not agree about
// the cursor keys -- so a bound key is asked both ways. It follows the
// BINDING, not the action: bind Look up to W and the up arrow stops
// looking up.
static bsp_input_navigation_key_t nav_for(uint16_t sc) {
    switch (sc) {
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_UP: return BSP_INPUT_NAVIGATION_KEY_UP;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_DOWN: return BSP_INPUT_NAVIGATION_KEY_DOWN;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_LEFT: return BSP_INPUT_NAVIGATION_KEY_LEFT;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_RIGHT: return BSP_INPUT_NAVIGATION_KEY_RIGHT;
        case BSP_INPUT_SCANCODE_SPACE: return BSP_INPUT_NAVIGATION_KEY_SPACE_M;
        case BSP_INPUT_SCANCODE_TAB: return BSP_INPUT_NAVIGATION_KEY_TAB;
        case BSP_INPUT_SCANCODE_ENTER: return BSP_INPUT_NAVIGATION_KEY_RETURN;
        case BSP_INPUT_SCANCODE_BACKSPACE: return BSP_INPUT_NAVIGATION_KEY_BACKSPACE;
        case BSP_INPUT_SCANCODE_ESC: return BSP_INPUT_NAVIGATION_KEY_ESC;
        case BSP_INPUT_SCANCODE_F2: return BSP_INPUT_NAVIGATION_KEY_F2;
        case BSP_INPUT_SCANCODE_F3: return BSP_INPUT_NAVIGATION_KEY_F3;
        case BSP_INPUT_SCANCODE_F4: return BSP_INPUT_NAVIGATION_KEY_F4;
        case BSP_INPUT_SCANCODE_F5: return BSP_INPUT_NAVIGATION_KEY_F5;
        case BSP_INPUT_SCANCODE_F6: return BSP_INPUT_NAVIGATION_KEY_F6;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_PGUP: return BSP_INPUT_NAVIGATION_KEY_PGUP;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_PGDN: return BSP_INPUT_NAVIGATION_KEY_PGDN;
        default: return BSP_INPUT_NAVIGATION_KEY_NONE;
    }
}

_Static_assert(NISP_ACTION_COUNT <= SE_BINDINGS_MAX,
               "more actions than the engine keeps bindings for: se_bindings_init clamps, and the last "
               "ones would silently never fire");

// Which section each action is presented in (input.h). STORED AS SECTION+1,
// so a row this file forgot reads as 0 -- unassigned -- instead of landing
// silently in the first section, which is 0 and would never be noticed.
// input_init() reports any action that ends up with no section.
static uint8_t const SECTION[NISP_ACTION_COUNT] = {
    [NISP_FORWARD]     = NISP_SECTION_FLIGHT + 1,
    [NISP_BACK]        = NISP_SECTION_FLIGHT + 1,
    [NISP_LEFT]        = NISP_SECTION_FLIGHT + 1,
    [NISP_RIGHT]       = NISP_SECTION_FLIGHT + 1,
    [NISP_UP]          = NISP_SECTION_FLIGHT + 1,
    [NISP_DOWN]        = NISP_SECTION_FLIGHT + 1,
    [NISP_BOOST]       = NISP_SECTION_FLIGHT + 1,
    [NISP_ROLL_LEFT]   = NISP_SECTION_FLIGHT + 1,
    [NISP_ROLL_RIGHT]  = NISP_SECTION_FLIGHT + 1,
    [NISP_STAGE]       = NISP_SECTION_FLIGHT + 1,
    [NISP_CUT]         = NISP_SECTION_FLIGHT + 1,
    [NISP_CHUTE]       = NISP_SECTION_FLIGHT + 1,
    [NISP_LEGS]        = NISP_SECTION_FLIGHT + 1,
    [NISP_SAS]         = NISP_SECTION_FLIGHT + 1,
    [NISP_RCS]         = NISP_SECTION_FLIGHT + 1,

    [NISP_MAP]         = NISP_SECTION_MAP + 1,
    [NISP_WARP_UP]     = NISP_SECTION_MAP + 1,
    [NISP_WARP_DOWN]   = NISP_SECTION_MAP + 1,
    [NISP_TIME_FASTER] = NISP_SECTION_MAP + 1,
    [NISP_TIME_SLOWER] = NISP_SECTION_MAP + 1,

    [NISP_CONFIRM]     = NISP_SECTION_ENGINEERING + 1,
    [NISP_FOCUS]       = NISP_SECTION_ENGINEERING + 1,

    [NISP_LOOK_UP]     = NISP_SECTION_VIEW + 1,
    [NISP_LOOK_DOWN]   = NISP_SECTION_VIEW + 1,
    [NISP_LOOK_LEFT]   = NISP_SECTION_VIEW + 1,
    [NISP_LOOK_RIGHT]  = NISP_SECTION_VIEW + 1,

    [NISP_PAUSE]       = NISP_SECTION_SYSTEM + 1,
    [NISP_SAVE]        = NISP_SECTION_SYSTEM + 1,
    [NISP_INFO]        = NISP_SECTION_SYSTEM + 1,
    [NISP_SCREENSHOT]  = NISP_SECTION_SYSTEM + 1,
};

static char const* const SECTION_LABEL[NISP_SECTION_COUNT] = {
    "Flight", "Map and warp", "Engineering", "Camera", "System",
};

static nisp_actions_t s_last, s_pressed;

void input_init(void) {
    // nvs_namespace NULL: the engine keeps the bindings in memory and
    // settings.txt owns them on disk, where a player can back them up
    // with their saves (se_bindings.h).
    se_bindings_init(&(se_bindings_config_t){.nvs_namespace = NULL, .defs = BINDINGS, .count = NISP_ACTION_COUNT});

    int counts[NISP_SECTION_COUNT] = {0};
    int unassigned = 0;
    for (int a = 0; a < NISP_ACTION_COUNT; a++) {
        int const section = input_action_section((nisp_action_t)a);
        if (section < 0) unassigned++;
        else counts[section]++;
    }
    ESP_LOGI(TAG, "%d bindings: flight %d, map %d, engineering %d, camera %d, system %d", NISP_ACTION_COUNT,
             counts[NISP_SECTION_FLIGHT], counts[NISP_SECTION_MAP], counts[NISP_SECTION_ENGINEERING],
             counts[NISP_SECTION_VIEW], counts[NISP_SECTION_SYSTEM]);
    if (unassigned > 0)
        ESP_LOGW(TAG, "%d actions have no Controls section: they cannot be rebound", unassigned);
}

static bool held(uint16_t sc) {
    bool state = false;
    if (sc == 0 || gl_input_read_scancode((bsp_input_scancode_t)sc, &state) != ESP_OK) state = false;
    if (state) return true;
    bsp_input_navigation_key_t const nav = nav_for(sc);
    if (nav == BSP_INPUT_NAVIGATION_KEY_NONE) return false;
    if (gl_input_read_navigation_key(nav, &state) != ESP_OK) return false;
    return state;
}

nisp_actions_t input_sample(void) {
    nisp_actions_t mask = 0;
    for (int a = 0; a < NISP_ACTION_COUNT; a++) {
        if (held(se_bindings_get(a))) mask |= (nisp_actions_t)1u << a;
    }
    s_pressed = mask & ~s_last;
    s_last = mask;
    return mask;
}

nisp_actions_t input_pressed(void) {
    return s_pressed;
}

void input_look(nisp_actions_t mask, float* dyaw, float* dpitch) {
    // The engine's convention, which is not the obvious one (se_scene.c,
    // camera_build_basis): forward = (sin yaw cos pitch, -sin pitch,
    // cos yaw cos pitch). So yaw grows towards the right, and POSITIVE
    // PITCH LOOKS DOWN.
    float x = 0.0f, y = 0.0f;
    if (act_held(mask, NISP_LOOK_LEFT)) x -= 1.0f;
    if (act_held(mask, NISP_LOOK_RIGHT)) x += 1.0f;
    if (act_held(mask, NISP_LOOK_UP)) y -= 1.0f;
    if (act_held(mask, NISP_LOOK_DOWN)) y += 1.0f;
    if (dyaw != NULL) *dyaw = x * LOOK_RATE;
    if (dpitch != NULL) *dpitch = y * LOOK_RATE;
}

char const* input_action_label(nisp_action_t a) {
    if (a < 0 || a >= NISP_ACTION_COUNT) return "?";
    return BINDINGS[a].label;
}

char const* input_action_id(nisp_action_t a) {
    if (a < 0 || a >= NISP_ACTION_COUNT) return "?";
    // The label column is the engine's; this is the file's. Both live in
    // BINDINGS so they cannot drift apart.
    return BINDINGS[a].nvs_key;
}

uint16_t input_key(nisp_action_t a) {
    if (a < 0 || a >= NISP_ACTION_COUNT) return 0;
    return se_bindings_get(a);
}

uint16_t input_default_key(nisp_action_t a) {
    if (a < 0 || a >= NISP_ACTION_COUNT) return 0;
    return BINDINGS[a].default_sc;
}

void input_bind(nisp_action_t a, uint16_t sc) {
    if (a < 0 || a >= NISP_ACTION_COUNT || sc == 0) return;
    uint16_t const old = se_bindings_get(a);
    if (old == sc) return;
    for (int b = 0; b < NISP_ACTION_COUNT; b++) {
        if (b != (int)a && se_bindings_get(b) == sc) se_bindings_set(b, old);
    }
    se_bindings_set(a, sc);
    settings_save();
}

void input_reset_defaults(void) {
    for (int a = 0; a < NISP_ACTION_COUNT; a++) se_bindings_set(a, BINDINGS[a].default_sc);
    settings_save();
}

char const* input_key_name(uint16_t sc, char* buf, int cap) {
    switch (sc) {
        case BSP_INPUT_SCANCODE_W: snprintf(buf, (size_t)cap, "%s", "W"); return buf;
        case BSP_INPUT_SCANCODE_A: snprintf(buf, (size_t)cap, "%s", "A"); return buf;
        case BSP_INPUT_SCANCODE_S: snprintf(buf, (size_t)cap, "%s", "S"); return buf;
        case BSP_INPUT_SCANCODE_D: snprintf(buf, (size_t)cap, "%s", "D"); return buf;
        case BSP_INPUT_SCANCODE_Q: snprintf(buf, (size_t)cap, "%s", "Q"); return buf;
        case BSP_INPUT_SCANCODE_E: snprintf(buf, (size_t)cap, "%s", "E"); return buf;
        case BSP_INPUT_SCANCODE_G: snprintf(buf, (size_t)cap, "%s", "G"); return buf;
        case BSP_INPUT_SCANCODE_I: snprintf(buf, (size_t)cap, "%s", "I"); return buf;
        case BSP_INPUT_SCANCODE_C: snprintf(buf, (size_t)cap, "%s", "C"); return buf;
        case BSP_INPUT_SCANCODE_V: snprintf(buf, (size_t)cap, "%s", "V"); return buf;
        case BSP_INPUT_SCANCODE_B: snprintf(buf, (size_t)cap, "%s", "B"); return buf;
        case BSP_INPUT_SCANCODE_N: snprintf(buf, (size_t)cap, "%s", "N"); return buf;
        case BSP_INPUT_SCANCODE_X: snprintf(buf, (size_t)cap, "%s", "X"); return buf;
        case BSP_INPUT_SCANCODE_Z: snprintf(buf, (size_t)cap, "%s", "Z"); return buf;
        case BSP_INPUT_SCANCODE_SPACE: snprintf(buf, (size_t)cap, "%s", "Space"); return buf;
        case BSP_INPUT_SCANCODE_ENTER: snprintf(buf, (size_t)cap, "%s", "Enter"); return buf;
        case BSP_INPUT_SCANCODE_ESC: snprintf(buf, (size_t)cap, "%s", "Esc"); return buf;
        case BSP_INPUT_SCANCODE_TAB: snprintf(buf, (size_t)cap, "%s", "Tab"); return buf;
        case BSP_INPUT_SCANCODE_BACKSPACE: snprintf(buf, (size_t)cap, "%s", "Bksp"); return buf;
        case BSP_INPUT_SCANCODE_LEFTSHIFT: snprintf(buf, (size_t)cap, "%s", "L-Shift"); return buf;
        case BSP_INPUT_SCANCODE_RIGHTSHIFT: snprintf(buf, (size_t)cap, "%s", "R-Shift"); return buf;
        case BSP_INPUT_SCANCODE_LEFTCTRL: snprintf(buf, (size_t)cap, "%s", "L-Ctrl"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_RCTRL: snprintf(buf, (size_t)cap, "%s", "R-Ctrl"); return buf;
        case BSP_INPUT_SCANCODE_T: snprintf(buf, (size_t)cap, "%s", "T"); return buf;
        case BSP_INPUT_SCANCODE_R: snprintf(buf, (size_t)cap, "%s", "R"); return buf;
        case BSP_INPUT_SCANCODE_M: snprintf(buf, (size_t)cap, "%s", "M"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_UP: snprintf(buf, (size_t)cap, "%s", "Up"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_DOWN: snprintf(buf, (size_t)cap, "%s", "Down"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_LEFT: snprintf(buf, (size_t)cap, "%s", "Left"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_RIGHT: snprintf(buf, (size_t)cap, "%s", "Right"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_HOME: snprintf(buf, (size_t)cap, "%s", "Home"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_END: snprintf(buf, (size_t)cap, "%s", "End"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_PGUP: snprintf(buf, (size_t)cap, "%s", "PgUp"); return buf;
        case BSP_INPUT_SCANCODE_ESCAPED_GREY_PGDN: snprintf(buf, (size_t)cap, "%s", "PgDn"); return buf;
        default: break;
    }
    if (sc >= BSP_INPUT_SCANCODE_1 && sc <= BSP_INPUT_SCANCODE_0) {
        // The number row, in scan order: 1..9 then 0.
        int const d = (int)sc - (int)BSP_INPUT_SCANCODE_1 + 1;
        snprintf(buf, (size_t)cap, "%d", d == 10 ? 0 : d);
        return buf;
    }
    if (sc >= BSP_INPUT_SCANCODE_F1 && sc <= BSP_INPUT_SCANCODE_F12) {
        snprintf(buf, (size_t)cap, "F%d", sc - BSP_INPUT_SCANCODE_F1 + 1);
        return buf;
    }
    snprintf(buf, (size_t)cap, "0x%04x", (unsigned)sc);
    return buf;
}

int input_action_section(nisp_action_t a) {
    if (a < 0 || a >= NISP_ACTION_COUNT) return -1;
    uint8_t const packed = SECTION[a];
    return (packed == 0) ? -1 : (int)packed - 1;
}

char const* input_section_label(int section) {
    if (section < 0 || section >= NISP_SECTION_COUNT) return "?";
    return SECTION_LABEL[section];
}

int input_section_row_count(int section) {
    if (section < 0 || section >= NISP_SECTION_COUNT) return 0;
    int n = 0;
    for (int a = 0; a < NISP_ACTION_COUNT; a++) {
        if (input_action_section((nisp_action_t)a) == section) n++;
    }
    return n;
}

int input_section_action(int section, int row) {
    if (section < 0 || section >= NISP_SECTION_COUNT || row < 0) return -1;
    for (int a = 0; a < NISP_ACTION_COUNT; a++) {
        if (input_action_section((nisp_action_t)a) != section) continue;
        if (row == 0) return a;
        row--;
    }
    return -1;
}

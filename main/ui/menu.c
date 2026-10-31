// =====================================================================
//  Null Island Space Program  --  the menus (see menu.h)
// =====================================================================

#include "ui/menu.h"

#include <stdio.h>
#include <string.h>

#include "audio/music.h"
#include "common/rng.h"
#include "engineering/sheet.h"
#include "esp_timer.h"
#include "game/input.h"
#include "rocket/design.h"
#include "se_audio.h"
#include "se_config.h"
#include "se_direct565.h"
#include "se_hw.h"
#include "se_stream.h"
#include "se_text.h"
#include "se_ui.h"
#include "ui/settings.h"

// --- Which screen is up ---------------------------------------------------

typedef enum {
    SCR_NONE = 0,
    SCR_TITLE,     // the title bar: Play / Settings / Quit
    SCR_WORLDS,    // the slots
    SCR_WORLD,     // one slot: the Spaceport hub (Part N1)
    SCR_LAUNCH,    // which saved design leaves the pad
    SCR_DESIGNS,   // the blueprints on the card
    SCR_DESIGN,    // one blueprint: Engineering / Rename / Delete
    SCR_NEW,       // New world: name, seed, Create
    SCR_NAME,      // typing a name
    SCR_SEED,      // typing a seed
    SCR_DELETE,    // are you sure
    SCR_SETTINGS,  // Controls / Graphics / Audio / Display
    SCR_CONTROLS,
    SCR_GRAPHICS,
    SCR_AUDIO,
    SCR_DISPLAY,
    SCR_PAUSE,
    SCR_COUNT
} screen_t;

static screen_t s_scr = SCR_NONE;

// One cursor per screen, so going into Controls and back does not leave
// the cursor on whatever row was last highlighted somewhere else.
static int s_cursor[SCR_COUNT];

// What BACK leaves you on. Two of these depend on where you came from, so
// they are state rather than a table: Settings opens from the title bar or
// from the pause menu, and typing a name is reached both from New world and
// from Rename.
static screen_t s_settings_back = SCR_TITLE;
static screen_t s_edit_return = SCR_NEW;

// Whether this badge has a half-size layer to draw the scene into. main.c
// allocated one and knows; the Graphics screen would otherwise promise a
// picture change that cannot happen (menu_half_res_available).
static bool s_half_available = true;

static screen_t parent_of(screen_t s) {
    switch (s) {
        case SCR_WORLDS: return SCR_TITLE;
        case SCR_WORLD: return SCR_WORLDS;
        case SCR_NEW: return SCR_WORLDS;
        case SCR_LAUNCH:
        case SCR_DESIGNS: return SCR_WORLD;
        case SCR_DESIGN: return SCR_DESIGNS;
        case SCR_NAME:
        case SCR_SEED: return s_edit_return;
        case SCR_DELETE: return SCR_WORLD;
        case SCR_SETTINGS: return s_settings_back;
        case SCR_CONTROLS:
        case SCR_GRAPHICS:
        case SCR_AUDIO:
        case SCR_DISPLAY: return SCR_SETTINGS;
        default: return SCR_NONE;
    }
}

// --- The world the screens are about --------------------------------------

static int s_slot;      // the slot SCR_WORLD and SCR_DELETE are about
static int s_open_slot = -1;  // the slot main.c has open, -1 when it has none
static int s_new_slot;  // the slot SCR_NEW will write to
static char s_new_name[NISP_NAME_MAX];
static uint32_t s_new_seed;

// The blueprints on the card, read once when a design screen opens -- the
// same rule as the slots: a menu is not a place to re-read files per frame.
#define MAX_DESIGNS 16
static char s_designs[MAX_DESIGNS][NISP_DESIGN_NAME_MAX + 1];
static int s_design_n;
static int s_design_sel;

// The blueprints, read off the card. Sorted already (nisp_design_list).
static void refresh_designs(void) {
    s_design_n = nisp_design_list(s_designs, MAX_DESIGNS);
}
// What the text being typed is for. Four different things want a name, and
// only one of them is a world.
typedef enum {
    EDIT_NEW_WORLD = 0,  // the name a new slot will be created with
    EDIT_RENAME_WORLD,   // a slot that exists
    EDIT_RENAME_DESIGN,  // a blueprint that exists
    EDIT_WORKING_NAME    // the design Engineering is drawing (a command back out)
} edit_kind_t;
static edit_kind_t s_edit_kind = EDIT_NEW_WORLD;
static char s_edit_target[NISP_DESIGN_NAME_MAX + 1];  // what a rename is renaming

// The text being typed. Wider than a name so there is room to type past
// the limit and see it refused rather than watch characters vanish.
static char s_edit[48];

// Read once, when the slot list opens. A menu is not a place to re-read
// files every frame, and nothing else changes a slot while one is up.
static nisp_slot_t s_slot_info[NISP_SAVE_SLOTS];
static char s_slot_label[NISP_SAVE_SLOTS][64];
static char s_title[64];
static char s_subtitle[64];
static char s_seed_text[16];
static char s_key_name[NISP_ACTION_COUNT][12];

// --- News -----------------------------------------------------------------

static char s_status[96];
static int64_t s_status_until;

static void menu_status_set(char const* msg) {
    snprintf(s_status, sizeof(s_status), "%s", msg != NULL ? msg : "");
    s_status_until = esp_timer_get_time() + 2500000;
}

static char const* status_line(void) {
    return (s_status[0] != '\0' && esp_timer_get_time() < s_status_until) ? s_status : NULL;
}

// --- The menu the engine draws --------------------------------------------

#define MAX_ROWS 32  // the controls screen is one row per action, plus a reset row
static se_menu_row_t s_rows[MAX_ROWS];
static se_menu_def_t s_def;
static se_menu_t s_menu;

// One layout for every screen, so they look like one menu system and not
// like a pile of screens. begin() sets it; the odd screen that needs more
// room (Controls) overrides the two fields that matter.
static void begin(char const* title, char const* subtitle, char const* hint) {
    s_def = (se_menu_def_t){
        .title       = title,
        // News, when there is any, replaces the subtitle: it is the more
        // useful of the two, and it is the one that says whether the thing
        // you just pressed worked.
        .subtitle    = status_line() != NULL ? s_status : subtitle,
        .rows        = s_rows,
        .row_count   = 0,
        .hint        = hint,
        .title_h     = 30.0f,
        .row_h       = 40.0f,
        .value_dx    = 260.0f,
        .panel_w     = 0.78f,
        .panel_h     = 0.84f,
        .visible_rows = 0,
    };
}

// --- The screens ----------------------------------------------------------

static int build_worlds(void) {
    begin("Play", "A world is a seed and a pose. Three slots.", "up/down choose   enter opens   esc back");
    for (int i = 0; i < NISP_SAVE_SLOTS; i++) {
        if (s_slot_info[i].exists) {
            char stamp[24];
            snprintf(s_slot_label[i], sizeof(s_slot_label[i]), "%d. %s  (%s)", i + 1, s_slot_info[i].name,
                     save_stamp(stamp, sizeof(stamp), s_slot_info[i].timestamp));
        } else {
            snprintf(s_slot_label[i], sizeof(s_slot_label[i]), "%d. empty", i + 1);
        }
        s_rows[i] = (se_menu_row_t){.label = s_slot_label[i], .kind = SE_MENU_VAL_NONE};
    }
    s_rows[NISP_SAVE_SLOTS] = (se_menu_row_t){.label = "New world", .kind = SE_MENU_VAL_NONE};
    return NISP_SAVE_SLOTS + 1;
}

// The Spaceport hub (Part N1). Verbs, never verbs-with-arguments (D-24):
// the rows that need a parameter open a picker, so this screen is seven
// rows long no matter how many blueprints the player has.
static int build_world(void) {
    nisp_slot_t const* info = &s_slot_info[s_slot];
    refresh_designs();
    snprintf(s_title, sizeof(s_title), "Spaceport  --  slot %d", s_slot + 1);
    snprintf(s_subtitle, sizeof(s_subtitle), "%s  --  %d blueprint%s", info->info, s_design_n,
             s_design_n == 1 ? "" : "s");
    begin(s_title, s_subtitle, "enter chooses   esc back");
    s_rows[0] = (se_menu_row_t){.label = "Continue mission", .kind = SE_MENU_VAL_NONE};
    s_rows[1] = (se_menu_row_t){.label = "Build a rocket", .kind = SE_MENU_VAL_NONE};
    s_rows[2] = (se_menu_row_t){.label = "Launch...", .kind = SE_MENU_VAL_NONE};
    s_rows[3] = (se_menu_row_t){.label = "Designs...", .kind = SE_MENU_VAL_NONE};
    s_rows[4] = (se_menu_row_t){.label = "Rename", .kind = SE_MENU_VAL_NONE};
    s_rows[5] = (se_menu_row_t){.label = "Delete", .kind = SE_MENU_VAL_NONE};
    s_rows[6] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    return 7;
}

// A row's value column is a little blueprint: the same sheet call the
// Engineering screen uses, in a 34x34 box (Part B5). A design looks
// identical here and on the drawing it was named after.
static void draw_design_thumb(pax_buf_t* fb, float x, float y, float h, pax_col_t col, void* ctx) {
    (void)col;
    char const* const name = (char const*)ctx;
    nisp_design_t design;
    if (name == NULL || !nisp_design_load(name, &design)) return;

    int const box = 34;
    int const bx = (int)x, by = (int)(y + (40.0f - (float)box) * 0.5f);
    nisp_rect_t const r = {bx, by, box, box};

    uint16_t* const px = (uint16_t*)pax_buf_get_pixels(fb);
    uint16_t const paper = direct_565_pack_for(fb, NISP_BLUEPRINT_ARGB);
    for (int i = 0; i < box; i++) direct_565_vrun(px, bx + i, by, by + box - 1, paper);
    uint16_t const edge = direct_565_pack_for(fb, NISP_PENCIL_DIM_ARGB);
    direct_565_line(px, bx, by, bx + box - 1, by, edge);
    direct_565_line(px, bx, by + box - 1, bx + box - 1, by + box - 1, edge);
    direct_565_line(px, bx, by, bx, by + box - 1, edge);
    direct_565_line(px, bx + box - 1, by, bx + box - 1, by + box - 1, edge);

    nisp_sheet_thumb(fb, r, &design);
}

// Which design leaves the pad. One row per blueprint, each with its
// thumbnail, plus a way out. An empty list says so rather than showing a
// panel with nothing in it.
static int build_launch(void) {
    refresh_designs();
    snprintf(s_title, sizeof(s_title), "Launch");
    snprintf(s_subtitle, sizeof(s_subtitle), "Which blueprint leaves the pad?");
    begin(s_title, s_subtitle, "enter launches   esc back");
    if (s_design_n == 0) {
        s_rows[0] = (se_menu_row_t){.label = "Nothing built yet", .kind = SE_MENU_VAL_NONE};
        return 1;
    }
    for (int i = 0; i < s_design_n; i++) {
        s_rows[i] = (se_menu_row_t){.label = s_designs[i], .kind = SE_MENU_VAL_CUSTOM,
                                    .draw_value = draw_design_thumb, .ctx = s_designs[i]};
    }
    s_rows[s_design_n] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    s_def.visible_rows = 6;  // six 40 px rows is what the panel has room for
    return s_design_n + 1;
}

static int build_designs(void) {
    refresh_designs();
    snprintf(s_title, sizeof(s_title), "Designs");
    snprintf(s_subtitle, sizeof(s_subtitle), "Blueprints are shared between slots (Part N3).");
    begin(s_title, s_subtitle, "enter opens   esc back");
    if (s_design_n == 0) {
        s_rows[0] = (se_menu_row_t){.label = "Nothing built yet", .kind = SE_MENU_VAL_NONE};
        return 1;
    }
    for (int i = 0; i < s_design_n; i++) {
        s_rows[i] = (se_menu_row_t){.label = s_designs[i], .kind = SE_MENU_VAL_CUSTOM,
                                    .draw_value = draw_design_thumb, .ctx = s_designs[i]};
    }
    s_rows[s_design_n] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    s_def.visible_rows = 6;
    return s_design_n + 1;
}

static int build_design(void) {
    snprintf(s_title, sizeof(s_title), "%s",
             (s_design_sel >= 0 && s_design_sel < s_design_n) ? s_designs[s_design_sel] : "Design");
    snprintf(s_subtitle, sizeof(s_subtitle), "Open it in Engineering, rename it, or delete it.");
    begin(s_title, s_subtitle, "enter chooses   esc back");
    s_rows[0] = (se_menu_row_t){.label = "Open in Engineering", .kind = SE_MENU_VAL_NONE};
    s_rows[1] = (se_menu_row_t){.label = "Rename", .kind = SE_MENU_VAL_NONE};
    s_rows[2] = (se_menu_row_t){.label = "Delete", .kind = SE_MENU_VAL_NONE};
    s_rows[3] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    return 4;
}

static int build_new(void) {
    snprintf(s_seed_text, sizeof(s_seed_text), "%u", (unsigned)s_new_seed);
    snprintf(s_title, sizeof(s_title), "New world in slot %d", s_new_slot + 1);
    begin(s_title, "A seed builds the island. The name is yours.", "enter edits   esc cancels");
    s_rows[0] = (se_menu_row_t){.label = "Name", .kind = SE_MENU_VAL_TEXT, .value = s_new_name};
    s_rows[1] = (se_menu_row_t){.label = "Seed", .kind = SE_MENU_VAL_TEXT, .value = s_seed_text};
    s_rows[2] = (se_menu_row_t){.label = "Random seed", .kind = SE_MENU_VAL_NONE};
    s_rows[3] = (se_menu_row_t){.label = "Create", .kind = SE_MENU_VAL_NONE};
    s_rows[4] = (se_menu_row_t){.label = "Cancel", .kind = SE_MENU_VAL_NONE};
    return 5;
}

// The typing screens have no rows: the text IS the screen, drawn by
// draw_edit() below, and every key is either a character or one of
// enter / esc / bksp.
static int build_name(void) {
    // The same panel types four different things; only the title says which.
    char const* title   = "Name";
    char const* subtitle = NULL;
    if (s_edit_kind == EDIT_RENAME_DESIGN) {
        title    = "Blueprint name";
        subtitle = s_edit_target;
    } else if (s_edit_kind == EDIT_WORKING_NAME) {
        title    = "Blueprint name";
        subtitle = "Engineering saves it under this (Part B6).";
    } else if (s_edit_kind == EDIT_RENAME_WORLD) {
        title = "World name";
    }
    begin(title, subtitle, "type   enter accepts   esc cancels   bksp erases");
    return 0;
}

static int build_seed(void) {
    begin("Seed", "Digits only.", "type   enter accepts   esc cancels   bksp erases");
    return 0;
}

static int build_delete(void) {
    nisp_slot_t const* info = &s_slot_info[s_slot];
    snprintf(s_title, sizeof(s_title), "Delete slot %d?", s_slot + 1);
    snprintf(s_subtitle, sizeof(s_subtitle), "%s", info->info);
    begin(s_title, s_subtitle, "enter chooses   esc keeps it");
    s_rows[0] = (se_menu_row_t){.label = "Delete it", .kind = SE_MENU_VAL_NONE};
    s_rows[1] = (se_menu_row_t){.label = "Keep it", .kind = SE_MENU_VAL_NONE};
    return 2;
}

static int build_settings(void) {
    begin("Settings", "Bindings, picture, sound, the stream.", "up/down choose   enter opens   esc back");
    s_rows[0] = (se_menu_row_t){.label = "Controls", .kind = SE_MENU_VAL_NONE};
    s_rows[1] = (se_menu_row_t){.label = "Graphics", .kind = SE_MENU_VAL_NONE};
    s_rows[2] = (se_menu_row_t){.label = "Audio", .kind = SE_MENU_VAL_NONE};
    s_rows[3] = (se_menu_row_t){.label = "Display", .kind = SE_MENU_VAL_NONE};
    s_rows[4] = (se_menu_row_t){.label = "Livestream", .kind = SE_MENU_VAL_CHECK, .checked = se_stream_running()};
    s_rows[5] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    return 6;
}

static int build_controls(void) {
    begin("Controls", NULL, "enter rebinds   esc back");
    // Sixteen rows will not fit at the usual pitch, so this screen is
    // taller, tighter, and scrolls (se_ui.h: visible_rows).
    s_def.panel_h = 0.95f;
    s_def.row_h = 38.0f;
    s_def.panel_w = 0.86f;
    s_def.value_dx = 300.0f;
    s_def.visible_rows = 8;
    for (int a = 0; a < NISP_ACTION_COUNT; a++) {
        input_key_name(input_key((nisp_action_t)a), s_key_name[a], (int)sizeof(s_key_name[a]));
        s_rows[a] = (se_menu_row_t){.label = input_action_label((nisp_action_t)a), .kind = SE_MENU_VAL_TEXT,
                                    .value = s_key_name[a]};
    }
    s_rows[NISP_ACTION_COUNT] = (se_menu_row_t){.label = "Reset to defaults", .kind = SE_MENU_VAL_NONE};
    return NISP_ACTION_COUNT + 1;
}

static int build_graphics(void) {
    // Half resolution is a setting the hardware can refuse: the scene draws
    // into a half-size layer, and a badge with no room for one draws at full
    // resolution whatever the setting says. Saying so here is the
    // difference between a switch that appears to do nothing and one that
    // was never going to (main.c).
    char const* subtitle = "View distance";
    if (!s_half_available && settings_half_res()) subtitle = "Half resolution is not available on this badge";

    begin("Graphics", subtitle, "left/right adjust   enter chooses   esc back");
    char const* const names[NISP_VIEW_COUNT] = {"Near view", "Medium view", "Far view"};
    for (int i = 0; i < NISP_VIEW_COUNT; i++) {
        // RADIO, not CHECK: these are one question with three answers, and
        // picking one answers the others (se_ui.h).
        s_rows[i] = (se_menu_row_t){.label = names[i], .kind = SE_MENU_VAL_RADIO, .checked = settings_view() == i};
    }
    s_rows[NISP_VIEW_COUNT] =
        (se_menu_row_t){.label = "Half resolution", .kind = SE_MENU_VAL_CHECK, .checked = settings_half_res()};
    s_rows[NISP_VIEW_COUNT + 1] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    return NISP_VIEW_COUNT + 2;
}

// The Audio screen has one row that only exists when the music is a
// playlist, so "Back" is not a fixed number. One function says where it
// is, so the screen that draws it and the one that reacts to it cannot
// disagree.
static int audio_back_row(void) {
    return music_is_mp3() ? 5 : 4;
}

static int build_audio(void) {
    // The screen names what is playing. A recording says who made it, from
    // the license file beside it on the card (audio/music.h): a CC-BY track
    // on a badge should say so somewhere a person can read it. The
    // generator has no author to name, so it gets the plain subtitle.
    char const* subtitle = "Music and effects";
    if (music_is_mp3()) {
        char const* const attr = music_attribution();
        if (attr[0] != '\0') subtitle = attr;
    }

    begin("Audio", subtitle, "left/right adjust   enter toggles   esc back");
    s_rows[0] = (se_menu_row_t){.label = "Music", .kind = SE_MENU_VAL_CHECK, .checked = settings_music()};
    s_rows[1] = (se_menu_row_t){.label = "Music volume", .kind = SE_MENU_VAL_RANGE, .range_pct = settings_music_volume()};
    s_rows[2] = (se_menu_row_t){.label = "Sound effects", .kind = SE_MENU_VAL_CHECK, .checked = settings_effects()};
    s_rows[3] =
        (se_menu_row_t){.label = "Effects volume", .kind = SE_MENU_VAL_RANGE, .range_pct = settings_effects_volume()};
    // A playlist has more than one track in it and the menu is the only
    // place to ask for the next one; the generator has one tune, so the row
    // that would skip it is not shown at all (audio/music.h).
    int row = 4;
    if (music_is_mp3()) {
        s_rows[row++] = (se_menu_row_t){.label = "Next track", .kind = SE_MENU_VAL_NONE};
    }
    s_rows[row++] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    return row;
}

static int build_display(void) {
    begin("Display", "The badge's own settings, shared with everything else on it",
          "left/right adjust   esc back");
    s_rows[0] = (se_menu_row_t){.label = "Display brightness", .kind = SE_MENU_VAL_RANGE,
                                .range_pct = se_hw_get_display_brightness()};
    s_rows[1] = (se_menu_row_t){.label = "Keyboard brightness", .kind = SE_MENU_VAL_RANGE,
                                .range_pct = se_hw_get_keyboard_brightness()};
    s_rows[2] = (se_menu_row_t){.label = "LED brightness", .kind = SE_MENU_VAL_RANGE,
                                .range_pct = se_hw_get_led_brightness()};
    // The speaker volume too, and straight through se_hw.h rather than into
    // settings.txt: it is the device's volume, and an app with its own copy
    // of it is how a badge ends up loud in one app and silent in the next.
    s_rows[3] = (se_menu_row_t){.label = "Volume", .kind = SE_MENU_VAL_RANGE, .range_pct = se_hw_get_volume()};
    s_rows[4] = (se_menu_row_t){.label = "Back", .kind = SE_MENU_VAL_NONE};
    return 5;
}

static int build_pause(void) {
    begin("Paused", NULL, "up/down choose   enter chooses");
    s_rows[0] = (se_menu_row_t){.label = "Resume", .kind = SE_MENU_VAL_NONE};
    s_rows[1] = (se_menu_row_t){.label = "Save", .kind = SE_MENU_VAL_NONE};
    s_rows[2] = (se_menu_row_t){.label = "Settings", .kind = SE_MENU_VAL_NONE};
    s_rows[3] = (se_menu_row_t){.label = "Save and quit to title", .kind = SE_MENU_VAL_NONE};
    s_rows[4] = (se_menu_row_t){.label = "Quit to launcher", .kind = SE_MENU_VAL_NONE};
    return 5;
}

static int build_rows(screen_t s) {
    switch (s) {
        case SCR_TITLE: return 0;  // a bar along the bottom, not a list
        case SCR_WORLDS: return build_worlds();
        case SCR_WORLD: return build_world();
        case SCR_LAUNCH: return build_launch();
        case SCR_DESIGNS: return build_designs();
        case SCR_DESIGN: return build_design();
        case SCR_NEW: return build_new();
        case SCR_NAME: return build_name();
        case SCR_SEED: return build_seed();
        case SCR_DELETE: return build_delete();
        case SCR_SETTINGS: return build_settings();
        case SCR_CONTROLS: return build_controls();
        case SCR_GRAPHICS: return build_graphics();
        case SCR_AUDIO: return build_audio();
        case SCR_DISPLAY: return build_display();
        case SCR_PAUSE: return build_pause();
        default: return 0;
    }
}

// begin() zeroes row_count and every builder returns how many rows it laid
// down. se_menu_draw draws exactly def->row_count of them, so a count that
// is never written is a panel with a title and nothing under it.
static int build_screen(screen_t s) {
    int const rows = build_rows(s);
    s_def.row_count = rows;
    return rows;
}

// --- Small things ---------------------------------------------------------

static void go(screen_t s) {
    s_scr = s;
    if (s_cursor[s] < 0) s_cursor[s] = 0;
}

static void refresh_slots(void) {
    for (int i = 0; i < NISP_SAVE_SLOTS; i++) save_slot_peek(i, &s_slot_info[i]);
}

static int first_empty_slot(void) {
    for (int i = 0; i < NISP_SAVE_SLOTS; i++) {
        if (!s_slot_info[i].exists) return i;
    }
    return 0;
}

// A seed nobody will type: the clock, mixed. rng_mix64 is the stateless
// mixer, so this needs no rng_init and disturbs no sequence.
static uint32_t fresh_seed(void) {
    return (uint32_t)rng_mix64((uint64_t)esp_timer_get_time() ^ 0x9E3779B97F4A7C15ull);
}

static void open_new(int slot) {
    s_new_slot = slot;
    snprintf(s_new_name, sizeof(s_new_name), "%s", "Null Island");
    s_new_seed = fresh_seed();
    s_cursor[SCR_NEW] = 0;
    go(SCR_NEW);
}

static void open_edit(screen_t scr, screen_t back_to, edit_kind_t kind, char const* initial) {
    snprintf(s_edit, sizeof(s_edit), "%s", initial != NULL ? initial : "");
    s_edit_return = back_to;
    s_edit_kind = kind;
    snprintf(s_edit_target, sizeof(s_edit_target), "%s", initial != NULL ? initial : "");
    go(scr);
}

// The mixer is a device-global singleton, so a row that changes a sound
// setting is finished the moment it is changed -- no command, no round
// trip through main.c.
static void apply_audio(void) {
    audio_mixer_set_music_enabled(settings_music());
    audio_mixer_set_music_volume(settings_music_volume());
    // Group 0: the engine gives the groups no meaning, and this app has
    // one kind of sound effect so far (se_audio_source.h).
    audio_mixer_set_group_enabled(0, settings_effects());
    audio_mixer_set_group_volume(0, settings_effects_volume());
}

static uint8_t clamp_pct(int v) {
    if (v < 0) return 0;
    if (v > 100) return 100;
    return (uint8_t)v;
}

// A RANGE row moved by left/right. Five percent a press: twenty presses
// across the whole bar, which is about how many a person makes anyway.
#define RANGE_STEP 5

static void adjust_range(int row, int delta) {
    switch (s_scr) {
        case SCR_AUDIO:
            if (row == 1) settings_set_music_volume(clamp_pct((int)settings_music_volume() + delta));
            else if (row == 3) settings_set_effects_volume(clamp_pct((int)settings_effects_volume() + delta));
            else return;
            apply_audio();
            break;
        case SCR_DISPLAY:
            if (row == 0) se_hw_set_display_brightness(clamp_pct((int)se_hw_get_display_brightness() + delta));
            else if (row == 1) se_hw_set_keyboard_brightness(clamp_pct((int)se_hw_get_keyboard_brightness() + delta));
            else if (row == 2) se_hw_set_led_brightness(clamp_pct((int)se_hw_get_led_brightness() + delta));
            else if (row == 3) se_hw_set_volume(clamp_pct((int)se_hw_get_volume() + delta));
            break;
        default: break;
    }
}

// --- Typing ---------------------------------------------------------------

static void edit_char(char c) {
    size_t const len = strlen(s_edit);
    if (len >= sizeof(s_edit) - 1) return;
    // A name is short on purpose: it goes into a save header and onto a
    // menu row, and both have a width.
    if (s_scr == SCR_NAME && len >= NISP_NAME_MAX - 1) return;
    if (s_scr == SCR_SEED) {
        if (c < '0' || c > '9') return;
        if (len >= 9) return;  // fits in a uint32 with room to spare
    }
    s_edit[len] = c;
    s_edit[len + 1] = '\0';
}

static void edit_erase(void) {
    size_t const len = strlen(s_edit);
    if (len == 0) return;
    s_edit[len - 1] = '\0';
}

static void edit_commit(menu_cmd_t* cmd) {
    if (s_scr == SCR_SEED) {
        if (s_edit[0] != '\0') s_new_seed = (uint32_t)strtoul(s_edit, NULL, 10);
    } else if (s_edit_kind == EDIT_RENAME_WORLD) {
        if (s_edit[0] != '\0' && save_rename(s_slot, s_edit)) {
            menu_status_set("Renamed");
            refresh_slots();
        } else {
            menu_status_set(s_edit[0] != '\0' ? "Could not rename" : "A world needs a name");
        }
    } else if (s_edit_kind == EDIT_RENAME_DESIGN) {
        if (s_edit[0] != '\0' && nisp_design_name_ok(s_edit) && nisp_design_rename(s_edit_target, s_edit)) {
            menu_status_set("Renamed");
            refresh_designs();
        } else {
            menu_status_set(s_edit[0] != '\0' ? "Could not rename" : "A blueprint needs a name");
        }
    } else if (s_edit_kind == EDIT_WORKING_NAME) {
        // Engineering owns the working design; the name comes back to main.c
        // as a command rather than being written to a file here (D-11).
        if (s_edit[0] != '\0' && nisp_design_name_ok(s_edit)) {
            cmd->kind = MENU_CMD_NAME;
            snprintf(cmd->name, sizeof(cmd->name), "%.*s", (int)sizeof(cmd->name) - 1, s_edit);
        } else {
            menu_status_set("Letters, digits, space, dash, underscore");
        }
    } else {
        // The width is spelled out because it is the point: a name is
        // NISP_NAME_MAX characters, which is what edit_char lets a person
        // type and what the save and the menu row have room for.
        if (s_edit[0] != '\0') snprintf(s_new_name, sizeof(s_new_name), "%.*s", NISP_NAME_MAX - 1, s_edit);
    }
    go(s_edit_return);
}

// --- Input ----------------------------------------------------------------

// One bit per menu action, collected over the frame. A press of one key
// arrives as a scancode event, a navigation event, and a character event,
// and which of the three depends on the keyboard; OR-ing them into one bit
// is what makes one press move the cursor once.
#define ACT_UP    (1u << 0)
#define ACT_DOWN  (1u << 1)
#define ACT_LEFT  (1u << 2)
#define ACT_RIGHT (1u << 3)
#define ACT_OK    (1u << 4)
#define ACT_BACK  (1u << 5)
#define ACT_BKSP  (1u << 6)

static uint32_t s_pending;
static char s_typed[16];
static int s_typed_n;

void menu_event(bsp_input_event_t const* ev) {
    if (ev == NULL || s_scr == SCR_NONE) return;

    switch (ev->type) {
        case INPUT_EVENT_TYPE_SCANCODE:
            switch (ev->args_scancode.scancode) {
                case BSP_INPUT_SCANCODE_ESCAPED_GREY_UP: s_pending |= ACT_UP; break;
                case BSP_INPUT_SCANCODE_ESCAPED_GREY_DOWN: s_pending |= ACT_DOWN; break;
                case BSP_INPUT_SCANCODE_ESCAPED_GREY_LEFT: s_pending |= ACT_LEFT; break;
                case BSP_INPUT_SCANCODE_ESCAPED_GREY_RIGHT: s_pending |= ACT_RIGHT; break;
                case BSP_INPUT_SCANCODE_ENTER: s_pending |= ACT_OK; break;
                case BSP_INPUT_SCANCODE_ESC: s_pending |= ACT_BACK; break;
                case BSP_INPUT_SCANCODE_BACKSPACE: s_pending |= ACT_BKSP; break;
                default: break;
            }
            break;

        case INPUT_EVENT_TYPE_NAVIGATION:
            if (!ev->args_navigation.state) break;  // releases are not presses
            switch (ev->args_navigation.key) {
                case BSP_INPUT_NAVIGATION_KEY_UP: s_pending |= ACT_UP; break;
                case BSP_INPUT_NAVIGATION_KEY_DOWN: s_pending |= ACT_DOWN; break;
                case BSP_INPUT_NAVIGATION_KEY_LEFT: s_pending |= ACT_LEFT; break;
                case BSP_INPUT_NAVIGATION_KEY_RIGHT: s_pending |= ACT_RIGHT; break;
                case BSP_INPUT_NAVIGATION_KEY_RETURN: s_pending |= ACT_OK; break;
                case BSP_INPUT_NAVIGATION_KEY_ESC: s_pending |= ACT_BACK; break;
                case BSP_INPUT_NAVIGATION_KEY_BACKSPACE: s_pending |= ACT_BKSP; break;
                default: break;
            }
            break;

        case INPUT_EVENT_TYPE_KEYBOARD: {
            char const c = ev->args_keyboard.ascii;
            if (c == '\r' || c == '\n') s_pending |= ACT_OK;
            else if (c == '\b') s_pending |= ACT_BKSP;
            else if (c >= 32 && c < 127 && s_typed_n < (int)sizeof(s_typed)) s_typed[s_typed_n++] = c;
            break;
        }

        default: break;
    }
}

// --- What a press does ----------------------------------------------------

// The title bar's items, left to right. The bar is the whole of the title's
// menu; everything else on that screen is the wordmark (ui/title.h). Three
// items, like every other game on the badge: the stream is a setting, not a
// thing to reach for from the title (Settings > Livestream).
#define TITLE_ITEMS 3

static char const* title_label(int i) {
    switch (i) {
        case 0: return "Play";
        case 1: return "Settings";
        default: return "Quit";
    }
}

static void go_back(menu_cmd_t* cmd) {
    if (s_scr == SCR_PAUSE) {
        cmd->kind = MENU_CMD_RESUME;
        return;
    }
    screen_t const parent = parent_of(s_scr);
    if (parent == SCR_NONE) return;  // the title bar has nowhere to back out to
    go(parent);
}

static void activate(menu_cmd_t* cmd) {
    int const row = s_menu.cursor;

    switch (s_scr) {
        case SCR_TITLE:
            if (row == 0) {
                refresh_slots();
                s_cursor[SCR_WORLDS] = 0;
                go(SCR_WORLDS);
            } else if (row == 1) {
                s_settings_back = SCR_TITLE;
                go(SCR_SETTINGS);
            } else if (row == 2) {
                cmd->kind = MENU_CMD_LEAVE;
            }
            break;

        case SCR_WORLDS:
            if (row < NISP_SAVE_SLOTS) {
                if (s_slot_info[row].exists) {
                    // Opening a world is main.c's business: the save is read,
                    // the island is built from its seed, and the hub opens on
                    // top of it (plan 4.7).
                    cmd->kind = MENU_CMD_PLAY;
                    cmd->slot = row;
                } else {
                    open_new(row);
                }
            } else {
                open_new(first_empty_slot());
            }
            break;

        case SCR_WORLD:
            if (row == 0) {
                // "Continue mission" is a verb with an argument the hub cannot
                // see: which mission is main.c's business, and if there is
                // none it says so rather than pretending.
                cmd->kind = MENU_CMD_RESUME;
            } else if (row == 1) {
                cmd->kind = MENU_CMD_BUILD;
                cmd->slot = s_slot;
            } else if (row == 2) {
                s_cursor[SCR_LAUNCH] = 0;
                go(SCR_LAUNCH);
            } else if (row == 3) {
                s_cursor[SCR_DESIGNS] = 0;
                go(SCR_DESIGNS);
            } else if (row == 4) {
                open_edit(SCR_NAME, SCR_WORLD, EDIT_RENAME_WORLD, s_slot_info[s_slot].name);
            } else if (row == 5) {
                s_cursor[SCR_DELETE] = 1;  // "Keep it" is the safe thing to land on
                go(SCR_DELETE);
            } else {
                go_back(cmd);
            }
            break;

        case SCR_LAUNCH:
            if (s_design_n == 0) {
                go(SCR_WORLD);
            } else if (row < s_design_n) {
                cmd->kind = MENU_CMD_LAUNCH;
                cmd->slot = s_slot;
                snprintf(cmd->name, sizeof(cmd->name), "%s", s_designs[row]);
            } else {
                go_back(cmd);
            }
            break;

        case SCR_DESIGNS:
            if (s_design_n == 0) {
                go(SCR_WORLD);
            } else if (row < s_design_n) {
                s_design_sel = row;
                s_cursor[SCR_DESIGN] = 0;
                go(SCR_DESIGN);
            } else {
                go_back(cmd);
            }
            break;

        case SCR_DESIGN:
            if (s_design_sel < 0 || s_design_sel >= s_design_n) {
                go(SCR_DESIGNS);
                break;
            }
            if (row == 0) {
                cmd->kind = MENU_CMD_ENGINEERING;
                cmd->slot = s_slot;
                snprintf(cmd->name, sizeof(cmd->name), "%s", s_designs[s_design_sel]);
            } else if (row == 1) {
                open_edit(SCR_NAME, SCR_DESIGN, EDIT_RENAME_DESIGN, s_designs[s_design_sel]);
            } else if (row == 2) {
                if (nisp_design_delete(s_designs[s_design_sel])) {
                    menu_status_set("Deleted");
                    refresh_designs();
                } else {
                    menu_status_set("Could not delete");
                }
                go(SCR_DESIGNS);
            } else {
                go_back(cmd);
            }
            break;

        case SCR_NEW:
            if (row == 0) {
                open_edit(SCR_NAME, SCR_NEW, EDIT_NEW_WORLD, s_new_name);
            } else if (row == 1) {
                snprintf(s_edit, sizeof(s_edit), "%u", (unsigned)s_new_seed);
                open_edit(SCR_SEED, SCR_NEW, EDIT_NEW_WORLD, s_seed_text);
            } else if (row == 2) {
                s_new_seed = fresh_seed();
                menu_status_set("A new seed");
            } else if (row == 3) {
                cmd->kind = MENU_CMD_CREATE;
                cmd->slot = s_new_slot;
                snprintf(cmd->name, sizeof(cmd->name), "%s", s_new_name);
                cmd->seed = s_new_seed;
            } else {
                refresh_slots();
                go(SCR_WORLDS);
            }
            break;

        case SCR_DELETE:
            if (row == 0) {
                bool const ok = save_delete(s_slot);
                refresh_slots();
                menu_status_set(ok ? "Deleted" : "Could not delete");
                go(SCR_WORLDS);
                // Deleting the world that is open leaves main.c flying a
                // mission in a save that no longer exists.
                if (ok && s_slot == s_open_slot) cmd->kind = MENU_CMD_CLOSE_WORLD;
            } else {
                go(SCR_WORLD);
            }
            break;

        case SCR_SETTINGS:
            if (row == 0) go(SCR_CONTROLS);
            else if (row == 1) go(SCR_GRAPHICS);
            else if (row == 2) go(SCR_AUDIO);
            else if (row == 3) go(SCR_DISPLAY);
            else if (row == 4) cmd->kind = MENU_CMD_STREAM;
            else go_back(cmd);
            break;

        case SCR_CONTROLS:
            if (row < NISP_ACTION_COUNT) {
                nisp_action_t const action = (nisp_action_t)row;
                // Blocks: the engine pumps frames until a key is pressed,
                // and takes F1 and the volume keys along the way (se_ui.h).
                uint16_t const sc = se_ui_capture_key(input_action_label(action));
                if (sc != 0) {
                    input_bind(action, sc);
                    menu_status_set("Bound");
                }
            } else if (row == NISP_ACTION_COUNT) {
                input_reset_defaults();
                menu_status_set("Back to the shipped keys");
            }
            break;

        case SCR_GRAPHICS:
            if (row < NISP_VIEW_COUNT) {
                settings_set_view(row);
            } else if (row == NISP_VIEW_COUNT) {
                settings_set_half_res(!settings_half_res());
                if (settings_half_res() && !s_half_available) menu_status_set("Not available on this badge");
            } else {
                go_back(cmd);
            }
            break;

        case SCR_AUDIO:
            if (row == 0) {
                settings_set_music(!settings_music());
                apply_audio();
            } else if (row == 2) {
                settings_set_effects(!settings_effects());
                apply_audio();
            } else if (row == 4 && music_is_mp3()) {
                music_skip();
                menu_status_set(music_now_playing());
            } else if (row == audio_back_row()) {
                go_back(cmd);
            }
            break;

        case SCR_DISPLAY:
            if (row == 4) go_back(cmd);
            break;

        case SCR_PAUSE:
            if (row == 0) cmd->kind = MENU_CMD_RESUME;
            else if (row == 1) cmd->kind = MENU_CMD_SAVE;
            else if (row == 2) {
                s_settings_back = SCR_PAUSE;
                go(SCR_SETTINGS);
            } else if (row == 3) cmd->kind = MENU_CMD_SAVE_QUIT;
            else if (row == 4) cmd->kind = MENU_CMD_LEAVE;
            break;

        default: break;
    }
}

menu_cmd_t menu_update(void) {
    menu_cmd_t cmd = {0};
    if (s_scr == SCR_NONE) {
        s_pending = 0;
        s_typed_n = 0;
        return cmd;
    }

    // The title bar is a bar, not a list: left/right move along it, and
    // up/down do the same, because nobody should have to know that this one
    // screen is laid out differently.
    if (s_scr == SCR_TITLE) {
        if (s_pending & (ACT_LEFT | ACT_UP)) s_cursor[SCR_TITLE]--;
        if (s_pending & (ACT_RIGHT | ACT_DOWN)) s_cursor[SCR_TITLE]++;
        if (s_cursor[SCR_TITLE] < 0) s_cursor[SCR_TITLE] = 0;
        if (s_cursor[SCR_TITLE] >= TITLE_ITEMS) s_cursor[SCR_TITLE] = TITLE_ITEMS - 1;

        s_menu.cursor = s_cursor[SCR_TITLE];
        if (s_pending & ACT_OK) activate(&cmd);
        s_pending = 0;
        s_typed_n = 0;
        return cmd;
    }

    int const rows = build_screen(s_scr);
    if (s_cursor[s_scr] >= rows) s_cursor[s_scr] = rows > 0 ? rows - 1 : 0;
    s_menu.def = &s_def;
    s_menu.cursor = s_cursor[s_scr];

    // The typing screens are not lists: every key is a character or one of
    // accept / cancel / erase, and none of it goes through the engine's
    // cursor.
    if (s_scr == SCR_NAME || s_scr == SCR_SEED) {
        for (int i = 0; i < s_typed_n; i++) edit_char(s_typed[i]);
        if (s_pending & ACT_BKSP) edit_erase();
        s_pending &= ~ACT_BKSP;
        if (s_pending & ACT_OK) edit_commit(&cmd);
        else if (s_pending & ACT_BACK) go(s_edit_return);  // discard
        s_pending = 0;
        s_typed_n = 0;
        return cmd;
    }

    if (s_pending & ACT_UP) se_menu_input(&s_menu, SE_MENU_ACT_UP);
    if (s_pending & ACT_DOWN) se_menu_input(&s_menu, SE_MENU_ACT_DOWN);

    se_menu_result_t result = SE_MENU_RESULT_NONE;
    if (s_pending & ACT_OK) result = se_menu_input(&s_menu, SE_MENU_ACT_ACTIVATE);
    else if (s_pending & ACT_BACK) result = se_menu_input(&s_menu, SE_MENU_ACT_BACK);
    else if (s_pending & ACT_LEFT) result = se_menu_input(&s_menu, SE_MENU_ACT_LEFT);
    else if (s_pending & ACT_RIGHT) result = se_menu_input(&s_menu, SE_MENU_ACT_RIGHT);

    s_cursor[s_scr] = s_menu.cursor;
    s_pending = 0;
    s_typed_n = 0;

    if (result == SE_MENU_RESULT_BACK) {
        go_back(&cmd);
    } else if (result == SE_MENU_RESULT_DECREMENT) {
        adjust_range(s_menu.cursor, -RANGE_STEP);
    } else if (result == SE_MENU_RESULT_INCREMENT) {
        adjust_range(s_menu.cursor, RANGE_STEP);
    } else if (result == SE_MENU_RESULT_ACTIVATED) {
        activate(&cmd);
    }
    return cmd;
}

// --- Drawing --------------------------------------------------------------

// The text being typed, big and centred in the panel, with a caret. The
// engine's menu draws the panel, the title and the hint; this is the one
// thing it has no row kind for.
static void draw_edit(pax_buf_t* fb) {
    float const fbw = pax_buf_get_widthf(fb);
    float const fbh = pax_buf_get_heightf(fb);
    float const pw = fbw * s_def.panel_w;
    float const ph = fbh * s_def.panel_h;
    float const px = (fbw - pw) * 0.5f;
    float const py = (fbh - ph) * 0.5f;

    char disp[64];
    snprintf(disp, sizeof(disp), "%s_", s_edit);

    pax_vec2f const sz = rendertext_size(NULL, 34.0f, disp);
    float const x = px + (pw - sz.x) * 0.5f;
    float const y = py + SE_UI_TOP_PAD + s_def.title_h + 44.0f;

    rendertext_draw(fb, 0xff000000u, NULL, 34.0f, x + 2.0f, y + 2.0f, disp);
    rendertext_draw(fb, 0xffffff6bu, NULL, 34.0f, x, y, disp);
}

// The title's own row: a strip along the bottom, under the wordmark, so the
// island and the logo stay in view. Every other screen is a panel over them.
static void draw_title_bar(pax_buf_t* fb) {
    float const h   = 26.0f;
    float const gap = 48.0f;
    float       w[TITLE_ITEMS];
    float       total = 0.0f;
    for (int i = 0; i < TITLE_ITEMS; i++) {
        w[i] = rendertext_size(NULL, h, title_label(i)).x;
        total += w[i] + (i ? gap : 0.0f);
    }

    float const fw = pax_buf_get_widthf(fb);
    float const fh = pax_buf_get_heightf(fb);
    float const y  = fh - 44.0f;
    float       x  = (fw - total) * 0.5f;

    direct_565_dim_rect((uint16_t*)pax_buf_get_pixels(fb), fb->reverse_endianness, (int)(x - 34.0f),
                        (int)(y - 12.0f), (int)(total + 68.0f), (int)(h + 24.0f));

    for (int i = 0; i < TITLE_ITEMS; i++) {
        bool const sel = i == s_cursor[SCR_TITLE];
        if (sel) rendertext_draw(fb, SE_UI_COL_HILITE, NULL, h, x - 24.0f, y, ">");
        rendertext_draw(fb, sel ? SE_UI_COL_HILITE : SE_UI_COL_NORMAL, NULL, h, x, y, title_label(i));
        x += w[i] + gap;
    }

    char const* const st = status_line();
    if (st != NULL) {
        float const sw = rendertext_size(NULL, 16.0f, st).x;
        rendertext_draw(fb, SE_UI_COL_NORMAL, NULL, 16.0f, (fw - sw) * 0.5f, y - 34.0f, st);
    }
}

void menu_draw(pax_buf_t* fb) {
    if (fb == NULL || s_scr == SCR_NONE) return;
    if (s_scr == SCR_TITLE) {
        draw_title_bar(fb);
        return;
    }
    se_menu_draw(&s_menu, fb);
    if (s_scr == SCR_NAME || s_scr == SCR_SEED) draw_edit(fb);
}

// --- Opening, closing, and the test kit's shortcuts -----------------------

void menu_open_title(void) {
    s_open_slot = -1;
    refresh_slots();
    s_cursor[SCR_TITLE] = 0;
    s_settings_back = SCR_TITLE;
    go(SCR_TITLE);
}

// main.c: opening a slot opens its Spaceport, not a mission (plan 4.7).
void menu_open_spaceport(int slot) {
    s_slot = slot;
    s_open_slot = slot;
    refresh_slots();
    refresh_designs();
    s_cursor[SCR_WORLD] = 0;
    go(SCR_WORLD);
}

// The name Engineering is about to save under. The editor is modal over the
// drawing rather than a screen in the tree: its "back" screen is SCR_NONE,
// so accepting or cancelling hands the keyboard back to the sheet and main.c
// takes the name out of the command (D-11).
void menu_open_working_name(char const* initial) {
    open_edit(SCR_NAME, SCR_NONE, EDIT_WORKING_NAME, initial);
}

void menu_open_pause(void) {
    s_cursor[SCR_PAUSE] = 0;
    go(SCR_PAUSE);
}

void menu_close(void) {
    s_scr = SCR_NONE;
    s_pending = 0;
    s_typed_n = 0;
}

bool menu_active(void) {
    return s_scr != SCR_NONE;
}

bool menu_in_panel(void) {
    return s_scr != SCR_NONE && s_scr != SCR_TITLE;
}

void menu_status(char const* msg) {
    menu_status_set(msg);
}

// main.c says whether the half-size layer exists, once, at boot.
void menu_half_res_available(bool available) {
    s_half_available = available;
}

// The names tools/testrun.py can ask a `shots` run for.
bool menu_show(char const* name) {
    if (name == NULL) return false;
    refresh_slots();
    if (!strcmp(name, "title")) {
        go(SCR_TITLE);
    } else if (!strcmp(name, "worlds")) {
        go(SCR_WORLDS);
    } else if (!strcmp(name, "world") || !strcmp(name, "spaceport")) {
        s_slot = 0;
        refresh_designs();
        go(SCR_WORLD);
    } else if (!strcmp(name, "launch")) {
        s_slot = 0;
        refresh_designs();
        go(SCR_LAUNCH);
    } else if (!strcmp(name, "designs")) {
        s_slot = 0;
        refresh_designs();
        go(SCR_DESIGNS);
    } else if (!strcmp(name, "design")) {
        s_slot = 0;
        refresh_designs();
        s_design_sel = 0;
        go(SCR_DESIGN);
    } else if (!strcmp(name, "new")) {
        open_new(first_empty_slot());
    } else if (!strcmp(name, "delete")) {
        s_slot = 0;
        go(SCR_DELETE);
    } else if (!strcmp(name, "settings")) {
        s_settings_back = SCR_TITLE;
        go(SCR_SETTINGS);
    } else if (!strcmp(name, "controls")) {
        s_settings_back = SCR_TITLE;
        go(SCR_CONTROLS);
    } else if (!strcmp(name, "graphics")) {
        s_settings_back = SCR_TITLE;
        go(SCR_GRAPHICS);
    } else if (!strcmp(name, "audio")) {
        s_settings_back = SCR_TITLE;
        go(SCR_AUDIO);
    } else if (!strcmp(name, "display")) {
        s_settings_back = SCR_TITLE;
        go(SCR_DISPLAY);
    } else if (!strcmp(name, "pause")) {
        go(SCR_PAUSE);
    } else {
        return false;
    }
    return true;
}

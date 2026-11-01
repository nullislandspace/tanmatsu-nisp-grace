#pragma once
// =====================================================================
//  Null Island Space Program  --  the menus
// ---------------------------------------------------------------------
//  Every screen that is not the flight itself:
//
//    title bar     a strip along the bottom: Play / Settings / Quit
//      Play        the save slots, then
//        a world   its Spaceport: Continue mission / Build a rocket /
//                  Launch... / Designs... / Rename / Delete
//        empty     New world: name, Create
//      Settings    Controls / Graphics / Audio / Display / Livestream
//    pause         Resume / Save / Settings / Save and quit / Quit
//
//  THE HUB LISTS VERBS, NEVER VERBS WITH ARGUMENTS (D-24). A row that needs
//  a parameter opens a picker for it -- Launch... asks which design, Designs...
//  asks which one -- so the hub is the same fixed-length menu however many
//  blueprints the player has (Part N1).
//
//  Every screen but the title bar is drawn with the engine's list menu
//  (se_ui.h) over whatever is behind it -- the title's drift, or the
//  frozen ship -- so no screen needs a renderer of its own. The title bar
//  is the exception: a panel over the title would cover the logo and the
//  wordmark, so it is a row of items along the bottom instead, with the
//  cursor marker on the one that is chosen.
//
//  THE MENU DECIDES, main.c ACTS. Anything that changes which world is
//  open, whether the ship is flying, or whether the USB-C port is now a
//  video link comes back from menu_update() as a command: those have an
//  order to them and a frame they have to happen on, which is main.c's
//  business. What is the menu's own business -- a setting, a key binding,
//  renaming or deleting a world nobody has open -- it simply does.
//
//  INPUT IS EVENTS, not polling. One physical key press can arrive as up
//  to three events (scancode, navigation, character) and which ones
//  depends on the keyboard. menu_event() folds them into one action per
//  key per frame, so a press moves the cursor exactly once whichever
//  keyboard made it.
//
//  THE AUDIO AND DISPLAY ROWS ARE APPLIED HERE, not handed back: the
//  mixer and se_hw.h are device-global singletons, and a row that changes
//  one is finished the moment it is changed. The rows that are not
//  finished -- opening a world, starting the stream -- are commands.
// =====================================================================

#include <stdbool.h>
#include <stdint.h>

#include "bsp/input.h"
#include "pax_gfx.h"
#include "save/save.h"

typedef enum {
    MENU_CMD_NONE = 0,
    MENU_CMD_PLAY,      // open the world in `slot`: its Spaceport hub opens, the mission does not
    MENU_CMD_CLOSE_WORLD, // the open world was deleted: main.c drops it (plan 1.2)
    MENU_CMD_CREATE,    // make a world in `slot` from `name`, then play it (D-50)
    MENU_CMD_RESUME,    // close the pause menu, keep flying
    MENU_CMD_SAVE,      // save the open world, stay paused
    MENU_CMD_SAVE_QUIT, // save the open world and go back to the title
    MENU_CMD_LEAVE,     // back to the launcher
    MENU_CMD_STREAM,    // toggle the livestream (main.c owns the framebuffer)

    // Engineering (Part B, plan 4.6-4.7). The hub lists verbs and the
    // pickers fill in the arguments (D-24), so these are the verbs.
    MENU_CMD_BUILD,       // Engineering, on a fresh unnamed working design
    MENU_CMD_ENGINEERING, // Engineering, on the saved design in `name`
    MENU_CMD_LAUNCH,      // fly the saved design in `name`
    MENU_CMD_NAME,        // the design name the text entry accepted

    // Ending a flight, three different ways (plan 9.4). Which one the pause
    // menu offers is a question about the flight, and the pause menu is the
    // thing looking at it -- so it asks for one of these three rather than
    // letting main.c guess.
    MENU_CMD_BACK_TO_DESIGN, // it never left the pad: back to its blueprint
    MENU_CMD_LEAVE_CRAFT,    // leave it where it is and go back to the hub
    MENU_CMD_RECOVER,        // the flight is over: recover it, back to the hub
    MENU_CMD_MISSION,        // fly the archived craft at index `slot`
} menu_cmd_kind_t;

typedef struct {
    menu_cmd_kind_t kind;
    int             slot;
    // Wide enough for both names a command carries: a world's (NISP_NAME_MAX)
    // and a design's (NISP_DESIGN_NAME_MAX), which is one character longer.
    char            name[32];
} menu_cmd_t;

// What the flight looks like, so the pause menu can offer the way out that
// matches it (plan 9.4): a rocket that never left the pad goes back to its
// blueprint, one that is up can be left where it is, one that is down can be
// left or recovered, and a wreck can only be recovered. NULL is "no flight",
// which is what a test shot of the pause screen asks for.
typedef struct {
    bool on_pad;
    bool landed;
    bool wrecked;
    char craft[32];
} menu_flight_t;

// The title's bar (the root menu) or the pause menu. Either replaces
// whatever was showing.
void menu_open_title(void);
void menu_open_pause(menu_flight_t const* info);

// The craft left out there, for the hub's "Missions..." picker. main.c owns
// the archive (it is part of the save); the menu only reads it, and only
// when a screen that lists it is opened.
void menu_set_archive(nisp_archive_t const* archive);

// No menu: the ship has the keyboard.
void menu_close(void);
bool menu_active(void);

// Is the menu showing something other than the title's own bar? The title
// hides its wordmark and its "press ENTER" under a panel.
bool menu_in_panel(void);

// Every input event the engine hands the app while a menu is up.
void menu_event(bsp_input_event_t const* ev);

// Once a frame: act on this frame's keys and say what main.c must do.
menu_cmd_t menu_update(void);

// Draw the menu over `fb`, at full resolution, after everything else.
void menu_draw(pax_buf_t* fb);

// The Spaceport hub for `slot`, from main.c: opening a world opens its hub
// rather than launching whatever was last flown (plan 4.7).
void menu_open_spaceport(int slot);

// The text entry for the blueprint Engineering is holding. It closes itself:
// MENU_CMD_NAME carries the accepted name, and no menu is left open.
void menu_open_working_name(char const* initial);

// Open one screen by name ("worlds", "controls", "pause", ...) for a
// `shots` test: the screens are drawn from state, so a shot of each is a
// regression test of each. False for a name it does not know.
bool menu_show(char const* name);

// A line of news under the menu title for a couple of seconds ("Saved").
void menu_status(char const* msg);

// Whether this badge has the half-size layer half resolution needs. The
// Graphics screen says so when the setting is on and the picture cannot
// change, rather than letting a switch look broken (main.c).
void menu_half_res_available(bool available);

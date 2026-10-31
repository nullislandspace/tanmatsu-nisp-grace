#pragma once
// =====================================================================
//  Null Island Space Program  --  the menus
// ---------------------------------------------------------------------
//  Every screen that is not the flight itself:
//
//    title bar     a strip along the bottom: Play / Settings / Quit
//      Play        the save slots, then
//        a world   Play / Rename / Delete
//        empty     New world: name, seed, Random seed, Create
//      Settings    Controls / Graphics / Audio / Display / Livestream
//    pause         Resume / Save / Settings / Save and quit / Quit
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
    MENU_CMD_PLAY,      // open the world in `slot`
    MENU_CMD_CREATE,    // make a world in `slot` from `name` and `seed`, then play it
    MENU_CMD_RESUME,    // close the pause menu, keep flying
    MENU_CMD_SAVE,      // save the open world, stay paused
    MENU_CMD_SAVE_QUIT, // save the open world and go back to the title
    MENU_CMD_LEAVE,     // back to the launcher
    MENU_CMD_STREAM,    // toggle the livestream (main.c owns the framebuffer)
} menu_cmd_kind_t;

typedef struct {
    menu_cmd_kind_t kind;
    int             slot;
    char            name[NISP_NAME_MAX];
    uint32_t        seed;
} menu_cmd_t;

// The title's bar (the root menu) or the pause menu. Either replaces
// whatever was showing.
void menu_open_title(void);
void menu_open_pause(void);

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

#pragma once
// =====================================================================
//  Null Island Space Program  --  the game's own settings
// ---------------------------------------------------------------------
//  What the Graphics, Audio and Controls menus change -- key bindings
//  included -- in ONE TEXT FILE next to the saves:
//
//      /sd/nisp/settings.txt
//
//  so copying that directory backs up a player's saves and their
//  settings together. Plain `key=value` lines: readable, and fixable by
//  hand. Unknown keys are ignored and missing ones keep their defaults,
//  so the file survives settings being added and removed.
//
//  NOT here: volume and the three brightnesses. Those are the device's,
//  shared with the launcher and every other app, and se_hw.h owns them;
//  the menus call it directly. A game keeping its own copy of the volume
//  is how a badge ends up loud in one app and silent in the next.
//
//  NOT here either: the livestream. Turning it on takes the USB-C port
//  away from the debug console (se_stream.h), so a save file that
//  remembered it would mean an app that cannot be talked to after an
//  update. It is decided every session, on the screen that turns it on.
//
//  Each change rewrites the file at once. A setting is changed by a
//  person pressing a key in a menu, a few times a session at most, so
//  there is nothing to batch. The rewrite goes to settings.tmp first and
//  is moved over the old file, so a badge switched off mid-write keeps
//  one whole copy or the other.
// =====================================================================

#include <stdbool.h>
#include <stdint.h>

// View distance presets: an index, not a distance, so the number a
// player picks and the number the renderer uses can change independently
// (game/scene.h, nisp_view_range()).
#define NISP_VIEW_COUNT 3

// Called once at boot, after input_init(): it fills these values AND
// restores the key bindings, which is why the bindings have to exist
// before it runs.
void settings_init(void);
void settings_save(void);

int settings_view(void);
void settings_set_view(int preset);

bool settings_half_res(void);
void settings_set_half_res(bool on);

bool settings_music(void);
void settings_set_music(bool on);
uint8_t settings_music_volume(void);
void settings_set_music_volume(uint8_t percent);

bool settings_effects(void);
void settings_set_effects(bool on);
uint8_t settings_effects_volume(void);
void settings_set_effects_volume(uint8_t percent);

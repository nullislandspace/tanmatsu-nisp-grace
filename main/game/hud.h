#pragma once
// =====================================================================
//  Null Island Space Program  --  what the pilot is shown
// ---------------------------------------------------------------------
//  Drawn at full resolution over whatever the scene produced, after the
//  upscale and before the screenshot path, so a shot shows what the
//  player saw rather than the world alone.
//
//  The readouts are the ones a flight needs, and they are the numbers
//  flight/physics.c already computed (nisp_readout_t): altitude, speed,
//  speed through the air, dynamic pressure, apoapsis and periapsis,
//  mass, thrust, TWR, the hottest part's heat, and what the control
//  system is doing. The HUD draws them; it decides nothing.
//
//  TWO COLUMNS, one on each side, so the middle of the screen stays the
//  ship. Left is where the ship is and how it is moving; right is where
//  it is going. The bottom centre is the engine: throttle, stage, warp.
//
//  The INFO key adds the diagnostic block -- seed, slot, renderer
//  settings, primitive counts -- the one a bug report quotes.
// =====================================================================

#include <stdbool.h>

#include "pax_gfx.h"
#include "flight/mission.h"
#include "flight/physics.h"
#include "save/save.h"

typedef struct {
    float fps;
    int   slot;
    int   tris;
    int   lines;
    int   points;
    int64_t rast_us;
    // The mode this frame was actually drawn in. Not settings_half_res():
    // a setting that the hardware refused would otherwise be reported as
    // if it had taken (main.c knows whether the half-size layer exists).
    bool half;
} hud_stats_t;

// `mission` and `readout` may both be NULL -- a menu over a frozen view
// has nothing to report -- in which case only the world's name shows.
void hud_draw(pax_buf_t* fb, nisp_game_t const* game, nisp_mission_t const* mission,
              nisp_readout_t const* readout, bool info, hud_stats_t const* stats);

// A centre reticle: the ship points where the crosshair is.
void hud_reticle(pax_buf_t* fb);

// A line of news along the bottom ("Saved", "Screenshot saved"). Pass
// NULL or "" for none. It lives on the HUD, not on a timer of its own:
// the caller decides how long a message is worth showing.
void hud_flash(pax_buf_t* fb, char const* msg);

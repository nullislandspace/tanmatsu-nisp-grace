#pragma once
// =====================================================================
//  Null Island Space Program  --  what the pilot is shown
// ---------------------------------------------------------------------
//  Drawn at full resolution over whatever the scene produced, after the
//  upscale and before the screenshot path, so a shot shows what the
//  player saw rather than the world alone.
//
//  The readouts are the ones a flight needs: where the ship is, which
//  way it points, how far it has been flown, and how far the pad is.
//  The INFO key adds the diagnostic block -- seed, slot, renderer
//  settings, primitive counts -- the one a bug report quotes.
// =====================================================================

#include <stdbool.h>

#include "pax_gfx.h"
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

void hud_draw(pax_buf_t* fb, nisp_game_t const* game, nisp_pose_t const* pose, bool info, hud_stats_t const* stats);

// A centre reticle: the ship points where the crosshair is.
void hud_reticle(pax_buf_t* fb);

// A line of news along the bottom ("Saved", "Screenshot saved"). Pass
// NULL or "" for none. It lives on the HUD, not on a timer of its own:
// the caller decides how long a message is worth showing.
void hud_flash(pax_buf_t* fb, char const* msg);

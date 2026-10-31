#pragma once
// =====================================================================
//  Null Island Space Program  --  the title screen
// ---------------------------------------------------------------------
//  The solar system, flown past slowly, with the name over it. There is
//  no separate title model and no second renderer: the title IS a camera
//  (game/scene.h, nisp_camera_title) and a piece of text, so it draws
//  through exactly the same path as the game and a `shots` test of the
//  title is a test of the renderer.
//
//  The camera is a pure function of the clock, which is what the test kit
//  needs to be able to render a chosen instant and get the same pixels
//  (testkit/devtest.h). It lives in the renderer, not here, because the
//  title and the chase camera have to agree about what a camera is.
// =====================================================================

#include <stdbool.h>

#include "pax_gfx.h"

// The ball on its own, so main.c can time it as its own frame phase
// (testkit/profile.h). Drawn before title_draw, which leaves room for it.
void title_logo(pax_buf_t* fb);

// The wordmark, the version, and what to press. Drawn after the scene,
// at full resolution, and after title_logo.
void title_draw(pax_buf_t* fb, double t);

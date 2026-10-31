#pragma once
// =====================================================================
//  Null Island Space Program  --  the island
// ---------------------------------------------------------------------
//  The world so far is a placeholder with the right shape to build a
//  game on: a seeded island, a landing pad at the origin, an obelisk on
//  it, survey markers round it, and a starfield behind all of it. It is
//  drawn from the seed alone (common/rng.h), so a save that stores its
//  seed rebuilds the same island, and a frame is a pure function of the
//  clock -- which is what makes the test kit's screenshot hashes mean
//  something (testkit/devtest.h).
//
//  NO CHUNK STREAMING. The island is one small fixed mesh, submitted in
//  full every frame; nothing is generated, paged in, or cached. Where a
//  real world will need a store, this has a heightmap and a view range.
//
//  World space is the engine's: x lateral, y up, z forward. The island
//  sits on y = 0 and the origin is the pad.
// =====================================================================

#include <stdint.h>

#include "pax_gfx.h"

#include "save/save.h"

// View-distance presets, as settings.h numbers them. A preset, not a
// distance, so the menu's three choices and the renderer's metres can
// move independently.
float nisp_view_range(int preset);

// Build the island for this seed. Call whenever a world opens.
void nisp_scene_init(uint32_t seed);

// Submit one frame's geometry through `pose` and render it. Returns this
// frame's rasterizing time in microseconds, for the test kit.
int64_t nisp_scene_render(pax_buf_t* fb, nisp_pose_t const* pose, int view_preset, double t);

// The island's surface height at (x, z): the pad is flat at 0, the land
// rises off it. The ship is kept above this, so the ground is ground.
float nisp_scene_ground(float x, float z);

// Distance from (x, z) to the pad at the origin -- what the HUD's MARK
// readout shows, and the one thing in the world the player is meant to
// be navigating to.
float nisp_scene_mark_range(float x, float z);

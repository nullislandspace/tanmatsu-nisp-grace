#pragma once
// =====================================================================
//  Null Island Space Program  --  the sky, on the badge (Part O, G)
// ---------------------------------------------------------------------
//  Three lit spheres and a starfield. That is the whole world: Bob,
//  Cheese and Frodo, drawn from the same rows sim/bodies.c keeps, at the
//  positions sim/ephemeris.c puts them at. There is no terrain, no
//  heightmap and no island -- the planet IS the ground, so "landing" and
//  "seeing the ground" read the same radius test (sim/surface.c).
//
//  CAMERA-RELATIVE METRES (D-16). The sim lives in kilometres about a
//  body; the renderer lives in metres about the camera. Every vertex
//  submitted is (body position - camera position) expressed in the local
//  frame at the camera -- east, up, -north -- so the numbers the
//  rasterizer sees are small even when the solar system is not, and the
//  engine's float32 projection never has to resolve 13.6 million km.
//
//  THE SUN IS A DIRECTION, NOT A PLACE. Frodo is 13.6 million km away,
//  which projects to a tenth of a pixel. So its direction lights the
//  scene (se_light.h, a point light parked far along that direction),
//  and what is drawn for it is a disc of a fixed angular size on the sky
//  shell. Bodies further out than the shell are drawn ON the shell, at
//  their true angular size: same pixels, no 1e10-metre vertices.
//
//  LOD BY PROJECTED RADIUS. A sphere is bands x segments of quads, and
//  the ladder picks that from how many pixels wide the disc is, so a body
//  that fills the screen gets 48x24 and one that is a dot gets 8x4. Only
//  the bands that face the camera are submitted, which is half the
//  triangles for free and still covers the whole disc.
// =====================================================================

#include <stdint.h>

#include "pax_gfx.h"

#include "flight/physics.h"
#include "save/save.h"

// The camera, in sim units. `pos_km` is in the non-rotating frame of
// `frame_body` -- the same frame nisp_vessel_t keeps its position in, so a
// chase camera is one subtraction away from the ship.
//
// yaw / pitch / roll are the engine's (se_scene.h): yaw about local up,
// pitch about the camera's right (POSITIVE PITCH LOOKS DOWN), roll about
// forward. They are angles in the local frame at the camera, which is why
// "look north" means a different yaw at every point on the planet.
typedef struct {
    double pos_km[3];
    int    frame_body;
    float  yaw, pitch, roll;
} nisp_camera_t;

// Draw range per settings preset (ui/settings.h), in km. A preset rather
// than a distance so the menu's three words and the renderer's numbers can
// still move independently.
float nisp_view_range(int preset);

// The starfield. Built once per seed; the bodies need no build step.
void nisp_scene_init(uint32_t seed);

// Point the camera along `fwd` with `up` as its up, both unit vectors in
// `cam`'s frame axes. The camera position is the caller's business.
void nisp_camera_aim(nisp_camera_t* cam, float const fwd[3], float const up[3]);

// A chase camera behind and above the ship, looking along its nose.
// `back_km` / `up_km` are metres of standoff, in the ship's own axes.
void nisp_camera_chase(nisp_camera_t* cam, nisp_vessel_t const* v, double t, float back_m, float up_m);

// The title's camera: a scripted drift past Bob with Cheese in the frame.
// A pure function of the clock, which is what keeps the shot hashes
// meaningful (testkit/devtest.h).
void nisp_camera_title(nisp_camera_t* cam, double t);

// Submit one frame and rasterize it. `ship` may be NULL (the title, a menu
// over a frozen view). Returns this frame's rasterizing time in microseconds.
int64_t nisp_scene_render(pax_buf_t* fb, nisp_camera_t const* cam, nisp_vessel_t const* ship,
                          int view_preset, double t);

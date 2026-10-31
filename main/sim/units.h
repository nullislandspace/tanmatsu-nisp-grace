#pragma once
// =====================================================================
//  Null Island Space Program  --  the units the sim runs in
// ---------------------------------------------------------------------
//  Decided before a single line of orbital maths, because retrofitting
//  metres into an orbital code is the expensive direction (D-15).
//
//  LENGTH      kilometres.  A 100 km orbit is 100.0, Bob's radius is
//              600.0, Cheese's orbit is 12,000.0. Every number the sim
//              carries sits between 1e-3 and 1e5, which is where float
//              arithmetic is happiest and where a printed number needs
//              no engineering notation.
//  TIME        seconds.  A 50 Hz physics tick is 0.02; an orbit is
//              1,875; a transfer coast is 28,600.
//  MU          km^3/s^2.  With km and s, mu is O(1e3) for a planet and
//              O(1e9) for a star, and sqrt(mu/r) comes out in km/s
//              directly -- no 6.674e-11 anywhere in the code.
//  SPEED       km/s in the sim.  The HUD converts to m/s at the last
//              moment, because 2,278.6 m/s is the number a player
//              recognises and 2.2786 is not.
//
//  RENDERING IS IN METRES, and only at the one place geometry is handed
//  to the engine: the engine wants body_position - camera_position in
//  metres, once per body per frame (D-16). That conversion lives in
//  game/scene.c and nowhere else, so a frame of drawing cannot leak a
//  metre back into the sim.
//
//  Angles are radians in every stored field. Degrees appear only in a
//  label, and only through the two helpers at the bottom.
// =====================================================================

#define NISP_KM_PER_M   (0.001)      // metres -> kilometres
#define NISP_M_PER_KM   (1000.0)     // kilometres -> metres
#define NISP_MS_PER_KMS (1000.0)     // sim speed -> the speed a HUD prints

// Standard gravity, m/s^2. Only used to turn an Isp in seconds into an
// exhaust velocity in m/s, and a thrust in kN into an acceleration --
// the two places the rocket tables are written in the units engineers
// actually use (Part V2).
#define NISP_G0 (9.80665)

#define NISP_RAD2DEG (57.29577951308232)
#define NISP_DEG2RAD (0.017453292519943295)
#define NISP_TWO_PI  (6.283185307179586)

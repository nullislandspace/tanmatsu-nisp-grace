#pragma once
// =====================================================================
//  Null Island Space Program  --  where the three bodies are
// ---------------------------------------------------------------------
//  Circular orbits in one fixed inertial frame (Part O5): no eccentricity,
//  no precession, no perturbations. That is not a shortcut taken for speed
//  -- it is what makes the sky predictable enough to plan against. A player
//  who looks up and sees Cheese over the same spot at the same time every
//  Cheese-day is looking at a model that will not surprise them, and every
//  transfer window the planner computes stays true.
//
//  Frodo sits at the origin and everything else goes round it, so the
//  inertial frame is the sun's frame and Bob's own motion is a number the
//  sim carries rather than one the player has to reason about.
//
//  Positions are kilometres, velocities km/s, both in the sim frame
//  (sim/units.h). The one place they become metres is game/scene.c, at the
//  moment geometry is handed to the engine (D-16).
// =====================================================================

#include "sim/bodies.h"

// Position and velocity of a body in the inertial frame, at time t.
void nisp_body_position(int id, double t, double out_km[3]);
void nisp_body_velocity(int id, double t, double out_kms[3]);

// A body's position relative to another one -- the vector a patched-conic
// handover needs, and the vector the scene submits.
void nisp_body_relative(int id, int ref, double t, double out_km[3]);

// --- Day and night (plan 6.1) -------------------------------------------
//
// A body that turns has a day, and a launch wants to happen in the part of
// it a person can see. These three are the whole of it: how high the sun is
// over a point, what the local clock says there, and when that point next
// faces the sun.
//
// `pos_km` is body-relative, in the same non-rotating frame a vessel keeps
// its position in (flight/physics.h) -- which is also what nisp_surface_point
// writes, so a pad and a ship answer the same question the same way.
//
// THE LOCAL CLOCK IS THE HOUR ANGLE, not an integration. A point's meridian
// angle and the sun's angle about the body's spin axis are two atan2s, and
// the difference between them is the time of day: 12 h when they agree,
// 0 / 24 h when they are opposite. That works when the sun is underfoot,
// where an azimuth would be undefined and read as noon.

// The sun's height above the horizon at a point: the sine of its elevation.
// +1 overhead, 0 on the horizon, -1 underfoot. Below the horizon is negative
// and nothing about it is a lie: a point underground reads negative too.
float nisp_sun_elevation(int body, double t, double const pos_km[3]);

// Local solar time at a point, hours, 0..24, 12 == the sun at its highest.
// A body that does not rotate has no day and reads 12.
float nisp_solar_time(int body, double t, double const pos_km[3]);

// The local noon in the day that begins at `t_from`, for a point fixed to
// the surface at this latitude and longitude -- the next one, except when
// `t_from` is already noon there. A launch epoch is chosen with it
// (plan 6.1): the pad in daylight, rather than a clock someone has to wind.
double nisp_local_noon(int body, double t_from, float lat_rad, float lon_rad);

// The sun's direction from a point, as a unit vector. One direction lights
// the whole scene (step 2.2): 13.6 million km away, Frodo subtends a
// fraction of a degree, so a point light would cost more and lie.
void nisp_sun_direction(double t, double const at_km[3], float out[3]);

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

// The sun's direction from a point, as a unit vector. One direction lights
// the whole scene (step 2.2): 13.6 million km away, Frodo subtends a
// fraction of a degree, so a point light would cost more and lie.
void nisp_sun_direction(double t, double const at_km[3], float out[3]);

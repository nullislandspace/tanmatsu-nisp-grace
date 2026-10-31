#pragma once
// =====================================================================
//  Null Island Space Program  --  the air, as one exponential
// ---------------------------------------------------------------------
//  Part A2, in four lines of maths:
//
//      rho(h) = rho0 * exp(-h / H)      p(h) = p0 * exp(-h / H)
//
//  with H = 5.6 km on Bob and nothing above 70 km. No temperature model,
//  no layers, no Mach-dependent anything. One scale height is what makes
//  "how much air is left at 40 km" a lookup rather than a table, and it is
//  the same number the heat model (Part A4) and the drag model (Part A5)
//  read, so the two can never disagree about whether the ship is in air.
//
//  UNITS. Density is kg/m^3 and pressure kPa -- the units a HUD prints --
//  because they are inputs to 0.5 * rho * v^2 * CdA, which wants metres and
//  seconds. Altitude is kilometres like the rest of the sim; the conversion
//  happens inside this file and nowhere else.
// =====================================================================

#include <stdbool.h>

#include "sim/bodies.h"

bool  nisp_has_atmosphere(int body);
bool  nisp_in_atmosphere(int body, float alt_km);

// kg/m^3. 0 above the top of the atmosphere, and 0 for a body with no air
// at all -- the same answer, which is the point: no caller has to know
// which of the two it is asking about.
float nisp_atm_density(int body, float alt_km);

// kPa, for the HUD.
float nisp_atm_pressure(int body, float alt_km);

// Dynamic pressure q = 0.5 rho v^2, pascals. `v_rel_kms` is the speed
// through the air in km/s (sim units); the aero torque (Part A5) and the
// heat model (Part A4) both want it, and both want it in SI.
float nisp_dynamic_pressure(int body, float alt_km, float v_rel_kms);

// Drag force, newtons: 0.5 rho v^2 * CdA, CdA summed over the stack
// (Part V2). The direction is not this function's business -- flight/physics
// applies it along -v_rel.
float nisp_drag_force(int body, float alt_km, float v_rel_kms, float cda_m2);

// The acceleration that force produces on `mass_kg`, m/s^2.
float nisp_drag_accel(int body, float alt_km, float v_rel_kms, float cda_m2, float mass_kg);

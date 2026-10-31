#pragma once
// =====================================================================
//  Null Island Space Program  --  one float per part (Part A4)
// ---------------------------------------------------------------------
//  Air does the heating and one quantity does the air: the convective load
//  on a part goes as `rho * v^3`, density from sim/atmosphere and speed
//  relative to the air. A hard launch and a reentry are the same equation
//  read at different numbers (D-30), and on this atmosphere their peak rates
//  are within a factor of two of each other (F-15) -- what separates them is
//  how long they last. So heat is an accumulated quantity per part with a
//  limit, never a threshold on speed or dynamic pressure.
//
//      flow     = -normalize(v_rel)          where the air is coming from
//      rate(p)  = K * rho * |v_rel|^3 * area(p) * expose(p) / capacity(p)
//      temp(p) += (rate(p) - temp(p) / TAU) * dt
//      fail(p)  when temp(p) >= limit(p)
//
//  `expose` is 1 for everything except the capsule, whose integrated shield
//  is worth about 0.9 -- and only when it faces the flow. A shield that is
//  worth 0.1 of the load and nothing when it faces away is a factor, not
//  immunity (D-36), and nispcheck case 9's fourth profile is the reentry that
//  proves it: the stable attitude, at reentry speed, still kills the pod.
//
//  No shadowing, no conduction along the stack, no radiation view factors,
//  no solar heating, no shield that degrades with use (D-20). One float per
//  part, and the only thing that reads it is rocket/stack's split (Part F5).
//
//  PURE: takes (altitude, speed, attitude) as inputs, which is what lets
//  nispcheck drive it with scripted profiles instead of flying a badge.
// =====================================================================

#include <stdbool.h>

#include "rocket/stack.h"

// The tuning constant that turns rho*v^3 (W/m^2) into a per-second rise
// against the `capacity` column of rocket/parts.c. Case 9 is what sets it.
#define NISP_HEAT_K 1.5e-11
#define NISP_HEAT_TAU 30.0f  // seconds: parts cool toward ambient

// Where the air is and how hard it is coming. `cos_flow_axis` is
// dot(flow_hat, axis_hat) with axis_hat pointing from the capsule toward the
// tail: +1 is the air arriving at the nose, which is the shield facing it.
typedef struct {
    int   body;
    float alt_km;
    float v_rel_kms;      // sim units; converted here and nowhere else
    float cos_flow_axis;  // -1..1
} nisp_heat_input_t;

// Advance every part's heat by dt_s. No air, no heating -- but the cooling
// term still runs, which is what lets a part recover after a hot pass.
void nisp_heat_tick(nisp_stack_t* s, nisp_heat_input_t const* in, float dt_s);

// The one part that has reached its limit, hottest first, or -1.
int nisp_heat_breach(nisp_stack_t const* s);

// How much of the load a part eats, 0..1 (Part A4's `expose`).
float nisp_heat_expose(nisp_part_t const* p, float cos_flow_axis);

// rho * |v|^3 for a profile, W/m^2 -- the number Part A4's table quotes, and
// the one the HUD can show next to the hottest part's bar.
float nisp_heat_load_rate(int body, float alt_km, float v_rel_kms);


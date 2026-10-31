#include <math.h>

#include "sim/bodies.h"

// =====================================================================
//  The three rows (Part O2).
//
//  Bob is Kerbin with the decimal point moved: 600 km of radius and
//  3,530.46 km^3/s^2 of mu give 9.81 m/s^2 at the surface and 3,430.5
//  m/s of escape speed, which are the two numbers a player already knows
//  from Kerbin. Cheese is the Mun (200 km, 65.136, 12,000 km out, 12 deg
//  inclined, tidally locked). Frodo is Kerbol, 13,599,840 km away with
//  1.17234e9 of mu -- far enough that Bob's own SOI (84,160 km) is a
//  comfortable place to do everything the game will ever do.
//
//  The frame is the engine's orientation, in kilometres: x lateral, y up,
//  z forward, y normal to the ecliptic. Bob and Frodo orbit in the X-Z
//  plane; Cheese's plane is tipped 12 deg about x. Keeping the sim frame
//  the same shape as the render frame is what lets game/scene.c submit a
//  position with one subtraction and no rotation (D-16).
//
//  Bob turns about +y, so the pad at 0N 0E moves toward -z: that direction
//  is "east", and it is where a launch gets its 174.9 m/s for free. Both
//  orbits go the same way round, so a prograde launch is prograde about
//  Frodo too, and Bob's solar day comes out at Kerbin's 21,600 s (F-40).
// =====================================================================
static nisp_body_t const BODY[BODY_COUNT] = {
    // Bob: the planet.
    {
        .name           = "Bob",
        .radius_km      = 600.0f,
        .mu             = 3530.46f,
        .rotation_s     = 21549.4f,
        .tidally_locked = false,
        .parent         = BODY_FRODO,
        .orbit_a_km     = 13599840.0f,
        .orbit_inc_rad  = 0.0f,
        .orbit_phase0   = 0.0f,
        .soi_km         = 84160.0f,
        .atmo_scale_km  = 5.6f,
        .atmo_top_km    = 70.0f,
        .atmo_rho0      = 1.225f,
        .atmo_p0_kpa    = 101.325f,
    },
    // Cheese: the moon. No air, so legs are the only landing system it has.
    {
        .name           = "Cheese",
        .radius_km      = 200.0f,
        .mu             = 65.136f,
        .rotation_s     = 139000.0f,  // tidally locked: one face, always
        .tidally_locked = true,
        .parent         = BODY_BOB,
        .orbit_a_km     = 12000.0f,
        .orbit_inc_rad  = 12.0f * 0.017453292519943295f,
        .orbit_phase0   = 0.0f,
        .soi_km         = 2400.0f,
        .atmo_scale_km  = 0.0f,
        .atmo_top_km    = 0.0f,
        .atmo_rho0      = 0.0f,
        .atmo_p0_kpa    = 0.0f,
    },
    // Frodo: the sun, and the root of the tree.
    {
        .name           = "Frodo",
        .radius_km      = 2616.0f,
        .mu             = 1.17234e9f,
        .rotation_s     = 0.0f,
        .tidally_locked = false,
        .parent         = BODY_COUNT,
        .orbit_a_km     = 0.0f,
        .orbit_inc_rad  = 0.0f,
        .orbit_phase0   = 0.0f,
        .soi_km         = 0.0f,
        .atmo_scale_km  = 0.0f,
        .atmo_top_km    = 0.0f,
        .atmo_rho0      = 0.0f,
        .atmo_p0_kpa    = 0.0f,
    },
};

nisp_body_t const* nisp_body(int id) {
    if (id < 0 || id >= BODY_COUNT) return &BODY[BODY_BOB];
    return &BODY[id];
}

float nisp_body_g0(int id) {
    nisp_body_t const* b = nisp_body(id);
    // mu / r^2 is km/s^2; a TWR is quoted against m/s^2.
    return b->mu / (b->radius_km * b->radius_km) * 1000.0f;
}

float nisp_circular_speed(int id, float r_km) {
    if (r_km <= 0.0f) return 0.0f;
    return sqrtf(nisp_body(id)->mu / r_km);
}

float nisp_orbital_period(int id, float r_km) {
    if (r_km <= 0.0f) return 0.0f;
    return 6.283185307179586f * sqrtf(r_km * r_km * r_km / nisp_body(id)->mu);
}

float nisp_radius_for_period(int id, float period_s) {
    if (period_s <= 0.0f) return 0.0f;
    float const n = 6.283185307179586f / period_s;
    return cbrtf(nisp_body(id)->mu / (n * n));
}

float nisp_escape_speed(int id, float r_km) {
    if (r_km <= 0.0f) return 0.0f;
    return sqrtf(2.0f * nisp_body(id)->mu / r_km);
}

float nisp_body_hill_km(int id) {
    nisp_body_t const* b = nisp_body(id);
    if (b->parent >= BODY_COUNT || b->orbit_a_km <= 0.0f) return 0.0f;
    float const ratio = b->mu / nisp_body(b->parent)->mu;
    return b->orbit_a_km * cbrtf(ratio);
}

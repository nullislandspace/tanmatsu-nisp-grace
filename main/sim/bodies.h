#pragma once
// =====================================================================
//  Null Island Space Program  --  the universe, as three rows
// ---------------------------------------------------------------------
//  Bob, Cheese and Frodo: a planet, one moon and a sun, sized, weighted
//  and orbited to correspond to Kerbin, the Mun and Kerbol (Part O2).
//  Everything here is DATA -- a row per body -- so adding a fourth body
//  is a row and not a code change.
//
//  The numbers are derived from KSP 1's published parameters and were
//  cross-checked against four anchors a player can quote from a wiki
//  (F-11): an 80 km circular orbit at 2,278.6 m/s over 1,875.1 s, a
//  stationary orbit at 2,863.9 km, semi-synchronous at 1,581.5 km and
//  1,272.1 m/s, and surface escape at 3,430.5 m/s. nispcheck case 1 is
//  those four anchors; if a row here is wrong, that test says so.
//
//  Units: km, s, km^3/s^2 (sim/units.h).
// =====================================================================

#include <stdbool.h>

typedef enum {
    BODY_BOB = 0,    // the planet. Everything starts here.
    BODY_CHEESE,     // its moon, and the reason landing legs exist (Part V7)
    BODY_FRODO,      // the sun. The root of the patched-conic tree.
    BODY_COUNT
} nisp_body_id_t;

typedef struct {
    char const* name;

    // Gravity.
    float radius_km;   // the drawn radius and the surface, the same number
    float mu;          // km^3/s^2
    float rotation_s;  // sidereal day; 0 for a body that does not rotate
    bool  tidally_locked;

    // Where it is. Circular orbits in one fixed inertial frame (Part O5):
    // no eccentricity, no precession, no inclination change, ever.
    int   parent;      // BODY_COUNT for the root
    float orbit_a_km;  // 0 for the root, which sits at the origin
    float orbit_inc_rad;
    float orbit_phase0;  // true anomaly at t = 0

    // The region its gravity is the one the sim answers to (Part O4).
    // 0 for the root.
    float soi_km;

    // Air (Part A2). atmo_scale_km == 0 means none, and nothing above
    // atmo_top_km has any effect at all.
    float atmo_scale_km;   // the exponential density scale height
    float atmo_top_km;
    float atmo_rho0;       // kg/m^3 at sea level
    float atmo_p0_kpa;     // kPa at sea level, for the HUD only
} nisp_body_t;

// The row. Never NULL, never modified.
nisp_body_t const* nisp_body(int id);

// Surface gravity, m/s^2 -- the number a TWR is divided by.
float nisp_body_g0(int id);

// Circular orbital speed at radius r (km), in sim units (km/s).
float nisp_circular_speed(int id, float r_km);

// Period of a circular orbit of radius r (km), in seconds.
float nisp_orbital_period(int id, float r_km);

// Radius (km) of the circular orbit whose period is `period_s`. The
// stationary and semi-synchronous orbits are this function called with
// the body's own day, and half of it.
float nisp_radius_for_period(int id, float period_s);

// Escape speed at radius r (km), km/s.
float nisp_escape_speed(int id, float r_km);

// The body's Hill sphere, km -- the region where its own gravity beats
// the parent's tidal pull. KSP wiki's convention, a * (mu/parent_mu)^(1/3),
// which is the number a wiki reader will compare the SOI against. 0 for
// the root. A SOI outside its Hill sphere would be a patched-conic tree
// that lies, and nispcheck asserts that it is not.
float nisp_body_hill_km(int id);

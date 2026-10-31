#pragma once
// =====================================================================
//  Null Island Space Program  --  orbits, two ways of saying the same thing
// ---------------------------------------------------------------------
//  A state vector (where, how fast) is the truth; orbital elements (a, e,
//  i, ...) are a view of it (D-17). The flight sim integrates a state
//  vector, the map draws and the planner reasons in elements, and every
//  trip between the two goes through this file.
//
//  ONE CONIC FOR ALL THREE. Position and velocity are written from the
//  true anomaly nu and the semi-latus rectum p = h^2/mu:
//
//      r_vec = r (cos nu * P + sin nu * Q)
//      v_vec = sqrt(mu/p) (-sin nu * P + (e + cos nu) * Q)
//      r     = p / (1 + e cos nu)
//
//  which is exact for ellipses, parabolas and hyperbolas alike, so there
//  is no elliptic branch and hyperbolic branch of the position code --
//  only of Kepler's equation, which is the one place an ellipse and an
//  escape trajectory genuinely differ.
//
//  THE FRAME is sim/bodies.c's: x lateral, y up, z forward, with the
//  reference plane X-Z and its normal +y. So inclination is measured from
//  +y, the node line is the intersection with X-Z, and a prograde orbit
//  (angular momentum along +y) travels the same way Bob turns (Part O5).
//
//  THE MATH IS DOUBLE, the state is float. Elements and every anomaly
//  solve run in double; the state vector a flight integrates is float,
//  because it is touched 50 times a second. That is what makes the
//  round-trip test (Part H case 2) mean something: propagate 10,000
//  periods and come back within a metre, where the metre is the float
//  quantisation of a 680 km radius, not an accumulating error.
// =====================================================================

#include <stdbool.h>

#include "sim/bodies.h"

typedef struct {
    float r[3];  // km, relative to the central body
    float v[3];  // km/s, same frame
} nisp_state_t;

typedef struct {
    int   body;      // the central body this orbit is about
    float mu;        // km^3/s^2, copied so an element set stands alone

    // The conic. a < 0 for a hyperbola (the usual convention), e >= 0.
    double a;
    double e;

    // Orientation (radians; see the frame note above).
    double inc;      // from +y
    double lan;      // node line, from +x about +y
    double aop;      // periapsis, from the node line, in the orbit plane
    double m0;       // mean anomaly at t0
    double t0;       // s

    // Derived, filled in by nisp_state_to_elements / nisp_elements_init.
    double periapsis_km;   // radius, not altitude
    double apoapsis_km;    // 0 when there is no apoapsis
    double period_s;       // 0 for a hyperbola
    double p_km;           // semi-latus rectum
} nisp_elements_t;

// The unit vectors of the orbit's perifocal frame: P toward periapsis,
// Q a quarter turn ahead along the motion, H along the angular momentum.
typedef struct {
    double P[3];
    double Q[3];
    double H[3];
} nisp_perifocal_t;

void nisp_perifocal(nisp_elements_t const* el, nisp_perifocal_t* pf);

// Where it is and how fast, at time `t`.
void nisp_elements_to_state(nisp_elements_t const* el, double t, nisp_state_t* out);

// The view: state -> elements, at time `t`. False (and no output) for a
// degenerate input -- zero radius, or a radius and velocity that define
// no plane.
bool nisp_state_to_elements(nisp_state_t const* state, int body, double t, nisp_elements_t* out);

// Fill the derived fields (periapsis, apoapsis, period, p). Called by both
// directions above; call it yourself after editing a, e or mu by hand.
void nisp_elements_derive(nisp_elements_t* el);

// Convenience: build an element set for a circular orbit of radius r in
// the body's equatorial plane, at phase `nu0`, centred on t0.
void nisp_elements_circular(int body, float r_km, double nu0, double t0, nisp_elements_t* out);

// --- Anomaly solving (Part O3) -----------------------------------------

// Elliptic Kepler's equation, E - e sin E = M. Newton-Raphson seeded from
// M: four iterations is enough for e up to 0.99 to well under 1e-10 rad.
double nisp_kepler_elliptic(double M, double e);

// Hyperbolic Kepler's equation, H - e sinh H = M. Newton seeded from the
// large-M asymptote, which is where a flyby actually sits.
double nisp_kepler_hyperbolic(double M, double e);

// Mean anomaly at time t (wraps for an ellipse).
double nisp_mean_at(nisp_elements_t const* el, double t);

// True anomaly at time t, and time at true anomaly (the inverse, used to
// place a maneuver node on the orbit).
double nisp_true_at(nisp_elements_t const* el, double t);
double nisp_time_at_true(nisp_elements_t const* el, double nu);

// Time from t0 to the next periapsis / apoapsis passage. Negative when the
// ship is heading away from it, which is honest: "next Ap in -40 s" means
// it passed 40 s ago and the next one is a period later.
double nisp_time_to_apsis(nisp_elements_t const* el, double t, bool apoapsis);

// Speed at radius r on this conic (vis-viva), km/s.
float nisp_conic_speed(nisp_elements_t const* el, float r_km);

// The delta-v to go from the current conic to a circular orbit of radius
// r at the point where the conic is at radius r. Positive means prograde.
float nisp_circularisation_delta_v(nisp_elements_t const* el, float r_km);

#include <math.h>

#include "sim/ephemeris.h"

// =====================================================================
//  Bob and Cheese on circles (Part O5).
//
//  Bob's plane is the ecliptic: X-Z, normal +y. Cheese's is tipped 12 deg
//  about x, which is the Mun's inclination and the reason a Cheese mission
//  needs a plane change -- the number Part M2 exists to compute.
//
//  Both turn counter-clockwise about +y, the same sense Bob itself turns,
//  so a prograde launch from the pad gets 174.9 m/s of help rather than a
//  headwind (sim/surface.c).
// =====================================================================

static double orbit_angle(nisp_body_t const* b, double t) {
    double const a = b->orbit_a_km;
    double const n = sqrt(nisp_body(b->parent)->mu / (a * a * a));  // mean motion, rad/s
    return b->orbit_phase0 + n * t;
}

void nisp_body_position(int id, double t, double out_km[3]) {
    nisp_body_t const* b = nisp_body(id);

    if (b->parent >= BODY_COUNT || b->orbit_a_km <= 0.0f) {
        out_km[0] = out_km[1] = out_km[2] = 0.0;  // the root sits at the origin
        return;
    }

    double const a = b->orbit_a_km;
    double const u = orbit_angle(b, t);
    double const cu = cos(u), su = sin(u);
    double const ci = cos(b->orbit_inc_rad), si = sin(b->orbit_inc_rad);

    double px = a * cu;
    double py = a * su * si;
    double pz = a * su * ci;

    double parent[3];
    nisp_body_position(b->parent, t, parent);
    px += parent[0];
    py += parent[1];
    pz += parent[2];

    out_km[0] = px;
    out_km[1] = py;
    out_km[2] = pz;
}

void nisp_body_velocity(int id, double t, double out_kms[3]) {
    nisp_body_t const* b = nisp_body(id);

    if (b->parent >= BODY_COUNT || b->orbit_a_km <= 0.0f) {
        out_kms[0] = out_kms[1] = out_kms[2] = 0.0;
        return;
    }

    double const a = b->orbit_a_km;
    double const n = sqrt(nisp_body(b->parent)->mu / (a * a * a));
    double const u = orbit_angle(b, t);
    double const ci = cos(b->orbit_inc_rad), si = sin(b->orbit_inc_rad);

    double vx = -a * n * sin(u);
    double vy = a * n * cos(u) * si;
    double vz = a * n * cos(u) * ci;

    double pv[3];
    nisp_body_velocity(b->parent, t, pv);
    out_kms[0] = vx + pv[0];
    out_kms[1] = vy + pv[1];
    out_kms[2] = vz + pv[2];
}

void nisp_body_relative(int id, int ref, double t, double out_km[3]) {
    double a[3], b[3];
    nisp_body_position(id, t, a);
    nisp_body_position(ref, t, b);
    out_km[0] = a[0] - b[0];
    out_km[1] = a[1] - b[1];
    out_km[2] = a[2] - b[2];
}

void nisp_sun_direction(double t, double const at_km[3], float out[3]) {
    double sun[3];
    nisp_body_position(BODY_FRODO, t, sun);

    double dx = sun[0] - at_km[0];
    double dy = sun[1] - at_km[1];
    double dz = sun[2] - at_km[2];
    double const d = sqrt(dx * dx + dy * dy + dz * dz);
    if (d < 1e-9) {
        out[0] = 0.0f; out[1] = 1.0f; out[2] = 0.0f;
        return;
    }
    out[0] = (float)(dx / d);
    out[1] = (float)(dy / d);
    out[2] = (float)(dz / d);
}

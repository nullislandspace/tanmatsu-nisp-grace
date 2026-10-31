#include <math.h>

#include "sim/ephemeris.h"
#include "sim/surface.h"

// =====================================================================
//  Bob and Cheese on circles (Part O5).
//
//  Bob's plane is the ecliptic: X-Z, normal +y. Cheese's is tipped 12 deg
//  about x, which is the Mun's inclination and the reason a Cheese mission
//  needs a plane change -- the number Part M2 exists to compute.
//
//  PROGRADE MEANS THE SAME WAY ROUND FOR BOTH. Bob turns about +y, which
//  carries the pad toward -z (sim/surface.c), and both orbits go the same
//  way: a position of (a cos u, ..., -a sin u) sweeps +x toward -z, and its
//  angular momentum r x v is +y. Get that sign wrong and the moon rises in
//  the west, a prograde launch is a retrograde orbit about the sun, and the
//  length of Bob's day comes out 21,499 s instead of the 21,600 s a Kerbin
//  player can quote (F-40).
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
    double py = a * su * si;   // the inclination tips the plane about x
    double pz = -a * su * ci;

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
    double vz = -a * n * cos(u) * ci;

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

// --- Day and night (plan 6.1) -------------------------------------------

// Wrap an angle into [-pi, pi). The half-open interval is what makes
// nisp_solar_time land in [0, 24) rather than occasionally printing 24:00.
static double wrap_pi(double a) {
    while (a >= M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// The angle of a body-relative point measured about the spin axis (+y), in
// the same sense nisp_surface_point measures longitude: it writes
// (r cos lon, ..., -r sin lon), so atan2(z, x) is -lon. Two points on the
// same meridian have the same value, which is all a meridian is for.
static double meridian_angle(double const pos_km[3]) {
    if (pos_km[0] == 0.0 && pos_km[2] == 0.0) return 0.0;  // on the spin axis
    return atan2(pos_km[2], pos_km[0]);
}

// The sun's angle about the same axis, seen from the body's centre.
static double sun_meridian(int body, double t) {
    double rel[3];
    nisp_body_relative(BODY_FRODO, body, t, rel);
    return meridian_angle(rel);
}

float nisp_sun_elevation(int body, double t, double const pos_km[3]) {
    double body_abs[3], sun_abs[3];
    nisp_body_position(body, t, body_abs);
    nisp_body_position(BODY_FRODO, t, sun_abs);

    double const point[3] = {body_abs[0] + pos_km[0], body_abs[1] + pos_km[1], body_abs[2] + pos_km[2]};

    // The local vertical is the BODY-relative radius, not the sun-frame one:
    // a point 600 km above Bob is 13.6 million km from Frodo, and pointing at
    // Frodo from there is not "up". This is the whole function's trap.
    double const r = sqrt(pos_km[0] * pos_km[0] + pos_km[1] * pos_km[1] + pos_km[2] * pos_km[2]);
    if (r < 1e-9) return 0.0f;

    double dx = sun_abs[0] - point[0];
    double dy = sun_abs[1] - point[1];
    double dz = sun_abs[2] - point[2];
    double const d = sqrt(dx * dx + dy * dy + dz * dz);
    if (d < 1e-9) return 1.0f;  // standing on the sun

    double const dot = (dx * pos_km[0] + dy * pos_km[1] + dz * pos_km[2]) / (d * r);
    if (dot > 1.0) return 1.0f;
    if (dot < -1.0) return -1.0f;
    return (float)dot;
}

float nisp_solar_time(int body, double t, double const pos_km[3]) {
    nisp_body_t const* b = nisp_body(body);
    if (b->rotation_s <= 0.0f) return 12.0f;  // no spin, no day

    // The hour angle: how far the point's meridian is behind the sun's, in
    // the direction the body turns. Zero is noon; half a turn is midnight.
    double const h = wrap_pi(sun_meridian(body, t) - meridian_angle(pos_km));
    float hours = 12.0f + (float)(h * (180.0 / M_PI) / 15.0);
    if (hours < 0.0f) hours += 24.0f;
    if (hours >= 24.0f) hours -= 24.0f;
    return hours;
}

double nisp_local_noon(int body, double t_from, float lat_rad, float lon_rad) {
    nisp_body_t const* b = nisp_body(body);
    if (b->rotation_s <= 0.0f) return t_from;

    // A day has one high point, so a coarse scan finds the bracket it lives
    // in and golden section finds the top of it. 64 samples is a quarter of
    // an hour on Bob, which is far finer than the resolution anything here
    // reads the local clock at.
    //
    // The scan reaches a little past one rotation on purpose. A solar day is
    // not a sidereal one -- Bob moves under the sun while it turns -- and on
    // Bob the difference is 51 s. Scan exactly one rotation and the next
    // noon sits just outside the window, which reads as "the sun never rises
    // here" (F-40).
    double const day = b->rotation_s;
    double const span = day * 1.05;
    int const n = 64;
    double best_t = t_from;
    double best = -2.0;
    for (int i = 0; i <= n; i++) {
        double const t = t_from + span * (double)i / (double)n;
        double p[3];
        nisp_surface_point(body, t, lat_rad, lon_rad, p);
        double const e = nisp_sun_elevation(body, t, p);
        if (e > best) {
            best = e;
            best_t = t;
        }
    }

    double lo = best_t - span / (double)n;
    double hi = best_t + span / (double)n;

    double const phi = 0.6180339887498949;  // 1/phi: the golden-section step
    double a = hi - phi * (hi - lo);
    double b_t = lo + phi * (hi - lo);
    double fa, fb;
    {
        double p[3];
        nisp_surface_point(body, a, lat_rad, lon_rad, p);
        fa = nisp_sun_elevation(body, a, p);
        nisp_surface_point(body, b_t, lat_rad, lon_rad, p);
        fb = nisp_sun_elevation(body, b_t, p);
    }
    for (int i = 0; i < 48 && hi - lo > 1e-4; i++) {
        if (fa >= fb) {
            hi = b_t;
            b_t = a;
            fb = fa;
            a = hi - phi * (hi - lo);
            double p[3];
            nisp_surface_point(body, a, lat_rad, lon_rad, p);
            fa = nisp_sun_elevation(body, a, p);
        } else {
            lo = a;
            a = b_t;
            fa = fb;
            b_t = lo + phi * (hi - lo);
            double p[3];
            nisp_surface_point(body, b_t, lat_rad, lon_rad, p);
            fb = nisp_sun_elevation(body, b_t, p);
        }
    }
    return 0.5 * (lo + hi);
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

#include <math.h>

#include "sim/surface.h"

// Bob's rotation rate, rad/s, about +y. Positive means the ground travels
// toward -z at longitude 0, which is what "east" means here.
static double spin_rate(nisp_body_t const* b) {
    if (b->rotation_s <= 0.0f) return 0.0;
    return 6.283185307179586 / b->rotation_s;
}

static double spin_angle(nisp_body_t const* b, double t) {
    return spin_rate(b) * t;
}

void nisp_surface_point(int body, double t, float lat_rad, float lon_rad, double out_km[3]) {
    nisp_body_t const* b = nisp_body(body);
    double const r = b->radius_km;
    double const cl = cos(lat_rad);
    double const lon = lon_rad + spin_angle(b, t);

    out_km[0] = r * cl * cos(lon);
    out_km[1] = r * sin(lat_rad);
    out_km[2] = -r * cl * sin(lon);
}

void nisp_surface_velocity(int body, double t, float lat_rad, float lon_rad, double out_kms[3]) {
    nisp_body_t const* b = nisp_body(body);
    double const w = spin_rate(b);
    double const r = b->radius_km;
    double const cl = cos(lat_rad);
    double const lon = lon_rad + spin_angle(b, t);

    // d/dt of the position above, with only `lon` depending on t.
    double const vx = -w * r * cl * sin(lon);
    double const vz = -w * r * cl * cos(lon);

    out_kms[0] = vx;
    out_kms[1] = 0.0;
    out_kms[2] = vz;
}

void nisp_ground_velocity(int body, double const pos_km[3], double out_kms[3]) {
    nisp_body_t const* b = nisp_body(body);
    double const w = spin_rate(b);
    // omega is along +y, so omega x r = (w*y? ...) -- written out: (0,w,0) x
    // (x,y,z) = (w*z - 0, 0 - 0, 0 - w*x). At the pad, where r is +x, that
    // is -z, which is the same answer nisp_surface_velocity gives there.
    out_kms[0] = w * pos_km[2];
    out_kms[1] = 0.0;
    out_kms[2] = -w * pos_km[0];
}

float nisp_altitude(int body, double const pos_km[3]) {
    nisp_body_t const* b = nisp_body(body);
    double dx = pos_km[0], dy = pos_km[1], dz = pos_km[2];
    // Callers work relative to the body (Part O4: one reference body), so
    // this is radius minus radius and nothing more.
    return (float)(sqrt(dx * dx + dy * dy + dz * dz) - b->radius_km);
}

void nisp_local_frame(int body, double const pos_km[3], double up[3], double north[3], double east[3]) {
    nisp_body_t const* b = nisp_body(body);
    double const r = sqrt(pos_km[0] * pos_km[0] + pos_km[1] * pos_km[1] + pos_km[2] * pos_km[2]);
    if (r < 1e-9) {
        up[0] = 0; up[1] = 1; up[2] = 0;
        north[0] = 1; north[1] = 0; north[2] = 0;
        east[0] = 0; east[1] = 0; east[2] = -1;
        return;
    }

    up[0] = pos_km[0] / r;
    up[1] = pos_km[1] / r;
    up[2] = pos_km[2] / r;

    // East is the direction the body turns: omega x up, with omega along +y.
    // omega is along +y, so east is omega x up = (up.z, 0, -up.x): at the
    // pad, where up is +x, that is -z -- the direction the ground moves.
    double e[3] = {b->rotation_s > 0.0f ? up[2] : 0.0, 0.0, b->rotation_s > 0.0f ? -up[0] : 0.0};
    double el = sqrt(e[0] * e[0] + e[2] * e[2]);
    if (el < 1e-9) {  // at a pole: no east, so borrow a fixed one
        e[0] = 1.0; e[2] = 0.0;
        el = 1.0;
    }
    east[0] = e[0] / el;
    east[1] = 0.0;
    east[2] = e[2] / el;

    // North completes the triple: up x east... which points north for a
    // right-handed (up, north, east) frame.
    north[0] = up[1] * east[2] - up[2] * east[1];
    north[1] = up[2] * east[0] - up[0] * east[2];
    north[2] = up[0] * east[1] - up[1] * east[0];
}

void nisp_latlon(int body, double const pos_km[3], float* lat_rad, float* lon_rad) {
    (void)body;  // a body-relative position needs no epoch and no centre
    double const r = sqrt(pos_km[0] * pos_km[0] + pos_km[1] * pos_km[1] + pos_km[2] * pos_km[2]);
    if (r < 1e-9) {
        if (lat_rad) *lat_rad = 0.0f;
        if (lon_rad) *lon_rad = 0.0f;
        return;
    }
    if (lat_rad) *lat_rad = (float)asin(pos_km[1] / r);
    if (lon_rad) *lon_rad = (float)atan2(-pos_km[2], pos_km[0]);
}

bool nisp_below_surface(int body, double const pos_km[3]) {
    return nisp_altitude(body, pos_km) < 0.0f;
}

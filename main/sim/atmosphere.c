#include <math.h>

#include "sim/atmosphere.h"

bool nisp_has_atmosphere(int body) {
    return nisp_body(body)->atmo_scale_km > 0.0f;
}

bool nisp_in_atmosphere(int body, float alt_km) {
    nisp_body_t const* b = nisp_body(body);
    if (b->atmo_scale_km <= 0.0f) return false;
    return alt_km < b->atmo_top_km;
}

float nisp_atm_density(int body, float alt_km) {
    nisp_body_t const* b = nisp_body(body);
    if (b->atmo_scale_km <= 0.0f) return 0.0f;
    if (alt_km >= b->atmo_top_km) return 0.0f;
    if (alt_km < 0.0f) alt_km = 0.0f;  // below the surface: sea-level air
    return b->atmo_rho0 * expf(-alt_km / b->atmo_scale_km);
}

float nisp_atm_pressure(int body, float alt_km) {
    nisp_body_t const* b = nisp_body(body);
    if (b->atmo_scale_km <= 0.0f) return 0.0f;
    if (alt_km >= b->atmo_top_km) return 0.0f;
    if (alt_km < 0.0f) alt_km = 0.0f;
    return b->atmo_p0_kpa * expf(-alt_km / b->atmo_scale_km);
}

float nisp_dynamic_pressure(int body, float alt_km, float v_rel_kms) {
    float const rho = nisp_atm_density(body, alt_km);
    if (rho <= 0.0f) return 0.0f;
    float const v = v_rel_kms * 1000.0f;  // km/s -> m/s
    return 0.5f * rho * v * v;
}

float nisp_drag_force(int body, float alt_km, float v_rel_kms, float cda_m2) {
    return nisp_dynamic_pressure(body, alt_km, v_rel_kms) * cda_m2;
}

float nisp_drag_accel(int body, float alt_km, float v_rel_kms, float cda_m2, float mass_kg) {
    if (mass_kg <= 0.0f) return 0.0f;
    return nisp_drag_force(body, alt_km, v_rel_kms, cda_m2) / mass_kg;
}

// =====================================================================
//  Null Island Space Program  --  time, on a ladder (see warp.h)
// =====================================================================

#include "flight/warp.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sim/atmosphere.h"
#include "sim/bodies.h"
#include "sim/surface.h"

static float const WARP[NISP_WARP_COUNT] = {1.0f, 5.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};

float nisp_warp_level(int index) {
    if (index < 0) index = 0;
    if (index >= NISP_WARP_COUNT) index = NISP_WARP_COUNT - 1;
    return WARP[index];
}

int nisp_warp_levels(void) { return NISP_WARP_COUNT; }

int nisp_warp_index_for(float warp) {
    int best = 0;
    for (int i = 0; i < NISP_WARP_COUNT; i++) {
        if (WARP[i] <= warp) best = i;
    }
    return best;
}

// Is the next periapsis somewhere a conic is not allowed to be? (F-34)
// Being above the air right now is not the test: a coast is analytic, so a
// warp-10,000 frame dives through an atmosphere it never samples and comes
// out the other side, which is not a coarse trajectory but a different one.
// KSP's rule is the right one -- you may not fast-forward towards a
// periapsis that is inside the air. On an airless body atmo_top_km is 0, so
// the same test stops a warp-driven moon impact before it happens.
static bool periapsis_blocks_warp(nisp_vessel_t const* v, bool thrusting, bool steering,
                                  float alt) {
    if (v == NULL || v->landed) return false;   // landed is not a coast
    if (thrusting || steering) return false;    // the powered ceiling is already tighter
    if (alt <= 0.0f) return false;
    return nisp_vessel_periapsis_alt_km(v) < nisp_body(v->body)->atmo_top_km;
}

float nisp_warp_ceiling(nisp_vessel_t const* v, bool thrusting, bool steering) {
    if (v == NULL) return 1.0f;

    float const alt = nisp_altitude(v->body, v->pos_km);
    bool const in_air = nisp_atm_density(v->body, alt) > 0.0f;

    if (in_air) return 10.0f;                  // Part F4: the air is not a place to skip through
    if (thrusting || steering) return 100.0f;  // the substep cap's comfort zone
    if (periapsis_blocks_warp(v, thrusting, steering, alt)) return 10.0f;
    if (alt < 100.0f) return 1000.0f;          // low orbit is still fast enough to need ticks
    return 10000.0f;
}

void nisp_warp_plan(nisp_vessel_t const* v, float requested, double frame_dt, bool thrusting,
                    bool steering, nisp_warp_step_t* out) {
    if (out == NULL) return;

    memset(out, 0, sizeof(*out));
    if (frame_dt <= 0.0) frame_dt = 1.0 / 60.0;

    float const ceiling = nisp_warp_ceiling(v, thrusting, steering);
    float warp = requested > ceiling ? ceiling : requested;
    if (warp < 1.0f) warp = 1.0f;

    float const alt = (v != NULL) ? nisp_altitude(v->body, v->pos_km) : 0.0f;
    bool const in_air = (v != NULL) && nisp_atm_density(v->body, alt) > 0.0f;

    // Analytic only while nothing is pushing on the ship (F-31): a coast
    // applies no forces, so an engine or a SAS both take it off the table.
    out->coasting = !thrusting && !steering;
    if (v != NULL) {
        if (in_air || alt <= 0.0f) out->coasting = false;
    }

    if (out->coasting) {
        // Analytic: one propagation whatever the warp (Part F1).
        out->substeps = 0;
    } else {
        int steps = (int)ceil((double)warp * frame_dt / (double)NISP_PHYS_DT);
        if (steps > NISP_SUBSTEP_MAX) {
            steps = NISP_SUBSTEP_MAX;
            warp = (float)((double)steps * (double)NISP_PHYS_DT / frame_dt);
            out->capped = true;
        }
        out->substeps = steps;
    }

    out->warp = warp;
    out->dt_sim = (double)warp * frame_dt;

    // The reason belongs to a refusal, and a refusal is "you asked for more
    // than this ship is allowed" -- not "you asked for less". Asking for
    // warp 10 in the sky needs no explanation; asking for 10,000 in the air
    // does, and that is the string the HUD puts next to the warp readout.
    if (requested > warp) {
        // The same order nisp_warp_ceiling walks, so the string names the
        // rule that actually bound this frame.
        char const* why = "atmosphere";
        if (in_air) why = "atmosphere";
        else if (thrusting) why = "engine burning";
        else if (steering) why = "steering";
        else if (periapsis_blocks_warp(v, thrusting, steering, alt)) {
            why = nisp_body(v->body)->atmo_top_km > 0.0f ? "periapsis in air" : "periapsis underground";
        } else if (alt < 100.0f) why = "below 100 km";
        snprintf(out->reason, sizeof(out->reason), "max %s", why);
    }
}

// =====================================================================
//  Null Island Space Program  --  see flight/archive.h
// =====================================================================

#include "flight/archive.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flight/physics.h"
#include "rocket/stack.h"

void nisp_archive_init(nisp_archive_t* a) {
    if (a == NULL) return;
    memset(a, 0, sizeof(*a));
}

int nisp_archive_count(nisp_archive_t const* a) {
    if (a == NULL) return 0;
    if (a->count < 0) return 0;
    if (a->count > NISP_ARCHIVE_MAX) return NISP_ARCHIVE_MAX;
    return a->count;
}

nisp_flight_t const* nisp_archive_get(nisp_archive_t const* a, int i) {
    if (a == NULL || i < 0 || i >= nisp_archive_count(a)) return NULL;
    if (!a->flight[i].used) return NULL;
    return &a->flight[i];
}

bool nisp_archive_remove(nisp_archive_t* a, int i) {
    if (a == NULL) return false;
    int const n = nisp_archive_count(a);
    if (i < 0 || i >= n) return false;
    for (int k = i; k < n - 1; k++) a->flight[k] = a->flight[k + 1];
    a->flight[n - 1] = (nisp_flight_t){0};
    a->count = n - 1;
    return true;
}

// Is design stage `st` still bolted to the ship? The parts carry their
// design stage index through every staging and every split (rocket/stack.c),
// so "is there a part with this stage left" is the whole question.
static bool stage_attached(nisp_stack_t const* s, int st) {
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].stage == st) return true;
    }
    return false;
}

bool nisp_archive_add(nisp_archive_t* a, nisp_mission_t const* m, char const* design_name) {
    if (a == NULL || m == NULL) return false;
    if (a->count >= NISP_ARCHIVE_MAX) return false;

    // Three things that are not a craft you can leave out there: a wreck,
    // a rocket that has not left the pad yet (that one goes back to
    // Engineering), and a hop on a radial trajectory, which defines no
    // orbit to put back (sim/kepler.h refuses it for the same reason).
    if (!m->ship.alive || m->state == NISP_MISSION_WRECKED) return false;
    if (!m->airborne) return false;  // never left the pad: that is a blueprint, not a mission

    nisp_flight_t* f = &a->flight[a->count];
    memset(f, 0, sizeof(*f));

    f->used     = true;
    snprintf(f->vessel, sizeof(f->vessel), "%s", m->vessel_name);
    snprintf(f->design, sizeof(f->design), "%s",
             (design_name != NULL && design_name[0] != '\0') ? design_name : m->vessel_name);

    f->t        = m->t;
    f->body     = m->ship.body;
    f->state    = m->state;
    f->airborne = m->airborne;
    f->landed   = m->ship.landed;
    f->lat_rad  = m->ship.lat_rad;
    f->lon_rad  = m->ship.lon_rad;

    if (!m->ship.landed) {
        nisp_state_t st;
        nisp_vessel_state(&m->ship, &st);
        f->has_orbit = nisp_state_to_elements(&st, m->ship.body, m->t, &f->el);
        if (!f->has_orbit) return false;  // nothing to restore, so nothing to keep
    }

    nisp_stack_t const* s = &m->ship.stack;
    int const stages = s->stage_count < NISP_DESIGN_STAGE_MAX ? s->stage_count : NISP_DESIGN_STAGE_MAX;
    for (int st = 0; st < stages; st++) {
        f->attached[st] = stage_attached(s, st);
        if (!f->attached[st]) continue;
        float left = 0.0f, full = 0.0f;
        nisp_stack_propellant(s, st, &left, &full);
        f->fill[st] = full > 0.0f ? left / full : 0.0f;
        f->legs[st] = nisp_stack_legs_deployed(s, st);
    }

    f->chute    = s->parachute;
    f->ignited  = m->ignited;
    f->throttle = m->throttle;
    f->sas      = m->sas;
    f->rcs      = m->rcs;

    f->max_ap_km     = m->max_ap_km;
    f->max_speed_ms  = m->max_speed_ms;
    f->max_heat      = m->max_heat;
    f->stages_dropped = m->stages_dropped;
    f->parts_lost    = m->parts_lost;

    a->count++;
    return true;
}

bool nisp_archive_restore(nisp_archive_t* a, int i, nisp_design_t const* design, nisp_mission_t* m) {
    if (a == NULL || m == NULL || design == NULL) return false;
    nisp_flight_t const* f = nisp_archive_get(a, i);
    if (f == NULL) return false;

    // Where the craft is. Down: on the surface at the coordinates it touched
    // down on, carrying the ground's own velocity (flight/physics.c). Up:
    // propagated from its elements to the record's clock, nose prograde --
    // which is the attitude a left-behind craft is assumed to be holding,
    // and the one a SAS would put it in anyway.
    nisp_vessel_t v;
    if (f->landed) {
        nisp_vessel_ground(&v, design, f->body, f->t, f->lat_rad, f->lon_rad);
    } else if (f->has_orbit) {
        nisp_state_t st;
        nisp_elements_to_state(&f->el, f->t, &st);
        double const speed = sqrt((double)st.v[0] * st.v[0] + (double)st.v[1] * st.v[1] +
                                  (double)st.v[2] * st.v[2]);
        float nose[3] = {0.0f, 0.0f, 1.0f};
        if (speed > 1e-9) {
            for (int k = 0; k < 3; k++) nose[k] = (float)((double)st.v[k] / speed);
        }
        nisp_vessel_orbit(&v, design, f->body, &st, f->t, nose);
    } else {
        return false;
    }

    // The rocket as it was left. nisp_vessel_ground / nisp_vessel_orbit built
    // the whole blueprint; this is what the flight took away from it.
    int const stages = design->stage_count < NISP_DESIGN_STAGE_MAX ? design->stage_count
                                                                   : NISP_DESIGN_STAGE_MAX;
    for (int st = 0; st < stages; st++) {
        if (!f->attached[st]) {
            nisp_stack_drop_stage(&v.stack, st);
            continue;
        }
        nisp_stack_set_fill(&v.stack, st, f->fill[st]);
        if (f->legs[st]) nisp_stack_deploy_legs(&v.stack, st, true);
    }
    nisp_stack_deploy_chute(&v.stack, f->chute);
    nisp_stack_recompute(&v.stack);

    nisp_mission_begin(m, &v, f->t);
    snprintf(m->vessel_name, sizeof(m->vessel_name), "%s", f->vessel);

    // mission_begin assumes a fresh flight: lit engines, no log, flying. A
    // restored craft has a history and, if it is down, a state.
    m->state          = f->state;
    m->airborne       = f->airborne;
    m->ignited        = f->ignited;
    m->throttle       = f->throttle;
    m->sas            = f->sas;
    m->rcs            = f->rcs;
    m->max_ap_km      = f->max_ap_km;
    m->max_speed_ms   = f->max_speed_ms;
    m->max_heat       = f->max_heat;
    m->stages_dropped = f->stages_dropped;
    m->parts_lost     = f->parts_lost;

    nisp_readout_t r;
    nisp_mission_readout(m, &r);
    if (r.alt_km > m->max_ap_km) m->max_ap_km = r.alt_km;
    if (r.speed_ms > m->max_speed_ms) m->max_speed_ms = r.speed_ms;

    nisp_archive_remove(a, i);
    return true;
}

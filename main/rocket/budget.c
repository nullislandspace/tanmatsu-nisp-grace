// =====================================================================
//  Null Island Space Program  --  delta-v, TWR, burn time (see budget.h)
// =====================================================================

#include "rocket/budget.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "sim/bodies.h"
#include "sim/units.h"

#define MAX_GROUPS 2

typedef struct {
    float thrust_kn;
    float isp_s;
    float prop_t;   // what it has to burn
    float dry_t;    // what it sheds when it runs out (boosters only)
} burn_group_t;

// One stage's burn, phase by phase: everything firing until something runs
// out, then what is left firing. For a phase of constant thrust and constant
// mass flow, delta-v is (T / mdot) * ln(m0 / m1) -- Kostya's equation with no
// averaging, which is why a stage carrying solid boosters is exact.
static float group_delta_v(burn_group_t g[MAX_GROUPS], int groups, float m0, float* burn_out,
                           float* thrust_out, float* isp_out) {
    float m = m0;
    float dv = 0.0f;
    float burn = 0.0f;

    float t_first = 0.0f, isp_first = 0.0f;
    bool counted = false;

    for (;;) {
        float T = 0.0f, mdot = 0.0f, t_end = 1e30f;
        int live = 0;

        for (int i = 0; i < groups; i++) {
            if (g[i].prop_t <= 0.0f || g[i].isp_s <= 0.0f || g[i].thrust_kn <= 0.0f) continue;
            float const flow = g[i].thrust_kn / (g[i].isp_s * (float)NISP_G0);  // t/s
            T += g[i].thrust_kn;
            mdot += flow;
            live++;
            float const t = g[i].prop_t / flow;
            if (t < t_end) t_end = t;
        }
        if (live == 0 || mdot <= 0.0f) break;

        if (!counted) {
            t_first = T;
            for (int i = 0; i < groups; i++) {
                if (g[i].thrust_kn <= 0.0f) continue;
                isp_first += g[i].thrust_kn * g[i].isp_s;
            }
            counted = true;
        }

        float const burned = mdot * t_end;
        float const m1 = m - burned;
        if (m1 <= 0.0f || m1 >= m) break;

        dv += (T / mdot) * logf(m / m1);
        m = m1;
        burn += t_end;

        for (int i = 0; i < groups; i++) {
            if (g[i].prop_t <= 0.0f) continue;
            float const flow = g[i].thrust_kn > 0.0f && g[i].isp_s > 0.0f
                                   ? g[i].thrust_kn / (g[i].isp_s * (float)NISP_G0)
                                   : 0.0f;
            float const used = flow * t_end;
            if (used >= g[i].prop_t) {
                g[i].prop_t = 0.0f;
                m -= g[i].dry_t;  // a spent booster drops away mid-burn (Part F3)
            } else {
                g[i].prop_t -= used;
            }
        }
    }

    if (burn_out != NULL) *burn_out = burn;
    if (thrust_out != NULL) *thrust_out = t_first;
    if (isp_out != NULL) *isp_out = t_first > 0.0f ? isp_first / t_first : 0.0f;
    return dv;
}

static void fill_identity(nisp_stage_budget_t* b, nisp_stage_t const* st) {
    nisp_part_row_t const* row = nisp_part_row(st->kind);
    nisp_engine_row_t const* eng = nisp_engine_row(st->engine);
    snprintf(b->name, sizeof(b->name), "%s", row->name);
    snprintf(b->engine, sizeof(b->engine), "%s", row->engines_allowed ? eng->name : "(none)");
    b->engine_count = row->engines_allowed ? st->engine_count : 0;
    b->boosters = st->boosters;
    b->legs = st->legs;
    b->rcs = st->rcs;
}

void nisp_budget_compute(nisp_design_t const* d, int body, nisp_budget_t* out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (d == NULL || d->stage_count < 1) return;

    float const g0 = nisp_body_g0(body);
    int const n = d->stage_count;
    out->stage_count = n;
    out->reach_orbit_ms = NISP_REACH_ORBIT_MS;

    nisp_stage_mass_t mass[NISP_DESIGN_STAGE_MAX];
    for (int i = 0; i < n; i++) nisp_stage_masses(&d->stage[i], &mass[i]);

    // Everything attached at the pad, and everything that stays attached
    // above a stage: the mass a stage has to push.
    float total = NISP_CAPSULE_MASS_T;
    for (int i = 0; i < n; i++) {
        total += mass[i].prop_t + mass[i].dry_t + mass[i].boosters_prop_t + mass[i].boosters_dry_t +
                 mass[i].separator_t;
    }
    out->total_mass_t = total;

    int parts = 1;  // the capsule
    for (int i = 0; i < n; i++) {
        parts++;  // the stage body
        parts++;  // its separator
        if (d->stage[i].boosters > 0) parts++;
        if (d->stage[i].legs != NISP_LEGS_NONE) parts++;
    }
    out->part_count = parts;

    for (int i = n - 1; i >= 0; i--) {
        nisp_stage_budget_t* b = &out->stage[i];
        b->stage = i;
        fill_identity(b, &d->stage[i]);

        // The mass this stage ignites with: the capsule, every stage above it,
        // and itself -- full, with its boosters and its separators.
        float m0 = NISP_CAPSULE_MASS_T;
        for (int j = 0; j <= i; j++) {
            m0 += mass[j].prop_t + mass[j].dry_t + mass[j].boosters_prop_t + mass[j].boosters_dry_t +
                  mass[j].separator_t;
        }

        burn_group_t groups[MAX_GROUPS];
        memset(groups, 0, sizeof(groups));
        int groups_n = 0;

        nisp_engine_row_t const* eng = nisp_engine_row(d->stage[i].engine);
        if (nisp_part_row(d->stage[i].kind)->engines_allowed && d->stage[i].engine != NISP_ENGINE_NONE) {
            groups[groups_n].thrust_kn = eng->thrust_kn * (float)d->stage[i].engine_count;
            groups[groups_n].isp_s = eng->isp_s;
            groups[groups_n].prop_t = mass[i].prop_t;
            groups[groups_n].dry_t = 0.0f;  // a stage's own structure waits for the staging press
            groups_n++;
        }
        if (d->stage[i].boosters > 0) {
            nisp_engine_row_t const* sol = nisp_engine_row(NISP_ENGINE_BOOSTER);
            groups[groups_n].thrust_kn = sol->thrust_kn * (float)d->stage[i].boosters;
            groups[groups_n].isp_s = sol->isp_s;
            groups[groups_n].prop_t = mass[i].boosters_prop_t;
            groups[groups_n].dry_t = mass[i].boosters_dry_t;  // and these do not
            groups_n++;
        }

        float thrust = 0.0f, burn = 0.0f, isp = 0.0f;
        b->delta_v_ms = group_delta_v(groups, groups_n, m0, &burn, &thrust, &isp);
        b->burn_s = burn;
        b->thrust_kn = thrust;
        b->isp_s = isp;
        b->twr = (m0 > 0.0f && g0 > 0.0f) ? thrust / (m0 * g0) : 0.0f;
        b->liftable = b->twr >= 1.0f;

        b->wet_t = mass[i].prop_t + mass[i].dry_t + mass[i].boosters_prop_t + mass[i].boosters_dry_t +
                   mass[i].separator_t;
        b->dry_t = mass[i].dry_t + mass[i].boosters_dry_t + mass[i].separator_t;
        b->prop_t = mass[i].prop_t + mass[i].boosters_prop_t;

        out->total_delta_v_ms += b->delta_v_ms;
    }

    int const bottom = n - 1;
    out->liftoff_twr = out->stage[bottom].twr;
    out->first_stage_ok = out->stage[bottom].liftable;
    out->reach_orbit_ok = out->total_delta_v_ms >= NISP_REACH_ORBIT_MS;
}

void nisp_budget_stack(nisp_stack_t const* s, int body, nisp_budget_t* out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (s == NULL || s->count <= 0) return;

    float const g0 = nisp_body_g0(body);
    out->stage_count = s->stage_count;
    out->total_mass_t = s->mass_t;
    out->reach_orbit_ms = NISP_REACH_ORBIT_MS;

    for (int i = s->stage_count - 1; i >= 0; i--) {
        // Is this stage still attached?
        nisp_part_t const* body_part = NULL;
        nisp_part_t const* booster_part = NULL;
        for (int k = 0; k < s->count; k++) {
            nisp_part_t const* p = &s->part[k];
            if (p->stage != i) continue;
            if (p->slot == NISP_PART_KIND_BODY) body_part = p;
            if (p->slot == NISP_PART_KIND_BOOSTER) booster_part = p;
        }
        if (body_part == NULL) continue;

        nisp_stage_budget_t* b = &out->stage[i];
        b->stage = i;
        snprintf(b->name, sizeof(b->name), "%s", nisp_part_row(body_part->kind)->name);
        snprintf(b->engine, sizeof(b->engine), "%s",
                 body_part->engine != NISP_ENGINE_NONE ? nisp_engine_row(body_part->engine)->name : "(none)");
        b->engine_count = body_part->engine_count;
        b->boosters = booster_part != NULL ? booster_part->engine_count : 0;
        b->legs = s->legs_id[i];
        b->rcs = false;

        float m0 = 0.0f;
        for (int k = 0; k < s->count; k++) {
            nisp_part_t const* p = &s->part[k];
            if (p->stage >= 0 && p->stage > i) continue;  // already dropped
            m0 += p->mass_t;
        }

        burn_group_t groups[MAX_GROUPS];
        memset(groups, 0, sizeof(groups));
        int groups_n = 0;
        if (body_part->engine != NISP_ENGINE_NONE) {
            nisp_engine_row_t const* e = nisp_engine_row(body_part->engine);
            groups[groups_n].thrust_kn = e->thrust_kn * (float)body_part->engine_count;
            groups[groups_n].isp_s = e->isp_s;
            groups[groups_n].prop_t = body_part->prop_t;
            groups_n++;
        }
        if (booster_part != NULL) {
            nisp_engine_row_t const* e = nisp_engine_row(booster_part->engine);
            groups[groups_n].thrust_kn = e->thrust_kn * (float)booster_part->engine_count;
            groups[groups_n].isp_s = e->isp_s;
            groups[groups_n].prop_t = booster_part->prop_t;
            groups[groups_n].dry_t = booster_part->dry_t;
            groups_n++;
        }

        float thrust = 0.0f, burn = 0.0f, isp = 0.0f;
        b->delta_v_ms = group_delta_v(groups, groups_n, m0, &burn, &thrust, &isp);
        b->burn_s = burn;
        b->thrust_kn = thrust;
        b->isp_s = isp;
        b->twr = (m0 > 0.0f && g0 > 0.0f) ? thrust / (m0 * g0) : 0.0f;
        b->liftable = b->twr >= 1.0f;
        b->wet_t = body_part->mass_t + (booster_part != NULL ? booster_part->mass_t : 0.0f);
        b->dry_t = body_part->dry_t + (booster_part != NULL ? booster_part->dry_t : 0.0f);
        b->prop_t = body_part->prop_t + (booster_part != NULL ? booster_part->prop_t : 0.0f);

        out->total_delta_v_ms += b->delta_v_ms;
    }

    int const bottom = nisp_stack_live_stage(s);
    if (bottom >= 0) {
        out->liftoff_twr = out->stage[bottom].twr;
        out->first_stage_ok = out->stage[bottom].liftable;
    }
    out->reach_orbit_ok = out->total_delta_v_ms >= NISP_REACH_ORBIT_MS;
}

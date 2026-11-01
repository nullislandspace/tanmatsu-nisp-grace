// =====================================================================
//  Null Island Space Program  --  the stack and its sums (see stack.h)
// =====================================================================

#include "rocket/stack.h"

#include <string.h>

#include "sim/units.h"

static int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

// --- Geometry ----------------------------------------------------------

static void add_part(nisp_stack_t* s, nisp_part_t const* p) {
    if (s->count >= NISP_STACK_PART_MAX) return;
    s->part[s->count] = *p;
    s->count++;
}

static void finish_body(nisp_part_t* p, nisp_part_row_t const* row) {
    p->area_m2 = nisp_side_area_m2(p->diameter_m, p->x1 - p->x0);
    p->cd_a = row->cd * p->area_m2;
    p->capacity = row->capacity_per_t * (p->dry_t + p->prop_t);
    p->limit = row->limit;
    p->shield = row->shield;
    p->heat = 0.0f;
    p->com_bias = 0.5f;
}

void nisp_stage_masses(nisp_stage_t const* st, nisp_stage_mass_t* out) {
    if (st == NULL || out == NULL) return;
    memset(out, 0, sizeof(*out));

    nisp_part_row_t const* row = nisp_part_row(st->kind);
    nisp_engine_row_t const* eng = nisp_engine_row(st->engine);
    float const dia = nisp_diameter_m(st->diameter);
    float const len = nisp_length_m(st->length);

    out->prop_t = nisp_propellant_mass_t(dia, len);
    out->dry_t = row->dry_fraction * out->prop_t;
    if (!eng->solid) out->dry_t += eng->mass_t * (float)st->engine_count;
    if (st->rcs) out->dry_t += NISP_RCS_MASS_T;
    out->dry_t += nisp_legs_row(st->legs)->mass_t;
    out->separator_t = NISP_SEPARATOR_MASS_T;

    if (st->boosters > 0) {
        nisp_part_row_t const* borow = nisp_part_row(NISP_KIND_BOOSTER);
        float const bprop = nisp_propellant_mass_t(dia, nisp_length_m(st->booster_length));
        out->boosters_prop_t = bprop * (float)st->boosters;
        out->boosters_dry_t = borow->dry_fraction * bprop * (float)st->boosters;
    }
}

void nisp_stack_build(nisp_design_t const* d, nisp_stack_t* out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (d == NULL || d->stage_count < 1) return;

    out->stage_count = d->stage_count;
    out->pod_rcs = d->pod_rcs;
    out->chutes = clamp_int(d->pod_chutes, 1, NISP_CHUTE_MAX);
    out->live_stage = d->stage_count - 1;  // the bottom stage burns first (D-42)
    out->throttle = 0.0f;

    float x = 0.0f;
    float const top_d = nisp_diameter_m(d->stage[0].diameter);

    // The capsule: always exactly one, always on top, never removable.
    nisp_part_t pod = {0};
    pod.slot = NISP_PART_KIND_BODY;
    pod.kind = NISP_KIND_CAPSULE;
    pod.stage = -1;
    pod.engine = NISP_ENGINE_NONE;
    pod.engine_count = 0;
    pod.x0 = x;
    pod.x1 = x + NISP_CAPSULE_LENGTH_M;
    pod.diameter_m = top_d;
    // The pod and the two things on it the player can choose (D-51). One
    // canopy is already inside the row's 1.0 t; the extras are bolted on.
    pod.dry_t = nisp_pod_mass_t(d->pod_rcs, d->pod_chutes);
    pod.prop_t = 0.0f;
    pod.prop_full_t = 0.0f;
    pod.mass_t = pod.dry_t;
    finish_body(&pod, nisp_part_row(NISP_KIND_CAPSULE));
    pod.com_bias = NISP_CAPSULE_MASS_BIAS;
    add_part(out, &pod);
    x = pod.x1;

    for (int i = 0; i < d->stage_count; i++) {
        nisp_stage_t const* st = &d->stage[i];
        nisp_part_row_t const* row = nisp_part_row(st->kind);
        float const dia = nisp_diameter_m(st->diameter);
        float const len = nisp_length_m(st->length);

        nisp_stage_mass_t mass;
        nisp_stage_masses(st, &mass);
        out->rcs[i] = st->rcs;

        // The separator above this stage (D-21). It goes with the stage.
        nisp_part_t sep = {0};
        sep.slot = NISP_PART_KIND_SEPARATOR;
        sep.kind = st->kind;
        sep.stage = i;
        sep.x0 = x;
        sep.x1 = x + NISP_SEPARATOR_SPAN_M;
        sep.diameter_m = dia;
        sep.dry_t = mass.separator_t;
        sep.mass_t = sep.dry_t;
        finish_body(&sep, row);
        sep.cd_a = 0.3f * sep.area_m2;  // a ring, not a cylinder
        // A decoupler is a few kilograms of steel: mass-scaled thermal
        // capacity would make every decoupler the first thing to burn on
        // every flight, which is not a thing this game models (D-20).
        sep.capacity = 0.0f;
        add_part(out, &sep);
        x = sep.x1;

        // The stage itself: a cylinder of propellant with engines on the bottom.
        float const prop = mass.prop_t;
        float const dry = mass.dry_t;

        nisp_part_t body = {0};
        body.slot = NISP_PART_KIND_BODY;
        body.kind = st->kind;
        body.stage = i;
        body.engine = row->engines_allowed ? st->engine : NISP_ENGINE_NONE;
        body.engine_count = body.engine == NISP_ENGINE_NONE ? 0 : st->engine_count;
        body.x0 = x;
        body.x1 = x + len;
        body.diameter_m = dia;
        body.prop_full_t = prop;
        body.prop_t = prop;
        body.dry_t = dry;
        body.mass_t = dry + prop;
        finish_body(&body, row);
        add_part(out, &body);

        // Side boosters: one group, because they are identical, ignite with
        // the stage, and leave together when they are empty (Part V1).
        if (st->boosters > 0) {
            nisp_part_row_t const* borow = nisp_part_row(NISP_KIND_BOOSTER);
            float const bprop = mass.boosters_prop_t / (float)st->boosters;
            float const bdry = mass.boosters_dry_t / (float)st->boosters;
            int const n = st->boosters;

            nisp_part_t bo = {0};
            bo.slot = NISP_PART_KIND_BOOSTER;
            bo.kind = NISP_KIND_BOOSTER;
            bo.stage = i;
            bo.engine = NISP_ENGINE_BOOSTER;
            bo.engine_count = n;
            bo.x0 = body.x0 + 0.25f;
            bo.x1 = body.x1 - 0.25f;
            if (bo.x1 <= bo.x0) {
                bo.x0 = body.x0;
                bo.x1 = body.x1;
            }
            bo.diameter_m = dia;
            bo.prop_full_t = bprop * (float)n;
            bo.prop_t = bo.prop_full_t;
            bo.dry_t = bdry * (float)n;
            bo.mass_t = bo.dry_t + bo.prop_t;
            finish_body(&bo, borow);
            // One row stands for n identical boosters, so its skin is n
            // cylinders. Its mass is already n times one booster's, and
            // thermal capacity goes with mass, so area has to follow or n
            // boosters would be n times harder to heat than one.
            bo.area_m2 *= (float)n;
            bo.cd_a = borow->cd * bo.area_m2;
            add_part(out, &bo);
        }

        // Legs: mass is already in the stage; deployed legs are drag at the
        // tail, so they are a massless row that only ever moves the drag centre.
        if (st->legs != NISP_LEGS_NONE) {
            nisp_part_t lg = {0};
            lg.slot = NISP_PART_KIND_LEGS;
            lg.kind = st->kind;
            lg.stage = i;
            lg.x0 = body.x1 - 0.1f;
            lg.x1 = body.x1 + 0.4f;
            lg.diameter_m = dia;
            lg.capacity = 0.0f;  // no heat on a drag plate
            lg.limit = 1.0f;
            lg.cd_a = 0.0f;  // stowed
            add_part(out, &lg);
            out->legs_deployed[i] = false;
            out->legs_id[i] = st->legs;
        }

        x = body.x1;
    }

    nisp_stack_recompute(out);
}

// --- The sums ----------------------------------------------------------

float nisp_part_fill(nisp_part_t const* p) {
    if (p->prop_full_t <= 0.0f) return 0.0f;
    return clamp01(p->prop_t / p->prop_full_t);
}

float nisp_part_com_x(nisp_part_t const* p) {
    float const len = p->x1 - p->x0;
    float const bias = p->com_bias > 0.0f ? p->com_bias : 0.5f;

    // Mass biased toward the shield end, which is what makes a lone
    // capsule weathervane shield-first (D-31).
    float const dry_x = p->x0 + bias * len;
    if (p->prop_t <= 0.0f || p->prop_full_t <= 0.0f) return dry_x;

    // The tank empties from its bottom: what is left is the top `fill`
    // fraction, so its centre walks toward the nose as it burns.
    float const fill = nisp_part_fill(p);
    float const prop_x = p->x0 + 0.5f * fill * len;

    float const m = p->dry_t + p->prop_t;
    if (m <= 0.0f) return dry_x;
    return (p->dry_t * dry_x + p->prop_t * prop_x) / m;
}

int nisp_stack_live_stage(nisp_stack_t const* s) {
    int best = -1;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        if (p->slot != NISP_PART_KIND_BODY) continue;
        if (p->kind == NISP_KIND_CAPSULE) continue;
        if (p->stage > best) best = p->stage;
    }
    return best;
}

int nisp_part_stage(nisp_stack_t const* s, int index) {
    if (s == NULL || index < 0 || index >= s->count) return -1;
    return s->part[index].stage;
}

void nisp_stack_recompute(nisp_stack_t* s) {
    if (s == NULL) return;

    float mass = 0.0f, prop = 0.0f, com = 0.0f;
    float thrust = 0.0f, cot = 0.0f, isp = 0.0f;
    float cd_a = 0.0f, cp = 0.0f;
    float cd_a_x2 = 0.0f;  // sum of cd_a * x^2, for the damping sum below
    bool capsule = false;

    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        float const x = nisp_part_com_x(p);

        mass += p->mass_t;
        prop += p->prop_t;
        com += p->mass_t * x;
        if (p->kind == NISP_KIND_CAPSULE) capsule = true;

        float const xc = 0.5f * (p->x0 + p->x1);
        cd_a += p->cd_a;
        cp += p->cd_a * xc;
        cd_a_x2 += p->cd_a * xc * xc;

        if (p->kind == NISP_KIND_CAPSULE && s->parachute) {
            // The canopy streams off the capsule's aft end, so its drag
            // centre is aft of the body it pulls on (Part A3). More canopies
            // is the same area, more times (D-51).
            float const chute_cd_a = nisp_chute_cd_a(s->chutes);
            cd_a += chute_cd_a;
            cp += chute_cd_a * p->x1;
            cd_a_x2 += chute_cd_a * p->x1 * p->x1;
        }

        if (p->igniting && p->engine != NISP_ENGINE_NONE) {
            nisp_engine_row_t const* e = nisp_engine_row(p->engine);
            float const t = e->thrust_kn * (float)p->engine_count;
            thrust += t;
            cot += t * p->x1;  // engines are bolted to the bottom of their stage
            isp += t * e->isp_s;
        }
    }

    s->mass_t = mass;
    s->prop_t = prop;
    s->com_x = mass > 0.0f ? com / mass : 0.0f;
    s->cd_a_total = cd_a;
    s->cp_x = cd_a > 0.0f ? cp / cd_a : s->com_x;
    s->static_margin_m = s->cp_x - s->com_x;
    // Sum cd_a*(x - com)^2 = sum cd_a*x^2 - CdA * com^2, which is the same
    // number without a second pass over the parts.
    s->aero_damp_m4 = cd_a_x2 - cd_a * s->com_x * s->com_x;
    if (s->aero_damp_m4 < 0.0f) s->aero_damp_m4 = 0.0f;
    s->thrust_kn = thrust * clamp01(s->throttle);
    s->cot_x = thrust > 0.0f ? cot / thrust : s->com_x;
    s->isp_s = thrust > 0.0f ? isp / thrust : 0.0f;
    s->has_capsule = capsule;
    if (!capsule) s->pod_rcs = false;  // no pod, no pod RCS
    s->live_stage = nisp_stack_live_stage(s);

    // Transverse moment of inertia: solid cylinders about their own centre,
    // then walked out to the stack's CoM.
    float inertia = 0.0f;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        if (p->mass_t <= 0.0f) continue;
        float const r = 0.5f * p->diameter_m;
        float const len = p->x1 - p->x0;
        float const own = p->mass_t * (3.0f * r * r + len * len) / 12.0f;
        float const d = nisp_part_com_x(p) - s->com_x;
        inertia += own + p->mass_t * d * d;
    }
    s->inertia_t_m2 = inertia;

    // About the stack axis: every part is a solid cylinder of its own
    // diameter, and nothing in this model is offset laterally from the
    // axis. Roll control (Part F2) fights this number, not the transverse
    // one, and it is one to two orders of magnitude smaller.
    float axial = 0.0f;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        float const r = 0.5f * p->diameter_m;
        axial += 0.5f * p->mass_t * r * r;
    }
    s->inertia_axial_t_m2 = axial;
}

void nisp_stack_deploy_chute(nisp_stack_t* s, bool deployed) {
    if (s == NULL || !s->has_capsule) return;
    s->parachute = deployed;
    nisp_stack_recompute(s);
}

// --- Burning -----------------------------------------------------------

void nisp_stack_ignite(nisp_stack_t* s, int stage, float throttle) {
    if (s == NULL) return;
    s->throttle = clamp01(throttle);
    for (int i = 0; i < s->count; i++) {
        nisp_part_t* p = &s->part[i];
        p->igniting = (p->stage == stage) && (p->engine != NISP_ENGINE_NONE);
    }
    nisp_stack_recompute(s);
}

void nisp_stack_cut(nisp_stack_t* s) {
    if (s == NULL) return;
    s->throttle = 0.0f;
    nisp_stack_recompute(s);
}

// Tonnes per second at full throttle for one part's engines.
static float part_mass_flow_t(nisp_part_t const* p) {
    if (p->engine == NISP_ENGINE_NONE) return 0.0f;
    nisp_engine_row_t const* e = nisp_engine_row(p->engine);
    if (e->isp_s <= 0.0f) return 0.0f;
    return e->thrust_kn * (float)p->engine_count / (e->isp_s * (float)NISP_G0);
}

bool nisp_stack_burn(nisp_stack_t* s, float dt_s) {
    if (s == NULL || dt_s <= 0.0f) return false;

    float const throttle = clamp01(s->throttle);
    if (throttle <= 0.0f) return false;

    bool booster_left = false;

    for (int i = 0; i < s->count; i++) {
        nisp_part_t* p = &s->part[i];
        if (!p->igniting || p->prop_t <= 0.0f) continue;

        float const flow = part_mass_flow_t(p) * throttle * dt_s;
        if (flow <= 0.0f) continue;

        if (flow >= p->prop_t) {
            p->mass_t -= p->prop_t;
            p->prop_t = 0.0f;
            if (p->mass_t < 0.0f) p->mass_t = 0.0f;
            if (p->slot == NISP_PART_KIND_BOOSTER) {
                // Empty boosters leave on their own burn-out, not on the
                // stage boundary (Part F3).
                int const stage = p->stage;
                for (int j = s->count - 1; j >= 0; j--) {
                    if (s->part[j].slot == NISP_PART_KIND_BOOSTER && s->part[j].stage == stage) {
                        for (int k = j; k < s->count - 1; k++) s->part[k] = s->part[k + 1];
                        s->count--;
                        booster_left = true;
                    }
                }
                i--;  // the array shifted under the cursor
            }
        } else {
            p->prop_t -= flow;
            p->mass_t -= flow;
        }
    }

    nisp_stack_recompute(s);
    return booster_left;
}

// --- Staging and splitting ---------------------------------------------

void nisp_stack_set_fill(nisp_stack_t* s, int stage, float fill) {
    if (s == NULL || stage < 0) return;
    if (fill < 0.0f) fill = 0.0f;
    if (fill > 1.0f) fill = 1.0f;
    // The same rule nisp_stack_burn uses, applied at once: a stage's tank and
    // its booster group empty together, so one fraction means both. Restoring
    // a craft that was left in orbit is this -- the flight already spent the
    // propellant, and the save says how much of it is left.
    for (int i = 0; i < s->count; i++) {
        nisp_part_t* p = &s->part[i];
        if (p->stage != stage) continue;
        p->prop_t = fill * p->prop_full_t;
    }
    nisp_stack_recompute(s);
}

int nisp_stack_drop_stage(nisp_stack_t* s, int stage) {
    if (s == NULL || stage < 0) return 0;
    int removed = 0;
    for (int i = s->count - 1; i >= 0; i--) {
        if (s->part[i].stage != stage) continue;
        for (int k = i; k < s->count - 1; k++) s->part[k] = s->part[k + 1];
        s->count--;
        removed++;
    }
    if (stage < NISP_DESIGN_STAGE_MAX) {
        s->legs_deployed[stage] = false;
        s->legs_id[stage] = NISP_LEGS_NONE;
    }
    nisp_stack_recompute(s);
    return removed;
}

int nisp_stack_advance(nisp_stack_t* s) {
    if (s == NULL) return 0;
    int const stage = nisp_stack_live_stage(s);
    if (stage < 0) return 0;
    s->throttle = 0.0f;
    int const removed = nisp_stack_drop_stage(s, stage);
    // The stage above is armed but not lit: the player ignites it.
    int const next = nisp_stack_live_stage(s);
    if (next >= 0) nisp_stack_ignite(s, next, 0.0f);
    return removed;
}

bool nisp_stack_split(nisp_stack_t* s, int index, nisp_stack_t* other) {
    if (s == NULL || index < 0 || index >= s->count) return false;
    if (s->part[index].kind == NISP_KIND_CAPSULE) return false;  // the pod is the mission
    if (index == 0) return false;  // nothing above the break to keep

    // Rows are kept nose-first, so the fragment that keeps control is
    // already the first `index` rows of `s`: cutting the stack is a shorter
    // count, not a copy. Only the tail has to move, and it moves straight
    // into the caller's `other`. Two whole `nisp_stack_t`s on the stack here
    // was the single biggest frame in the app (F-45).
    bool pod = false;
    for (int i = 0; i < index; i++) {
        if (s->part[i].kind == NISP_KIND_CAPSULE) pod = true;
    }
    if (!pod) return false;  // should not happen; do not lose the mission

    if (other != NULL) {
        nisp_stack_t* lower = other;
        memset(lower, 0, sizeof(*lower));
        lower->stage_count = s->stage_count;
        lower->pod_rcs = s->pod_rcs;
        lower->chutes = s->chutes;
        lower->throttle = 0.0f;  // debris does not fire its engines
        for (int i = 0; i < NISP_DESIGN_STAGE_MAX; i++) {
            lower->legs_deployed[i] = s->legs_deployed[i];
        }
        for (int i = index + 1; i < s->count && lower->count < NISP_STACK_PART_MAX; i++) {
            lower->part[lower->count++] = s->part[i];
        }
        nisp_stack_recompute(lower);
        lower->live_stage = -1;  // debris: nothing to light
        for (int i = 0; i < lower->count; i++) lower->part[i].igniting = false;
    }

    s->count = index;
    s->throttle = 0.0f;  // a separation goes cold, the same rule staging uses
    nisp_stack_recompute(s);
    return true;
}

// --- Legs, heat readouts -----------------------------------------------

void nisp_stack_deploy_legs(nisp_stack_t* s, int stage, bool deployed) {
    if (s == NULL || stage < 0 || stage >= NISP_DESIGN_STAGE_MAX) return;
    s->legs_deployed[stage] = deployed;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t* p = &s->part[i];
        if (p->slot != NISP_PART_KIND_LEGS || p->stage != stage) continue;
        p->cd_a = deployed ? nisp_legs_row(s->legs_id[stage])->cd_a : 0.0f;
    }
    nisp_stack_recompute(s);
}

bool nisp_stack_legs_deployed(nisp_stack_t const* s, int stage) {
    if (s == NULL || stage < 0 || stage >= NISP_DESIGN_STAGE_MAX) return false;
    return s->legs_deployed[stage];
}

int nisp_stack_hottest(nisp_stack_t const* s) {
    if (s == NULL) return -1;
    int best = -1;
    float best_load = 0.0f;
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].capacity <= 0.0f) continue;
        float const load = s->part[i].heat / (s->part[i].limit > 0.0f ? s->part[i].limit : 1.0f);
        if (best < 0 || load > best_load) {
            best_load = load;
            best = i;
        }
    }
    return best;
}

float nisp_stack_heat_load(nisp_stack_t const* s) {
    int const i = nisp_stack_hottest(s);
    if (i < 0) return 0.0f;
    float const limit = s->part[i].limit > 0.0f ? s->part[i].limit : 1.0f;
    return s->part[i].heat / limit;
}

// --- What a player asks about the stack -------------------------------

// How far down the tail goes, metres. The stack axis is measured from the
// nose, so the longest x1 is the length of the whole vehicle. game/scene.c
// needs it to stand a camera off far enough to see all of it, and the HUD
// needs it for nothing -- which is why it lives here rather than being
// recomputed in two places (F-41).
float nisp_stack_length_m(nisp_stack_t const* s) {
    if (s == NULL) return 0.0f;
    float x = 0.0f;
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].x1 > x) x = s->part[i].x1;
    }
    return x;
}

// How many engines a stage brings: its own motor(s) plus its side
// boosters' motors. Zero means the stage key has nothing to light, which is
// the difference between "SPACE ignites this stage" and "SPACE is refused".
int nisp_stack_stage_engines(nisp_stack_t const* s, int stage) {
    if (s == NULL || stage < 0) return 0;
    int n = 0;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        if (p->stage != stage) continue;
        if (p->engine == NISP_ENGINE_NONE || p->engine_count <= 0) continue;
        n += p->engine_count;
    }
    return n;
}

// How much propellant a stage holds and how much is left. `stage` -1 is
// every part still attached, which is the number a player reads as "how
// much rocket is left".
void nisp_stack_propellant(nisp_stack_t const* s, int stage, float* left_t, float* full_t) {
    float left = 0.0f, full = 0.0f;
    if (s != NULL) {
        for (int i = 0; i < s->count; i++) {
            nisp_part_t const* p = &s->part[i];
            if (stage >= 0 && p->stage != stage) continue;
            left += p->prop_t;
            full += p->prop_full_t;
        }
    }
    if (left_t) *left_t = left;
    if (full_t) *full_t = full;
}

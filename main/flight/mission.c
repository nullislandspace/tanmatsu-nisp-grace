// =====================================================================
//  Null Island Space Program  --  a flight, start to finish (see mission.h)
// =====================================================================

#include "flight/mission.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "rocket/heat.h"
#include "rocket/parts.h"
#include "sim/atmosphere.h"
#include "sim/surface.h"
#include "sim/units.h"

// Debris ticks per frame (Part G). The ship gets NISP_SUBSTEP_MAX because
// its trajectory is the game; a spent booster is flavour, and it is allowed
// a coarser tick so that one falling fragment can never hold the whole
// mission's warp down. 24 is enough to cover a warp-10 frame in air exactly
// (0.167 s / 24 = 7 ms < dt_phys) and to bound the worst case at warp 10,000
// to 24 ticks of 7 simulated seconds.
#define NISP_DEBRIS_SUBSTEP_MAX 24

// The frame dt the game runs at on the badge, and the largest one this will
// accept. A frame that took half a second to draw is a frame the physics
// should not try to repay in full.
#define NISP_FRAME_DT (1.0 / 60.0)
#define NISP_FRAME_DT_MAX 0.1

static double clamp_dt(double dt) {
    if (!(dt > 0.0)) return NISP_FRAME_DT;
    if (dt > NISP_FRAME_DT_MAX) return NISP_FRAME_DT_MAX;
    return dt;
}

// --- Advancing one body over one frame's simulated time ---------------

// Can this body be propagated analytically for this frame? No engine, no
// control torque, not landed, and above the air: the same tests flight/warp
// uses for the ship, applied to whatever body this is.
//
// The control test is the one that is easy to leave out (F-31): a coast is
// analytic, and an analytic propagation applies no forces, so a SAS that is
// fighting a rotation has to be paid for in ticks or it simply does not
// happen. Debris is never controlled, so it coasts whenever it is free.
static bool can_coast(nisp_vessel_t const* v, nisp_control_t const* ctl) {
    if (v->landed || !v->alive) return false;
    if (v->stack.thrust_kn > 0.0f) return false;
    if (nisp_control_active(&v->stack, ctl)) return false;
    float const alt = nisp_altitude(v->body, v->pos_km);
    if (alt <= 0.0f) return false;
    return nisp_atm_density(v->body, alt) <= 0.0f;
}

// One tick, and whatever that tick decided. `other` is where a fragment goes
// (NULL for debris, which is allowed to fail once and is then retired).
static nisp_hit_t tick(nisp_vessel_t* v, nisp_control_t const* ctl, double t, double dt,
                       nisp_vessel_t* other) {
    nisp_vessel_substep(v, ctl, t, dt);
    return nisp_vessel_resolve(v, t + dt, other);
}

// Cover `dt_sim` for one body: coast while it is free, tick while it is not,
// and resolve after every single tick -- heat failure and touchdown are not
// things a frame is allowed to skip over (Part F5).
// Returns the first thing it decided, if anything.
static nisp_hit_t advance_body(nisp_vessel_t* v, nisp_control_t const* ctl, double t, double dt_sim,
                               int max_ticks, nisp_vessel_t* other) {
    if (v == NULL || !v->alive || dt_sim <= 0.0) return NISP_HIT_NONE;

    nisp_hit_t hit = NISP_HIT_NONE;

    if (can_coast(v, ctl)) {
        double const used = nisp_vessel_coast(v, t, dt_sim);
        hit = nisp_vessel_resolve(v, t + used, other);
        if (hit != NISP_HIT_NONE) return hit;

        // A coast hands over to the ticks the moment it reaches the air or
        // the ground; the rest of the frame is spent the slow way, stretched
        // to the same tick budget the powered branch gets. Stretching is
        // what keeps the frame's simulated time whole: a frame that only got
        // 4 of its 167 seconds simulated would leave the mission clock and
        // the ship's clock disagreeing, and that is a wrong trajectory, not
        // a slow one.
        double const left = dt_sim - used;
        double spent = used;
        if (left > 1e-9) {
            int ticks = (int)ceil(left / NISP_PHYS_DT);
            if (ticks > max_ticks) ticks = max_ticks;
            if (ticks < 1) ticks = 1;
            double const dt = left / (double)ticks;
            for (int i = 0; i < ticks; i++) {
                hit = tick(v, ctl, t + spent, dt, other);
                spent += dt;
                if (hit != NISP_HIT_NONE) return hit;
            }
        }
        return NISP_HIT_NONE;
    }

    // Ticking. The tick size is stretched to fit the frame's tick budget,
    // which is what keeps a warp-10,000 frame from needing 8,000 ticks.
    int ticks = (int)ceil(dt_sim / NISP_PHYS_DT);
    if (ticks > max_ticks) ticks = max_ticks;
    if (ticks < 1) ticks = 1;
    double const dt = dt_sim / (double)ticks;

    for (int i = 0; i < ticks; i++) {
        hit = tick(v, ctl, t + (double)i * dt, dt, other);
        if (hit != NISP_HIT_NONE) return hit;
    }
    return NISP_HIT_NONE;
}

// --- The mission --------------------------------------------------------

void nisp_mission_init(nisp_mission_t* m) {
    if (m == NULL) return;
    memset(m, 0, sizeof(*m));
    m->state = NISP_MISSION_FLYING;
    m->ship.alive = false;
    m->ship.wreck_kind = -1;
    m->ship.wreck_stage = -1;
    m->last_warp.warp = 1.0f;
}

void nisp_mission_launch(nisp_mission_t* m, nisp_design_t const* d, int body, double t) {
    if (m == NULL) return;

    nisp_mission_init(m);
    nisp_vessel_pad(&m->ship, d, body, t);
    if (d != NULL) snprintf(m->vessel_name, sizeof(m->vessel_name), "%s", d->name);
    m->t        = t;
    m->t_launch = t;
    m->state = NISP_MISSION_FLYING;
    m->music_space = false;
    m->ignited  = false;  // the pad is not a flight yet (D-56)
    m->throttle = 0.0f;

    // The pad is not a flight yet: nothing is burning, and the chute and
    // the legs are stowed. nisp_vessel_pad did the stack; this is the log.
    nisp_readout_t r;
    nisp_vessel_readout(&m->ship, m->t, &r);
    m->max_ap_km = 0.0f;
    m->max_speed_ms = 0.0f;
    m->max_heat = r.heat_max;
}

void nisp_mission_begin(nisp_mission_t* m, nisp_vessel_t const* v, double t) {
    if (m == NULL || v == NULL) return;
    nisp_mission_init(m);
    nisp_vessel_copy(v, &m->ship);
    m->t        = t;
    m->t_launch = t;
    m->state = NISP_MISSION_FLYING;
    // A ship that is already flying is a mission whose engines were lit
    // whenever it got there: the stage key separates here, as it always did.
    m->ignited  = true;
    m->throttle = 0.0f;

    nisp_readout_t r;
    nisp_vessel_readout(&m->ship, m->t, &r);
    m->max_ap_km = r.apoapsis_km > 0.0f ? r.apoapsis_km : r.alt_km;
    m->max_speed_ms = r.speed_ms;
    m->max_heat = r.heat_max;
    m->music_space = r.alt_km >= NISP_MUSIC_SPACE_KM && !r.in_air;
}

// A command that will not be obeyed, said out loud (plan 6.5). The reason is
// the useful half: "you pressed C and nothing happened" is a bug report,
// "you pressed C at 900 m/s" is a lesson.
static void refuse(nisp_mission_t* m, char const* why) {
    snprintf(m->last_refusal, sizeof(m->last_refusal), "%s", why);
}

// The player's one-shot commands, consumed here so that a frame is never
// double-commanded (mission.h step 1).
static void apply_commands(nisp_mission_t* m, nisp_control_t* ctl, nisp_readout_t const* r) {
    nisp_vessel_t* v = &m->ship;

    if (ctl->stage) {
        ctl->stage = false;
        int const armed = v->stack.live_stage;

        if (armed < 0) {
            refuse(m, "Nothing left to stage");
        } else if (!m->ignited) {
            // Cold: this press lights the armed stage rather than dropping
            // it (D-56). A stage with no motor on it is a cargo stage, and
            // dropping it is not what the key was asked to do.
            int const engines = nisp_stack_stage_engines(&v->stack, armed);
            if (engines <= 0) {
                refuse(m, "Stage has no engine to light");
            } else {
                m->ignited = true;
            }
        } else {
            int const before = v->stack.count;
            nisp_vessel_stage(v);
            if (v->stack.count < before) m->stages_dropped++;
        }
    }

    if (ctl->chute) {
        ctl->chute = false;
        // A canopy shredded at Mach 2 is not a mechanic, it is a lesson
        // (Part A4): the chute only opens once the air is thick enough to
        // be useful and slow enough not to tear it.
        // ... and only below the speed a canopy survives opening at, which
        // is also the one speed at which its drag is integrable (F-29, F-30).
        if (v->stack.parachute) {
            // Already open: nothing to do, and nothing to explain.
        } else if (!r->in_air) {
            refuse(m, "Chute: there is no air here");
        } else if (r->q_dyn_pa < NISP_CHUTE_MIN_Q) {
            refuse(m, "Chute: the air is too thin to hold it");
        } else if (r->v_rel_ms > NISP_CHUTE_MAX_V_MS) {
            refuse(m, "Too fast for the chute");
        } else {
            nisp_vessel_chute(v, true);
        }
    }

    if (ctl->legs) {
        ctl->legs = false;
        nisp_vessel_legs(v);
    }

    if (ctl->cut) {
        ctl->cut = false;
        nisp_stack_cut(&v->stack);
    }
}

// Part N4, with the hysteresis band: space above 70 km, back to silence
// below 60, and never while in the air or sitting on the ground.
static void update_music(nisp_mission_t* m, nisp_readout_t const* r) {
    bool space = m->music_space;

    if (m->state != NISP_MISSION_FLYING) {
        space = false;
    } else if (!space) {
        if (!r->in_air && !m->ship.landed && r->alt_km >= NISP_MUSIC_SPACE_KM) space = true;
    } else {
        if (r->in_air || m->ship.landed || r->alt_km < NISP_MUSIC_AIR_KM) space = false;
    }

    m->music_space = space;
}

static void update_log(nisp_mission_t* m, nisp_readout_t const* r) {
    if (r->alt_km > m->max_ap_km) m->max_ap_km = r->alt_km;
    if (r->apoapsis_km > m->max_ap_km) m->max_ap_km = r->apoapsis_km;
    if (r->speed_ms > m->max_speed_ms) m->max_speed_ms = r->speed_ms;
    if (r->heat_max > m->max_heat) m->max_heat = r->heat_max;
}

// What a decision about the ship means for the mission.
static void apply_hit(nisp_mission_t* m, nisp_hit_t hit, nisp_vessel_t* fragment) {
    switch (hit) {
        case NISP_HIT_NONE:
            return;
        case NISP_HIT_SPLIT: {
            m->parts_lost++;
            if (fragment != NULL && fragment->alive && m->debris_count < NISP_DEBRIS_MAX) {
                m->debris[m->debris_count++] = *fragment;
            } else if (fragment != NULL && fragment->alive) {
                // The cap binds: the oldest fragment goes, because it is the
                // one furthest behind the action (Part G).
                for (int i = 1; i < NISP_DEBRIS_MAX; i++) m->debris[i - 1] = m->debris[i];
                m->debris[NISP_DEBRIS_MAX - 1] = *fragment;
            }
            return;
        }
        case NISP_HIT_LANDED:
            m->state = NISP_MISSION_LANDED;
            return;
        case NISP_HIT_CRASHED:
            m->parts_lost += m->ship.stack.count;
            m->state = NISP_MISSION_WRECKED;
            return;
        case NISP_HIT_LOST:
            m->parts_lost += m->ship.stack.count;
            m->state = NISP_MISSION_WRECKED;
            return;
    }
}

void nisp_mission_step(nisp_mission_t* m, nisp_control_t* ctl, double frame_dt) {
    if (m == NULL) return;
    if (!m->ship.alive) {
        // A wrecked mission still has debris to finish, and nothing else.
        if (ctl != NULL) {
            ctl->stage = ctl->chute = ctl->legs = ctl->cut = false;
        }
        return;
    }

    double const dt_frame = clamp_dt(frame_dt);

    // A refusal belongs to the frame it happened on: the key was pressed
    // once, and the reason is on screen for that frame (mission.h).
    m->last_refusal[0] = '\0';

    nisp_control_t const idle = {0};
    nisp_control_t const* c = ctl != NULL ? ctl : &idle;

    // The switches are state the player left set, so the mission keeps a
    // copy for whatever is drawing the controls (mission.h).
    m->sas = c->sas;
    m->rcs = c->rcs;

    // 1. the HUD's numbers, computed once for the frame (physics.h)
    nisp_readout_t r;
    nisp_vessel_readout(&m->ship, m->t, &r);
    apply_commands(m, ctl, &r);

    // 2. the throttle onto the stage that is armed. Re-arming every frame is
    // what makes the throttle key a throttle rather than an ignition switch,
    // and it is what re-lights a stage after a coast or a burn-through. Cold
    // engines are the one case where the dial is not obeyed (D-56).
    m->throttle = c->throttle;
    nisp_stack_ignite(&m->ship.stack, m->ship.stack.live_stage, m->ignited ? c->throttle : 0.0f);

    // 3. what this frame is allowed to simulate. Steering counts as powered
    // for the same reason burning does: neither can be coasted through.
    bool const thrusting = m->ship.stack.thrust_kn > 0.0f;
    bool const steering = nisp_control_active(&m->ship.stack, c);
    nisp_warp_plan(&m->ship, c->warp, dt_frame, thrusting, steering, &m->last_warp);
    double const dt_sim = m->last_warp.dt_sim;

    // 4. the ship
    nisp_vessel_t fragment;
    nisp_vessel_copy(&m->ship, &fragment);
    nisp_hit_t const hit = advance_body(&m->ship, ctl, m->t, dt_sim, NISP_SUBSTEP_MAX, &fragment);
    apply_hit(m, hit, &fragment);
    m->t += dt_sim;

    // 5. the debris: never controlled, always simulated, retired when it is
    // done (Part F5). Compacted in place so the list is always dense.
    for (int i = 0; i < m->debris_count; i++) {
        nisp_vessel_t* d = &m->debris[i];
        if (!d->alive) {
            m->debris[i] = m->debris[m->debris_count - 1];
            m->debris[--m->debris_count] = (nisp_vessel_t){0};
            i--;
            continue;
        }
        nisp_hit_t const dh = advance_body(d, NULL, m->t, dt_sim, NISP_DEBRIS_SUBSTEP_MAX, NULL);
        if (dh == NISP_HIT_SPLIT) m->parts_lost++;
        if (!d->alive || dh == NISP_HIT_CRASHED || dh == NISP_HIT_LANDED) {
            // Landed debris stays on the list: it is a thing that is there,
            // and the map draws it. Wrecked debris goes.
            if (dh == NISP_HIT_LANDED) continue;
            m->debris[i] = m->debris[m->debris_count - 1];
            m->debris[--m->debris_count] = (nisp_vessel_t){0};
            i--;
        }
    }

    // 6. the log and the music context
    nisp_vessel_readout(&m->ship, m->t, &r);
    update_log(m, &r);
    update_music(m, &r);
}

bool nisp_mission_music_space(nisp_mission_t const* m) {
    return m != NULL && m->music_space;
}

char const* nisp_mission_state_name(int state) {
    switch (state) {
        case NISP_MISSION_FLYING: return "flying";
        case NISP_MISSION_LANDED: return "landed";
        case NISP_MISSION_WRECKED: return "wrecked";
        default: return "?";
    }
}

void nisp_mission_readout(nisp_mission_t const* m, nisp_readout_t* out) {
    if (m == NULL || out == NULL) return;
    nisp_vessel_readout(&m->ship, m->t, out);
}

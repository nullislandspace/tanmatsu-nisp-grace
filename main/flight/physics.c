// =====================================================================
//  Null Island Space Program  --  one rigid body, one tick (see physics.h)
// =====================================================================

#include "flight/physics.h"

#include <math.h>
#include <stddef.h>

#include "rocket/heat.h"
#include "rocket/parts.h"
#include "sim/atmosphere.h"
#include "sim/bodies.h"
#include "sim/ephemeris.h"
#include "sim/patches.h"
#include "sim/surface.h"
#include "sim/units.h"

// The smallest inertia the Euler step will divide by. A stack with no
// parts left is not flyable, but it must not produce an infinity either.
#define INERTIA_MIN 1e-4f
#define MASS_MIN    1e-4f
#define VEC_MIN     1e-9

// The largest attitude step a coast will take. Torque-free, the body rate
// is small, but the quaternion integration is first order in dt, and a
// warp-10,000 coast is 167 s of simulated time per frame -- which has to
// be chopped up or the nose would smear through ten turns.
#define COAST_ATT_DT 0.5
#define COAST_STEPS_MAX 400

// The body-rate cap: what "tumbling" means, and the ceiling the integrator
// will not let the ship pass (F-30).
#define NISP_OMEGA_CAP 8.0f

static double mag3(double const v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static double dot3(double const a[3], double const b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross3(double const a[3], double const b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// Cap a torque vector's magnitude, the same way flight/control.c caps the
// torque an RCS is allowed to spend.
static void clamp_mag(float v[3], float max_mag) {
    float const m = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (m > max_mag && m > 0.0f) {
        float const k = max_mag / m;
        for (int i = 0; i < 3; i++) v[i] *= k;
    }
}

// The body's rotation is about +y with the period from its row -- the same
// convention sim/surface.c's local frame uses, where "east" at the pad is
// -z. A frame that turns at omega carries the air with it, which is why
// the air's velocity is `omega x r` and not zero (Part A2).
static void frame_velocity(int body, double const pos_km[3], double out_kms[3]) {
    nisp_body_t const* b = nisp_body(body);
    double const w = b->rotation_s > 0.0f ? NISP_TWO_PI / b->rotation_s : 0.0;
    out_kms[0] = w * pos_km[2];
    out_kms[1] = 0.0;
    out_kms[2] = -w * pos_km[0];
}

// --- The force model ---------------------------------------------------

typedef struct {
    double a_grav[3];    // km/s^2
    double a_drag[3];    // km/s^2
    double a_thrust[3];  // km/s^2

    double v_rel[3];     // km/s, relative to the air
    double v_rel_mag;    // km/s

    float nose[3];       // unit, frame axes: the vehicle's +z
    float alt_km;
    float rho;           // kg/m^3
    float q_dyn;         // Pa
    float drag_n;        // N

    float torque_aero[3];  // kN*m, FRAME axes (Part A5)
    double damp_coeff;     // kg*m^2/s: aerodynamic rotational damping (F-32)
    float cot_com_m;       // CoT - CoM along the axis: the thrust lever arm

    float cos_flow_axis;  // v_rel_hat . nose: +1 is the shield facing the air
    bool  in_air;
} nisp_forces_t;

// Gravity, air and engines at one point, for one attitude. Everything the
// tick needs twice (once at each half step) is derived from here.
static void compute_forces(nisp_vessel_t const* v, double dt_s, nisp_forces_t* f) {
    nisp_body_t const* b = nisp_body(v->body);
    nisp_stack_t const* s = &v->stack;

    double const r2 = dot3(v->pos_km, v->pos_km);
    double const r = sqrt(r2 > VEC_MIN ? r2 : VEC_MIN);

    // a = -mu r / |r|^3, in km/s^2 because mu is km^3/s^2 (sim/units.h).
    double const k_grav = -b->mu / (r2 * r);
    for (int i = 0; i < 3; i++) f->a_grav[i] = k_grav * v->pos_km[i];

    double air[3];
    frame_velocity(v->body, v->pos_km, air);
    for (int i = 0; i < 3; i++) f->v_rel[i] = v->vel_kms[i] - air[i];
    f->v_rel_mag = mag3(f->v_rel);

    f->alt_km = (float)(r - b->radius_km);
    f->rho = nisp_atm_density(v->body, f->alt_km);
    f->in_air = f->rho > 0.0f;

    float const v_rel_ms = (float)(f->v_rel_mag * NISP_MS_PER_KMS);
    f->q_dyn = 0.5f * f->rho * v_rel_ms * v_rel_ms;
    f->drag_n = f->q_dyn * s->cd_a_total;

    float const mass = s->mass_t > MASS_MIN ? s->mass_t : MASS_MIN;

    // Drag, N -> km/s^2. N / (t * 1000 kg) is m/s^2, and km/s^2 is a
    // thousandth of that, hence 1e6. Divided by |v_rel| because what the
    // equation wants is the unit vector, and v_rel is already its direction.
    double k_drag = f->v_rel_mag > VEC_MIN ? -(double)f->drag_n / ((double)mass * 1.0e6 * f->v_rel_mag) : 0.0;

    // The drag clamp (F-30): one tick may not take more of the relative
    // velocity than the body has to lose. Drag goes as v^2 and the tick is
    // explicit, so past that point the answer is an oscillation about zero
    // instead of a descent. It only ever binds on a chute opened far too
    // fast, which is also why flight/mission refuses that key press.
    double const k_max = 1.0 / dt_s;
    if (k_drag < -k_max) k_drag = -k_max;

    for (int i = 0; i < 3; i++) f->a_drag[i] = k_drag * f->v_rel[i];

    nisp_vessel_nose(v, f->nose);

    // Thrust, kN -> km/s^2. kN / t is m/s^2 (the 1,000s cancel), and a
    // thousandth of that is km/s^2. Along the nose, because that is which
    // way the bells point.
    double const k_thrust = (double)s->thrust_kn / ((double)mass * 1000.0);
    for (int i = 0; i < 3; i++) f->a_thrust[i] = k_thrust * (double)f->nose[i];

    // The weathervane (Part A5): tau = q * CdA * SM * (nose x v_rel_hat),
    // newton-metres, and kN*m is a thousandth of that -- which is also
    // exactly the ratio between a newton-metre and a t*m^2 of inertia, so
    // torque over inertia needs no conversion at all (physics.h).
    double vhat[3] = {0.0, 0.0, 0.0};
    if (f->v_rel_mag > VEC_MIN) {
        for (int i = 0; i < 3; i++) vhat[i] = f->v_rel[i] / f->v_rel_mag;
    }
    double lever[3], c[3];
    for (int i = 0; i < 3; i++) lever[i] = (double)f->nose[i];
    cross3(lever, vhat, c);
    double k_aero = (double)f->q_dyn * (double)s->cd_a_total * (double)s->static_margin_m * 0.001;

    // The weathervane cap (F-30). `nose x vhat` is a unit vector, so this
    // coefficient is the torque magnitude and capping it caps the torque. A
    // restoring torque is only integrable while its natural frequency fits
    // the tick; past that the ship's attitude is noise. Capped, the air can
    // turn the ship at up to NISP_TORQUE_OMEGA_MAX rad/s of natural
    // frequency, which is about a second to line up with the flow.
    double const it = s->inertia_t_m2 > INERTIA_MIN ? (double)s->inertia_t_m2 : (double)INERTIA_MIN;
    double const tau_max = it * (double)NISP_TORQUE_OMEGA_MAX * (double)NISP_TORQUE_OMEGA_MAX;
    if (k_aero > tau_max) k_aero = tau_max;
    if (k_aero < -tau_max) k_aero = -tau_max;

    for (int i = 0; i < 3; i++) f->torque_aero[i] = (float)(k_aero * c[i]);

    // Aerodynamic damping (F-32). The weathervane above is a spring with no
    // damper on it, and a spring with no damper is a pendulum: a capsule
    // that opens a chute swings through vertical, past it, and back, forever
    // -- which is what nispcheck case 13's chute drop used to do, landing at 43 degrees
    // of tilt on a perfectly good canopy.
    //
    // The damper is the same drag, on the sideways motion rotation creates.
    // A part at distance d from the CoM moves at omega * d through the air,
    // and the drag on that is 0.5 * rho * |v_rel| * cd_a * omega * d (the
    // linearisation of q*v in omega, which is the standard rotational
    // damping derivative). Times the lever arm d, summed over the parts, is
    // 0.5 * rho * |v_rel| * sum(cd_a * d^2) -- and that sum is rocket/stack's
    // aero_damp_m4. It is why a canopy at the end of a long line is stable
    // and a stubby one is not.
    f->damp_coeff = 0.5 * (double)f->rho * (f->v_rel_mag * NISP_MS_PER_KMS) * (double)s->aero_damp_m4;

    // (CoT - CoM) x F. Both points lie on the stack axis and F is along
    // it, so this torque is identically zero for a one-dimensional stack
    // (Part V4). The lever arm is carried anyway: it is the number that
    // stops being zero the day a part is mounted off-axis, and it is the
    // number the HUD can show as "why is it not turning".
    f->cot_com_m = s->cot_x - s->com_x;

    f->cos_flow_axis = (float)dot3(vhat, lever);
}

// The acceleration at a point, for the second half of a Verlet step.
// Gravity is re-derived where the ship actually is; the air density is
// re-derived too, because that is one expf and the altitude moved. The
// attitude and the frame's rotation are held across the tick: over 0.02 s
// at orbital speed the air state moves by a few tens of metres, which is
// nothing against a 5.6 km scale height (F-28).
static void accel_at(nisp_vessel_t const* v, nisp_stack_t const* s, nisp_forces_t const* f, double dt_s,
                     double const pos_km[3], double const vel_kms[3], double a_out[3]) {
    nisp_body_t const* b = nisp_body(v->body);

    double const r2 = dot3(pos_km, pos_km);
    double const r = sqrt(r2 > VEC_MIN ? r2 : VEC_MIN);
    double const k_grav = -b->mu / (r2 * r);
    for (int i = 0; i < 3; i++) a_out[i] = k_grav * pos_km[i];

    double air[3];
    frame_velocity(v->body, pos_km, air);
    double vr[3];
    for (int i = 0; i < 3; i++) vr[i] = vel_kms[i] - air[i];
    double const vmag = mag3(vr);

    float const alt = (float)(r - b->radius_km);
    float const rho = nisp_atm_density(v->body, alt);
    if (rho > 0.0f && vmag > VEC_MIN) {
        float const v_ms = (float)(vmag * NISP_MS_PER_KMS);
        float const q = 0.5f * rho * v_ms * v_ms;
        float const drag_n = q * s->cd_a_total;
        float const mass = s->mass_t > MASS_MIN ? s->mass_t : MASS_MIN;
        double k = -(double)drag_n / ((double)mass * 1.0e6 * vmag);
        double const k_max = 1.0 / dt_s;  // the same clamp as compute_forces
        if (k < -k_max) k = -k_max;
        for (int i = 0; i < 3; i++) a_out[i] += k * vr[i];
    }

    float const mass = s->mass_t > MASS_MIN ? s->mass_t : MASS_MIN;
    double const k_thrust = (double)s->thrust_kn / ((double)mass * 1000.0);
    for (int i = 0; i < 3; i++) a_out[i] += k_thrust * (double)f->nose[i];
}

// Euler's equations for a symmetric top: I1 = I2 = transverse, I3 = about
// the stack axis. The products are what make a rocket with no RCS precess
// instead of tumbling, and they are the only place the two inertias meet.
static void euler_step(nisp_stack_t const* s, float omega[3], float const tau[3], float dt) {
    float const it = s->inertia_t_m2 > INERTIA_MIN ? s->inertia_t_m2 : INERTIA_MIN;
    float const ia = s->inertia_axial_t_m2 > INERTIA_MIN ? s->inertia_axial_t_m2 : INERTIA_MIN;

    float const w1 = omega[0], w2 = omega[1], w3 = omega[2];
    omega[0] += (tau[0] + (it - ia) * w2 * w3) / it * dt;
    omega[1] += (tau[1] + (ia - it) * w3 * w1) / it * dt;
    omega[2] += tau[2] / ia * dt;
}

// --- Sitting on the ground ---------------------------------------------

// A landed ship is a point fixed to a turning sphere: its position and
// velocity come from the surface, not from an integration. That is what
// keeps a landed mission from creeping through the ground at warp 100.
static void soi_update(nisp_vessel_t* v, double t);

static void rest_on_ground(nisp_vessel_t* v, double t) {
    nisp_surface_point(v->body, t, v->lat_rad, v->lon_rad, v->pos_km);
    nisp_surface_velocity(v->body, t, v->lat_rad, v->lon_rad, v->vel_kms);
    v->omega[0] = v->omega[1] = v->omega[2] = 0.0f;
}

// --- The tick -----------------------------------------------------------

void nisp_vessel_substep(nisp_vessel_t* v, nisp_control_t const* ctl, double t, double dt_s) {
    if (v == NULL || dt_s <= 0.0) return;
    if (!v->alive) return;

    nisp_stack_t* s = &v->stack;

    if (v->landed) {
        // On the ground it stays, unless the engines out-accelerate the
        // planet. That is the launch clamp, and it is the only contact
        // force in the game: the pad holds the rocket up until it is
        // clearly lifting (Part A3).
        double const g = (double)nisp_body_g0(v->body);
        double const a = (double)s->thrust_kn / (double)(s->mass_t > MASS_MIN ? s->mass_t : MASS_MIN);
        if (!(ctl != NULL && a > g)) {
            rest_on_ground(v, t + dt_s);
            nisp_heat_input_t const cool = {v->body, 0.0f, 0.0f, 0.0f};
            nisp_heat_tick(s, &cool, dt_s);
            return;
        }
        v->landed = false;  // liftoff, this tick
    }

    nisp_forces_t f;
    compute_forces(v, dt_s, &f);

    // --- attitude: aero torque (frame -> vehicle) plus the control command
    float tau_aero[3];
    nisp_quat_rotate_inv(&v->q, f.torque_aero, tau_aero);

    // Damping is isotropic -- a scalar times the rate vector -- so it needs
    // no frame change at all. kg*m^2/s times rad/s is N*m, and N*m is a
    // thousandth of the kN*m every other torque here is in.
    //
    // Capped by the same rule as every other damper here (F-30): it may
    // remove a rotation and may not reverse one.
    float tau_damp[3] = {0.0f, 0.0f, 0.0f};
    float const damp_k = (float)f.damp_coeff * 0.001f;
    if (damp_k > 0.0f) {
        for (int i = 0; i < 3; i++) tau_damp[i] = -damp_k * v->omega[i];
        float const w = sqrtf(v->omega[0] * v->omega[0] + v->omega[1] * v->omega[1] + v->omega[2] * v->omega[2]);
        float const it = s->inertia_t_m2 > INERTIA_MIN ? s->inertia_t_m2 : INERTIA_MIN;
        clamp_mag(tau_damp, it * w / (float)dt_s);
    }

    float tau_cmd[3] = {0.0f, 0.0f, 0.0f};
    if (ctl != NULL) nisp_control_torque(v, ctl, (float)dt_s, tau_cmd);
    float tau[3] = {tau_aero[0] + tau_damp[0] + tau_cmd[0], tau_aero[1] + tau_damp[1] + tau_cmd[1],
                    tau_aero[2] + tau_damp[2] + tau_cmd[2]};

    euler_step(s, v->omega, tau, (float)dt_s);

    // The last-resort rate cap (F-30). Every torque here is capped against
    // the tick, so this should never bind; if it ever does, something in the
    // force model is wrong, and a ship tumbling at eight rad/s is a visibly
    // wrong answer rather than a subtly wrong one.
    float const w_mag = sqrtf(v->omega[0] * v->omega[0] + v->omega[1] * v->omega[1] + v->omega[2] * v->omega[2]);
    if (w_mag > NISP_OMEGA_CAP) {
        float const k = NISP_OMEGA_CAP / w_mag;
        for (int i = 0; i < 3; i++) v->omega[i] *= k;
    }

    nisp_quat_integrate(&v->q, v->omega, (float)dt_s);

    // --- translation: velocity Verlet, with the velocity-dependent drag
    // evaluated at the half-step velocity rather than at the start.
    double a0[3];
    for (int i = 0; i < 3; i++) a0[i] = f.a_grav[i] + f.a_drag[i] + f.a_thrust[i];

    double p1[3], vh[3];
    for (int i = 0; i < 3; i++) {
        p1[i] = v->pos_km[i] + v->vel_kms[i] * dt_s + 0.5 * a0[i] * dt_s * dt_s;
        vh[i] = v->vel_kms[i] + 0.5 * a0[i] * dt_s;
    }

    double a1[3];
    accel_at(v, s, &f, dt_s, p1, vh, a1);

    for (int i = 0; i < 3; i++) {
        v->pos_km[i] = p1[i];
        v->vel_kms[i] = vh[i] + 0.5 * a1[i] * dt_s;
    }

    // --- propellant, and the boosters that run out with it (Part F3)
    nisp_stack_burn(s, (float)dt_s);

    // --- heat, from the same air the drag came from (Part A4)
    nisp_heat_input_t const hi = {v->body, f.alt_km, (float)f.v_rel_mag, f.cos_flow_axis};
    nisp_heat_tick(s, &hi, (float)dt_s);

    // --- whose gravity this is now (Part O4)
    soi_update(v, t + dt_s);
}

// --- Patched conics in flight (Part O4) --------------------------------

// One handover per tick, and the direction of the test is the one the
// ephemeris makes cheap: leaving is a radius test against the current
// body's own sphere, entering is a distance test against each child's.
static void soi_update(nisp_vessel_t* v, double t) {
    nisp_body_t const* b = nisp_body(v->body);
    double const r = mag3(v->pos_km);

    if (b->parent < BODY_COUNT && b->soi_km > 0.0f && r > (double)b->soi_km) {
        double dp[3], dv[3], pv[3];
        nisp_body_relative(v->body, b->parent, t, dp);
        nisp_body_velocity(v->body, t, dv);
        nisp_body_velocity(b->parent, t, pv);
        for (int i = 0; i < 3; i++) {
            v->pos_km[i] += dp[i];
            // The stored velocity is relative to the body being left, so
            // what has to be added is that body's velocity relative to the
            // one taking over -- not its velocity in the sun's frame (F-35).
            // Frodo is the root and its velocity is zero, which is why the
            // leaving case looked right on its own.
            v->vel_kms[i] += dv[i] - pv[i];
        }
        v->body = b->parent;
        return;
    }

    int best = -1;
    double best_d = 1e300;
    double best_rel[3] = {0.0, 0.0, 0.0};
    for (int cand = 0; cand < BODY_COUNT; cand++) {
        nisp_body_t const* c = nisp_body(cand);
        if (c->parent != v->body || c->soi_km <= 0.0f) continue;
        double rel[3];
        nisp_body_relative(cand, v->body, t, rel);
        double d[3] = {v->pos_km[0] - rel[0], v->pos_km[1] - rel[1], v->pos_km[2] - rel[2]};
        double const dist = mag3(d);
        if (dist < (double)c->soi_km && dist < best_d) {
            best_d = dist;
            best = cand;
            for (int i = 0; i < 3; i++) best_rel[i] = rel[i];
        }
    }

    if (best >= 0) {
        double dv[3], cur[3];
        nisp_body_velocity(best, t, dv);
        nisp_body_velocity(v->body, t, cur);
        for (int i = 0; i < 3; i++) {
            v->pos_km[i] -= best_rel[i];
            // The same frame change as the position, and the same mistake
            // to avoid (F-35): subtracting the child's velocity in the
            // sun's frame from a velocity that is relative to the parent
            // leaves the parent's own 9.3 km/s in the answer, which turns
            // a Cheese approach into a Cheese flyby at ten times escape
            // speed. sim/patches.c's handover() is the correct form.
            v->vel_kms[i] -= dv[i] - cur[i];
        }
        v->body = best;
    }
}

// --- Coasting: analytic, whatever the warp (Part F1) -------------------

// A coast stops at the point where the analytic answer stops being the
// right one -- the air, or the ground -- and reports how much simulated
// time it actually covered. The caller spends the rest in substeps.
static bool coast_stops_here(int body, double const pos_km[3]) {
    float const alt = nisp_altitude(body, pos_km);
    if (alt <= 0.0f) return true;
    return nisp_atm_density(body, alt) > 0.0f;
}

static bool coast_stops_at(int body, nisp_state_t const* st) {
    double const pos[3] = {(double)st->r[0], (double)st->r[1], (double)st->r[2]};
    return coast_stops_here(body, pos);
}

static int chain_index_at(nisp_patch_t const* chain, int count, double t) {
    for (int i = 0; i < count; i++) {
        if (t >= chain[i].t_enter && t <= chain[i].t_exit) return i;
    }
    return -1;
}

static bool chain_state_at(nisp_patch_t const* chain, int count, double t, nisp_state_t* out,
                           int* body) {
    int const i = chain_index_at(chain, count, t);
    if (i < 0) return false;
    nisp_elements_to_state(&chain[i].el, t, out);
    *body = chain[i].body;
    return true;
}

// The edge of the air is inside a chunk, never on a chunk boundary (F-34).
// A chunk exit is an SOI handover, and a warp-10,000 frame is 167 simulated
// seconds: an orbit can dive through the atmosphere and come back out
// without ever touching a boundary, and the old code happily flew the whole
// way through it and reported the ship underground. So bisect the conic.
// COAST_BISECT_S of simulated time costs about a dozen Kepler solves, which
// is what lets the ticks take over exactly where the analytic answer stops
// being one.
#define COAST_BISECT_S 0.001
#define COAST_BISECT_MAX 40

// `hi` stops, `lo` does not. Leaves the vessel state at the last moment the
// conic is still the right answer and returns that absolute time.
static double coast_bisect(nisp_patch_t const* chain, int count, double lo, double hi,
                           nisp_state_t* out, int* body) {
    int const idx = chain_index_at(chain, count, hi);
    if (idx < 0) return lo;
    if (lo < chain[idx].t_enter) lo = chain[idx].t_enter;

    for (int i = 0; i < COAST_BISECT_MAX && hi - lo > COAST_BISECT_S; i++) {
        double const mid = 0.5 * (lo + hi);
        nisp_state_t at_mid;
        nisp_elements_to_state(&chain[idx].el, mid, &at_mid);
        if (coast_stops_at(chain[idx].body, &at_mid)) hi = mid; else lo = mid;
    }

    nisp_elements_to_state(&chain[idx].el, lo, out);
    *body = chain[idx].body;
    return lo;
}

double nisp_vessel_coast(nisp_vessel_t* v, double t, double dt_s) {
    if (v == NULL || dt_s <= 0.0 || !v->alive) return 0.0;
    if (v->landed) {
        rest_on_ground(v, t + dt_s);
        return dt_s;
    }

    nisp_state_t st;
    nisp_vessel_state(v, &st);

    double done = 0.0;
    int guard = 0;
    while (done < dt_s - 1e-9 && guard++ < 16) {
        nisp_patch_t chain[NISP_PATCH_MAX];
        int const n = nisp_patch_predict(&st, v->body, t + done, dt_s - done, chain, NISP_PATCH_MAX);
        if (n <= 0) break;

        // The frame ends the chain, not the other way round: the last chunk
        // normally runs past the end of the frame, and stopping at its exit
        // instead of at the frame would drop simulated time.
        double t_end = chain[n - 1].t_exit;
        if (t_end > t + dt_s) t_end = t + dt_s;

        nisp_state_t got;
        int body = v->body;
        if (!chain_state_at(chain, n, t_end, &got, &body)) break;

        if (coast_stops_at(body, &got)) {
            nisp_state_t clear;
            int clear_body = v->body;
            double const stop_t = coast_bisect(chain, n, t + done, t_end, &clear, &clear_body);
            for (int i = 0; i < 3; i++) {
                v->pos_km[i] = clear.r[i];
                v->vel_kms[i] = clear.v[i];
            }
            v->body = clear_body;
            if (stop_t <= t + done) break;  // already at the edge: the ticks take the frame
            done = stop_t - t;
            break;
        }

        for (int i = 0; i < 3; i++) {
            v->pos_km[i] = got.r[i];
            v->vel_kms[i] = got.v[i];
        }
        v->body = body;
        done = t_end - t;
        nisp_vessel_state(v, &st);
    }

    // Torque-free, the body rate still turns the ship, and the quaternion
    // step is first order in dt -- so a coast chops its attitude into
    // COAST_ATT_DT pieces while its position goes in one analytic jump.
    // The attitude is paid for over the time the coast actually covered,
    // not the time it was asked for: a coast that stopped at the air has no
    // business spinning the ship for the rest of the frame (F-34).
    if (done > 0.0) {
        int steps = (int)ceil(done / COAST_ATT_DT);
        if (steps > COAST_STEPS_MAX) steps = COAST_STEPS_MAX;
        if (steps < 1) steps = 1;
        float const h = (float)(done / (double)steps);
        float const no_torque[3] = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < steps; i++) {
            euler_step(&v->stack, v->omega, no_torque, h);
            nisp_quat_integrate(&v->q, v->omega, h);
        }
    }

    // Nothing but cooling up here: a coast is by definition above the air
    // it would heat the ship in (Part A4).
    nisp_heat_input_t const cool = {v->body, nisp_altitude(v->body, v->pos_km), 0.0f, 0.0f};
    nisp_heat_tick(&v->stack, &cool, (float)done);

    return done;
}

// --- Putting a ship somewhere ------------------------------------------

static void clear_vessel(nisp_vessel_t* v) {
    for (int i = 0; i < 3; i++) {
        v->pos_km[i] = 0.0;
        v->vel_kms[i] = 0.0;
        v->omega[i] = 0.0f;
    }
    v->body = BODY_BOB;
    nisp_quat_identity(&v->q);
    v->landed = false;
    v->alive = true;
    v->lat_rad = 0.0f;
    v->lon_rad = 0.0f;
    v->impact_ms = 0.0f;
    v->impact_tilt_deg = 0.0f;
    v->wreck_kind = -1;
    v->wreck_stage = -1;
}

void nisp_vessel_ground(nisp_vessel_t* v, nisp_design_t const* d, int body, double t, float lat_rad,
                       float lon_rad) {
    if (v == NULL) return;
    clear_vessel(v);
    nisp_stack_build(d, &v->stack);
    nisp_stack_cut(&v->stack);

    v->body = body;
    nisp_surface_point(body, t, lat_rad, lon_rad, v->pos_km);
    nisp_surface_velocity(body, t, lat_rad, lon_rad, v->vel_kms);
    v->lat_rad = lat_rad;
    v->lon_rad = lon_rad;

    double up[3], north[3], east[3];
    nisp_local_frame(body, v->pos_km, up, north, east);
    float const dir[3] = {(float)up[0], (float)up[1], (float)up[2]};
    nisp_quat_from_dir(dir, &v->q);

    v->landed = true;  // resting on the surface, co-rotating with it
}

void nisp_vessel_pad(nisp_vessel_t* v, nisp_design_t const* d, int body, double t) {
    // The pad is one place on the ground, so it is this and two numbers.
    nisp_vessel_ground(v, d, body, t, NISP_PAD_LAT, NISP_PAD_LON);
    v->landed = true;  // the mount holds it until the engines say otherwise
}

void nisp_vessel_orbit(nisp_vessel_t* v, nisp_design_t const* d, int body,
                       nisp_state_t const* state, double t, float const nose_dir[3]) {
    if (v == NULL || state == NULL) return;
    clear_vessel(v);
    nisp_stack_build(d, &v->stack);
    nisp_stack_cut(&v->stack);

    v->body = body;
    for (int i = 0; i < 3; i++) {
        v->pos_km[i] = state->r[i];
        v->vel_kms[i] = state->v[i];
    }
    nisp_quat_from_dir(nose_dir, &v->q);
    (void)t;  // the state is already at t; there is nothing to propagate
}

void nisp_vessel_copy(nisp_vessel_t const* src, nisp_vessel_t* dst) {
    if (src == NULL || dst == NULL) return;
    *dst = *src;
}

// --- Reading it ---------------------------------------------------------

void nisp_vessel_state(nisp_vessel_t const* v, nisp_state_t* out) {
    if (v == NULL || out == NULL) return;
    for (int i = 0; i < 3; i++) {
        out->r[i] = (float)v->pos_km[i];
        out->v[i] = (float)v->vel_kms[i];
    }
}

void nisp_vessel_nose(nisp_vessel_t const* v, float out[3]) {
    if (v == NULL || out == NULL) return;
    nisp_quat_nose(&v->q, out);
}

void nisp_vessel_prograde(nisp_vessel_t const* v, float out[3]) {
    double const m = mag3(v->vel_kms);
    if (m < VEC_MIN) {
        nisp_vessel_nose(v, out);
        return;
    }
    for (int i = 0; i < 3; i++) out[i] = (float)(v->vel_kms[i] / m);
}

void nisp_vessel_normal(nisp_vessel_t const* v, float out[3]) {
    double h[3];
    cross3(v->pos_km, v->vel_kms, h);
    double const m = mag3(h);
    if (m < VEC_MIN) {
        nisp_vessel_nose(v, out);
        return;
    }
    for (int i = 0; i < 3; i++) out[i] = (float)(h[i] / m);
}

void nisp_vessel_radial_out(nisp_vessel_t const* v, float out[3]) {
    double const m = mag3(v->pos_km);
    if (m < VEC_MIN) {
        nisp_vessel_nose(v, out);
        return;
    }
    for (int i = 0; i < 3; i++) out[i] = (float)(v->pos_km[i] / m);
}

bool nisp_vessel_reference(nisp_vessel_t const* v, int mode, float out[3]) {
    if (v == NULL || out == NULL) return false;
    switch (mode) {
        case NISP_SAS_HOLD:
            nisp_vessel_nose(v, out);
            return true;
        case NISP_SAS_PROGRADE:
            nisp_vessel_prograde(v, out);
            return true;
        case NISP_SAS_RETROGRADE: {
            float p[3];
            nisp_vessel_prograde(v, p);
            for (int i = 0; i < 3; i++) out[i] = -p[i];
            return true;
        }
        case NISP_SAS_NORMAL:
            nisp_vessel_normal(v, out);
            return true;
        case NISP_SAS_RADIAL_OUT:
            nisp_vessel_radial_out(v, out);
            return true;
        case NISP_SAS_RADIAL_IN: {
            float u[3];
            nisp_vessel_radial_out(v, u);
            for (int i = 0; i < 3; i++) out[i] = -u[i];
            return true;
        }
        default:
            return false;  // NISP_SAS_BURN needs an armed node (step 8.3)
    }
}

float nisp_vessel_periapsis_alt_km(nisp_vessel_t const* v) {
    if (v == NULL || !v->alive) return 0.0f;
    nisp_state_t st;
    nisp_vessel_state(v, &st);
    nisp_elements_t el;
    // The epoch only stamps the struct; a periapsis radius is a property of
    // the conic, not of the moment it was measured at.
    if (!nisp_state_to_elements(&st, v->body, 0.0, &el)) return 0.0f;
    return (float)(el.periapsis_km - nisp_body(v->body)->radius_km);
}

void nisp_vessel_readout(nisp_vessel_t const* v, double t, nisp_readout_t* out) {
    if (v == NULL || out == NULL) return;

    nisp_body_t const* b = nisp_body(v->body);
    nisp_stack_t const* s = &v->stack;

    out->alt_km = nisp_altitude(v->body, v->pos_km);
    out->speed_ms = (float)mag3(v->vel_kms) * (float)NISP_MS_PER_KMS;

    double air[3];
    frame_velocity(v->body, v->pos_km, air);
    double vr[3] = {v->vel_kms[0] - air[0], v->vel_kms[1] - air[1], v->vel_kms[2] - air[2]};
    out->v_rel_ms = (float)mag3(vr) * (float)NISP_MS_PER_KMS;

    out->rho = nisp_atm_density(v->body, out->alt_km);
    out->has_air = nisp_has_atmosphere(v->body);
    out->in_air = out->rho > 0.0f;
    out->q_dyn_pa = 0.5f * out->rho * out->v_rel_ms * out->v_rel_ms;

    float const mass = s->mass_t > MASS_MIN ? s->mass_t : MASS_MIN;
    out->mass_t = s->mass_t;
    out->thrust_kn = s->thrust_kn;
    out->accel_ms2 = s->thrust_kn / mass;  // kN per tonne is m/s^2

    double const r2 = dot3(v->pos_km, v->pos_km);
    double const g_local = b->mu / (r2 > VEC_MIN ? r2 : VEC_MIN) * NISP_MS_PER_KMS;  // m/s^2
    out->twr = g_local > 1e-9 ? out->accel_ms2 / (float)g_local : 0.0f;

    out->static_margin_m = s->static_margin_m;
    out->heat_part = nisp_stack_hottest(s);
    out->heat_max = nisp_stack_heat_load(s);
    out->authority_knm = nisp_control_authority_knm(s, true);

    nisp_state_t st;
    nisp_vessel_state(v, &st);
    nisp_elements_t el;
    out->escaped = false;
    out->apoapsis_km = 0.0f;
    out->periapsis_km = 0.0f;
    out->period_s = 0.0f;
    out->incl_deg = 0.0f;
    out->time_to_ap_s = -1.0f;
    out->time_to_pe_s = -1.0f;
    if (nisp_state_to_elements(&st, v->body, t, &el)) {
        out->escaped = el.e >= 1.0;
        out->periapsis_km = (float)(el.periapsis_km - b->radius_km);
        out->apoapsis_km = el.e < 1.0 ? (float)(el.apoapsis_km - b->radius_km) : 0.0f;
        out->period_s = (float)el.period_s;
        out->incl_deg = (float)(el.inc * NISP_RAD2DEG);
        out->time_to_ap_s = (float)nisp_time_to_apsis(&el, t, true);
        out->time_to_pe_s = (float)nisp_time_to_apsis(&el, t, false);
    }

    double up[3], north[3], east[3];
    nisp_local_frame(v->body, v->pos_km, up, north, east);
    float nose[3];
    nisp_vessel_nose(v, nose);
    out->nose_up_dot = (float)(nose[0] * up[0] + nose[1] * up[1] + nose[2] * up[2]);
}

// --- The things the player asks for ------------------------------------

// A decoupler's kick, shared by staging and by a burn-through: two m/s
// along the axis is enough that a spent stage does not sit inside the live
// one, and small enough that it is not a maneuver (Part F3).
#define NISP_SEP_IMPULSE_MS 2.0f

static void axis_kick(nisp_vessel_t* v, float sign) {
    float nose[3];
    nisp_vessel_nose(v, nose);
    double const dv = (double)sign * NISP_SEP_IMPULSE_MS * NISP_KM_PER_M;
    for (int i = 0; i < 3; i++) v->vel_kms[i] += dv * (double)nose[i];
}

void nisp_vessel_stage(nisp_vessel_t* v) {
    if (v == NULL || !v->alive) return;
    // nisp_stack_advance arms the stage above at zero; the throttle the
    // stage below was at has to be carried across by hand, which is what a
    // player pressing the stage key mid-burn expects (Part F3).
    float const throttle = v->stack.throttle;
    int const removed = nisp_stack_advance(&v->stack);
    if (removed <= 0) return;
    nisp_stack_ignite(&v->stack, v->stack.live_stage, throttle);
    axis_kick(v, +1.0f);
}

void nisp_vessel_chute(nisp_vessel_t* v, bool deploy) {
    if (v == NULL || !v->alive) return;
    nisp_stack_deploy_chute(&v->stack, deploy);
}

void nisp_vessel_legs(nisp_vessel_t* v) {
    if (v == NULL || !v->alive) return;
    int best = -1;
    for (int i = 0; i < v->stack.count; i++) {
        nisp_part_t const* p = &v->stack.part[i];
        if (p->slot != NISP_PART_KIND_LEGS) continue;
        if (best < 0 || p->stage > best) best = p->stage;  // the lowest set: the one that lands
    }
    if (best < 0) return;
    nisp_stack_deploy_legs(&v->stack, best, !nisp_stack_legs_deployed(&v->stack, best));
}

void nisp_vessel_touchdown_rating(nisp_vessel_t const* v, float* rated_v_ms, float* rated_tilt_deg) {
    float best_v = NISP_TOUCHDOWN_BELL_MS;
    float best_tilt = NISP_TOUCHDOWN_BELL_TILT_DEG;

    nisp_stack_t const* s = &v->stack;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        if (p->slot != NISP_PART_KIND_LEGS) continue;
        if (!nisp_stack_legs_deployed(s, p->stage)) continue;
        nisp_legs_row_t const* row = nisp_legs_row(s->legs_id[p->stage]);
        if (row->rated_v_ms > best_v) best_v = row->rated_v_ms;
        if (row->rated_tilt_deg > best_tilt) best_tilt = row->rated_tilt_deg;
    }

    if (rated_v_ms) *rated_v_ms = best_v;
    if (rated_tilt_deg) *rated_tilt_deg = best_tilt;
}

// --- What a tick decided ------------------------------------------------

nisp_hit_t nisp_vessel_resolve(nisp_vessel_t* v, double t, nisp_vessel_t* other) {
    if (v == NULL) return NISP_HIT_NONE;
    if (!v->alive) return NISP_HIT_LOST;
    if (v->landed) return NISP_HIT_NONE;

    // Heat first: a part that has had enough goes, and the stack is cut at
    // it through the same path staging uses (Part F5).
    int const idx = nisp_heat_breach(&v->stack);
    if (idx >= 0) {
        int const kind = v->stack.part[idx].kind;
        int const stage = v->stack.part[idx].stage;

        // Static, not a local: 2.3 kB of frame here sits under the mission
        // step on the badge's 8 kB main-task stack (F-45). Nothing in the
        // flight model is reentrant -- it all runs on that one task.
        static nisp_stack_t lower;
        if (!nisp_stack_split(&v->stack, idx, &lower)) {
            v->alive = false;  // the capsule was the part that went
            v->wreck_kind = kind;
            v->wreck_stage = stage;
            return NISP_HIT_LOST;
        }

        if (other != NULL) {
            nisp_vessel_copy(v, other);
            other->stack = lower;
            other->alive = true;
            other->landed = false;
            axis_kick(other, -1.0f);  // the shed fragment trails away
        }
        axis_kick(v, +1.0f);
        nisp_stack_ignite(&v->stack, v->stack.live_stage, v->stack.throttle);
        return NISP_HIT_SPLIT;
    }

    // Ground contact. Speed is measured against the ground it hits, not
    // against the body's centre: a landing on the day side of a turning
    // planet is not 174.9 m/s faster than one on the night side.
    float const alt = nisp_altitude(v->body, v->pos_km);
    if (alt > 0.0f) return NISP_HIT_NONE;

    double air[3];
    frame_velocity(v->body, v->pos_km, air);
    double vr[3] = {v->vel_kms[0] - air[0], v->vel_kms[1] - air[1], v->vel_kms[2] - air[2]};
    float const speed_ms = (float)mag3(vr) * (float)NISP_MS_PER_KMS;

    double up[3], north[3], east[3];
    nisp_local_frame(v->body, v->pos_km, up, north, east);
    float nose[3];
    nisp_vessel_nose(v, nose);
    // Either end down counts as vertical: a leged stage lands tail-first
    // and a capsule under chute lands shield-first (Part A3).
    float d = nose[0] * (float)up[0] + nose[1] * (float)up[1] + nose[2] * (float)up[2];
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    float const tilt_deg = (float)(acos(fabs(d)) * NISP_RAD2DEG);

    float rated_v, rated_tilt;
    nisp_vessel_touchdown_rating(v, &rated_v, &rated_tilt);

    v->impact_ms = speed_ms;
    v->impact_tilt_deg = tilt_deg;

    nisp_latlon(v->body, v->pos_km, &v->lat_rad, &v->lon_rad);

    if (speed_ms <= rated_v && tilt_deg <= rated_tilt) {
        v->landed = true;
        rest_on_ground(v, t);
        return NISP_HIT_LANDED;
    }

    v->alive = false;
    int lowest = 0;
    for (int i = 0; i < v->stack.count; i++) {
        if (v->stack.part[i].slot == NISP_PART_KIND_LEGS) continue;
        lowest = i;  // the last row is the tail: what actually hit
    }
    v->wreck_kind = v->stack.part[lowest].kind;
    v->wreck_stage = v->stack.part[lowest].stage;
    return NISP_HIT_CRASHED;
}

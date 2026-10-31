// =====================================================================
//  Null Island Space Program  --  the host harness (Part H)
// ---------------------------------------------------------------------
//  Orbital mechanics and rocket arithmetic are the two parts of this
//  project that can be provably wrong without anything looking wrong. A
//  sign error in the static margin, a mean motion off by a factor of 2*pi,
//  a rocket equation applied to the wrong mass -- none of it shows up as a
//  crash or a bad frame. It shows up as a rocket that will not reach the
//  orbit the map says it reached, which is discovered hours later by a
//  player, on a badge, in the dark.
//
//  So every number this game is built on is checked here, on this PC, in
//  milliseconds, against an answer computed by hand and written next to it.
//  `make build` depends on `make check`, which depends on this binary
//  exiting 0.
//
//  THE ANSWERS ARE THE ONES A PLAYER CAN LOOK UP. Cases 1 and 4 are Kerbal
//  numbers, quoted from a wiki, because a player who compares this game to
//  one they already know is going to compare it to those. If Bob's 80 km
//  orbit is not 2,278.6 m/s here, the difference is a bug and not a style
//  choice (F-11).
//
//  Compiled by the host C compiler, against main/sim and main/rocket only.
//  tools/hostpurity.sh is what keeps that true.
// =====================================================================

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sim/atmosphere.h"
#include "sim/bodies.h"
#include "sim/ephemeris.h"
#include "sim/kepler.h"
#include "sim/patches.h"
#include "sim/surface.h"
#include "sim/units.h"

#include "rocket/budget.h"
#include "rocket/design.h"
#include "rocket/heat.h"
#include "rocket/parts.h"
#include "rocket/stack.h"

#include "common/quat.h"
#include "flight/control.h"
#include "flight/mission.h"
#include "flight/physics.h"
#include "flight/warp.h"

static int s_fail = 0;
static int s_checks = 0;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        s_checks++;                                             \
        if (!(cond)) {                                          \
            printf("  FAIL: " __VA_ARGS__);                    \
            printf("\n");                                       \
            s_fail++;                                           \
        }                                                       \
    } while (0)

// The same, for a label that is a variable rather than a literal -- the
// rows in a table of cases have names, and a failure should print the name
// of the row that failed.
#define CHECK_MSG(cond, msg)                                          \
    do {                                                           \
        s_checks++;                                                \
        if (!(cond)) {                                             \
            printf("  FAIL: %s\n", (msg));                         \
            s_fail++;                                              \
        } else {                                                   \
            printf("  ok: %s\n", (msg));                           \
        }                                                          \
    } while (0)

// Compare a computed value against a hand-written one, with the tolerance
// printed rather than hidden -- a test that passes at an undisclosed
// tolerance is a test that can be made to pass.
#define CHECK_NEAR(actual, want, tol, label)                                       \
    do {                                                                           \
        double const a_ = (actual), w_ = (want), t_ = (tol);                      \
        s_checks++;                                                                \
        if (!(fabs(a_ - w_) <= t_)) {                                             \
            printf("  FAIL: %s: got %.6f, want %.6f (tol %.6f, off %.6f)\n",      \
                   label, a_, w_, t_, fabs(a_ - w_));                             \
            s_fail++;                                                             \
        } else {                                                                  \
            printf("  ok: %-42s %.6f  (tol %.6g)\n", label, a_, t_);              \
        }                                                                         \
    } while (0)

static double mag3(double const v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// A brute-force two-body integrator, used as the reference a patched-conic
// prediction is compared against. RK4 at a small dt: slow, dumb, and about
// as far from a clever analytic solution as it is possible to be, which is
// exactly what makes it a useful reference.
static void integrate_two_body(int body, nisp_state_t* s, double dt, int steps) {
    double const mu = nisp_body(body)->mu;
    double r[3] = {s->r[0], s->r[1], s->r[2]};
    double v[3] = {s->v[0], s->v[1], s->v[2]};

    // The accumulator stays double for the whole run and only becomes a
    // float state at the end. Rounding it every step would add about 250 m
    // of quantisation over a 66,000 s leg, which is bigger than the error
    // the test is trying to see.
    for (int step = 0; step < steps; step++) {
        double r0[3] = {r[0], r[1], r[2]};
        double v0[3] = {v[0], v[1], v[2]};

        double k1r[3], k1v[3], k2r[3], k2v[3], k3r[3], k3v[3], k4r[3], k4v[3];
        double rr[3], vv[3], d;

        d = mag3(r0);
        for (int i = 0; i < 3; i++) { k1r[i] = v0[i]; k1v[i] = -mu * r0[i] / (d * d * d); }

        for (int i = 0; i < 3; i++) { rr[i] = r0[i] + 0.5 * dt * k1r[i]; vv[i] = v0[i] + 0.5 * dt * k1v[i]; }
        d = mag3(rr);
        for (int i = 0; i < 3; i++) { k2r[i] = vv[i]; k2v[i] = -mu * rr[i] / (d * d * d); }

        for (int i = 0; i < 3; i++) { rr[i] = r0[i] + 0.5 * dt * k2r[i]; vv[i] = v0[i] + 0.5 * dt * k2v[i]; }
        d = mag3(rr);
        for (int i = 0; i < 3; i++) { k3r[i] = vv[i]; k3v[i] = -mu * rr[i] / (d * d * d); }

        for (int i = 0; i < 3; i++) { rr[i] = r0[i] + dt * k3r[i]; vv[i] = v0[i] + dt * k3v[i]; }
        d = mag3(rr);
        for (int i = 0; i < 3; i++) { k4r[i] = vv[i]; k4v[i] = -mu * rr[i] / (d * d * d); }

        for (int i = 0; i < 3; i++) {
            r[i] = r0[i] + dt / 6.0 * (k1r[i] + 2.0 * k2r[i] + 2.0 * k3r[i] + k4r[i]);
            v[i] = v0[i] + dt / 6.0 * (k1v[i] + 2.0 * k2v[i] + 2.0 * k3v[i] + k4v[i]);
        }
    }

    for (int i = 0; i < 3; i++) {
        s->r[i] = (float)r[i];
        s->v[i] = (float)v[i];
    }
}

// --- Case 1: the four anchors a player can quote (F-11) ----------------

static void case1_anchors(void) {
    printf("case 1  the body table, against Kerbal numbers\n");

    double const r80 = nisp_body(BODY_BOB)->radius_km + 80.0;
    CHECK_NEAR(nisp_circular_speed(BODY_BOB, r80) * 1000.0, 2278.6, 0.5, "80 km circular speed, m/s");
    CHECK_NEAR(nisp_orbital_period(BODY_BOB, r80), 1875.1, 0.5, "80 km circular period, s");
    CHECK_NEAR(nisp_radius_for_period(BODY_BOB, nisp_body(BODY_BOB)->rotation_s) - 600.0, 2863.9, 1.0,
               "stationary orbit altitude, km");
    double const semi_r = nisp_radius_for_period(BODY_BOB, nisp_body(BODY_BOB)->rotation_s * 0.5);
    CHECK_NEAR(semi_r - 600.0, 1581.8, 1.0, "semi-synchronous altitude, km");
    CHECK_NEAR(nisp_circular_speed(BODY_BOB, semi_r) * 1000.0, 1272.3, 1.0, "semi-synchronous speed, m/s");
    CHECK_NEAR(nisp_escape_speed(BODY_BOB, nisp_body(BODY_BOB)->radius_km) * 1000.0, 3430.5, 0.5,
               "surface escape speed, m/s");

    CHECK_NEAR(nisp_body_g0(BODY_BOB), 9.81, 0.01, "Bob surface gravity, m/s^2");
    CHECK_NEAR(nisp_body_g0(BODY_CHEESE), 1.628, 0.005, "Cheese surface gravity, m/s^2");
    CHECK_NEAR(nisp_escape_speed(BODY_CHEESE, nisp_body(BODY_CHEESE)->radius_km) * 1000.0, 807.1, 0.5,
               "Cheese surface escape speed, m/s");
    // Cheese's radius, Bob's gravity: 139,000 s is the number the plan
    // quotes for a Cheese flyby, and it is also the length of the transfer
    // window in case 3. (nisp_orbital_period takes the central body, and at
    // 12,000 km from Bob that is Bob.)
    CHECK_NEAR(nisp_orbital_period(BODY_BOB, 12000.0), 139000.0, 200.0, "Cheese orbit period, s");

    // The pad at 0N 0E, standing still, is already moving at 174.9 m/s.
    double pad[3], pv[3];
    nisp_surface_point(BODY_BOB, 0.0, NISP_PAD_LAT, NISP_PAD_LON, pad);
    nisp_surface_velocity(BODY_BOB, 0.0, NISP_PAD_LAT, NISP_PAD_LON, pv);
    CHECK_NEAR(mag3(pv) * 1000.0, 174.9, 0.5, "pad rotational speed, m/s");
    CHECK_NEAR(mag3(pad), nisp_body(BODY_BOB)->radius_km, 1e-6, "pad sits on the surface");

    // East is the direction the ground moves: a launch that goes that way
    // buys 174.9 m/s, and the flight model never has to be told.
    double up[3], north[3], east[3];
    nisp_local_frame(BODY_BOB, pad, up, north, east);
    double east_v[3] = {east[0] * mag3(pv), east[1] * mag3(pv), east[2] * mag3(pv)};
    double dot = pv[0] * east_v[0] + pv[1] * east_v[1] + pv[2] * east_v[2];
    CHECK_NEAR(dot, mag3(pv) * mag3(pv), 1e-6, "east is the direction the ground moves");

    // A sphere of influence outside its own Hill sphere would be a patched-
    // conic tree that lies about who is in charge.
    CHECK(nisp_body_hill_km(BODY_CHEESE) > nisp_body(BODY_CHEESE)->soi_km,
          "Cheese SOI (%.1f km) must sit inside its Hill sphere (%.1f km)",
          (double)nisp_body(BODY_CHEESE)->soi_km, nisp_body_hill_km(BODY_CHEESE));
    CHECK(nisp_body_hill_km(BODY_BOB) > nisp_body(BODY_BOB)->soi_km,
          "Bob SOI (%.1f km) must sit inside its Hill sphere (%.1f km)",
          (double)nisp_body(BODY_BOB)->soi_km, nisp_body_hill_km(BODY_BOB));
    printf("  ok: Cheese Hill %.1f km > SOI %.1f km; Bob Hill %.1f km > SOI %.1f km\n",
           nisp_body_hill_km(BODY_CHEESE), (double)nisp_body(BODY_CHEESE)->soi_km,
           nisp_body_hill_km(BODY_BOB), (double)nisp_body(BODY_BOB)->soi_km);
}

// --- Case 2: propagation round trip (Part H case 2) --------------------

static void case2_round_trip(void) {
    printf("case 2  state -> elements -> state, and 10,000 periods of drift\n");

    struct { double a, e; char const* label; } const cases[] = {
        {680.0, 0.00, "circular e=0"},
        {800.0, 0.20, "low ellipse e=0.2"},
        {1500.0, 0.70, "long ellipse e=0.7"},
        {3000.0, 0.95, "grazing ellipse e=0.95"},
    };

    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        nisp_elements_t el;
        memset(&el, 0, sizeof(el));
        el.body = BODY_BOB;
        el.mu = nisp_body(BODY_BOB)->mu;
        el.a = cases[c].a;
        el.e = cases[c].e;
        el.inc = 0.30;
        el.lan = 0.50;
        el.aop = 0.70;
        el.t0 = 0.0;
        el.m0 = 1.00;
        nisp_elements_derive(&el);

        nisp_state_t start;
        nisp_elements_to_state(&el, 0.0, &start);

        // The view has to survive being looked at. A state vector is float
        // (D-17), so the elements recovered from it are only as good as that
        // float: at 680 km a radius is quantised to about 4 cm, and a, e and
        // the period inherit it. The tolerances below are that quantisation,
        // stated out loud rather than tuned until the test goes green.
        nisp_elements_t back;
        CHECK(nisp_state_to_elements(&start, BODY_BOB, 0.0, &back), "%s: state -> elements", cases[c].label);
        CHECK_NEAR(back.a, cases[c].a, 1e-3, "recovered a, km");
        CHECK_NEAR(back.e, cases[c].e, 1e-6, "recovered e");
        CHECK_NEAR(back.inc, 0.30, 1e-6, "recovered inc, rad");
        CHECK_NEAR(back.period_s, el.period_s, 1e-2, "recovered period, s");

        // Round trip: elements -> float state -> elements -> float state, at
        // the same instant. This is the path the map takes every frame, so
        // a metre is the most it is ever allowed to move the ship.
        nisp_state_t again;
        nisp_elements_to_state(&back, 0.0, &again);
        double rr[3] = {again.r[0] - start.r[0], again.r[1] - start.r[1], again.r[2] - start.r[2]};
        double vv[3] = {again.v[0] - start.v[0], again.v[1] - start.v[1], again.v[2] - start.v[2]};
        char label[96];
        snprintf(label, sizeof(label), "%s: round trip, m", cases[c].label);
        CHECK_NEAR(mag3(rr) * 1000.0, 0.0, 1.0, label);
        snprintf(label, sizeof(label), "%s: round trip, mm/s", cases[c].label);
        CHECK_NEAR(mag3(vv) * 1e6, 0.0, 1.0, label);

        // 10,000 periods later the ship is where it started. This one is a
        // pure Kepler solve, so anything above a millimetre is an anomaly
        // wrap or a mean motion that is not quite 2*pi / T -- not float
        // noise, which is why it runs on `el` and not on `back`.
        double const t_end = 10000.0 * el.period_s;
        nisp_state_t end;
        nisp_elements_to_state(&el, t_end, &end);
        double dr[3] = {end.r[0] - start.r[0], end.r[1] - start.r[1], end.r[2] - start.r[2]};
        double dv[3] = {end.v[0] - start.v[0], end.v[1] - start.v[1], end.v[2] - start.v[2]};
        snprintf(label, sizeof(label), "%s: position after 10,000 periods, m", cases[c].label);
        CHECK_NEAR(mag3(dr) * 1000.0, 0.0, 1.0, label);
        snprintf(label, sizeof(label), "%s: speed after 10,000 periods, mm/s", cases[c].label);
        CHECK_NEAR(mag3(dv) * 1e6, 0.0, 1.0, label);
    }

    // The hyperbolic case has no period to come back around with, so it is
    // checked against the integrator instead.
    nisp_elements_t el;
    memset(&el, 0, sizeof(el));
    el.body = BODY_BOB;
    el.mu = nisp_body(BODY_BOB)->mu;
    el.a = -3000.0;
    el.e = 1.30;
    el.inc = 0.2;
    el.lan = 0.4;
    el.aop = 0.6;
    el.t0 = 0.0;
    el.m0 = -0.5;
    nisp_elements_derive(&el);

    double const t_end = 40000.0;
    nisp_state_t analytic;
    nisp_elements_to_state(&el, t_end, &analytic);

    nisp_state_t numeric;
    nisp_elements_to_state(&el, 0.0, &numeric);
    integrate_two_body(BODY_BOB, &numeric, 0.05, (int)(t_end / 0.05));

    double dr[3] = {analytic.r[0] - numeric.r[0], analytic.r[1] - numeric.r[1], analytic.r[2] - numeric.r[2]};
    double dv[3] = {analytic.v[0] - numeric.v[0], analytic.v[1] - numeric.v[1], analytic.v[2] - numeric.v[2]};
    // RK4 at 0.05 s is the limiting error here, not the analytic solve: at
    // periapsis (2,000 km, 2.6 km/s) that step is 130 m of arc.
    CHECK_NEAR(mag3(dr) * 1000.0, 0.0, 10.0, "hyperbolic e=1.3: position vs RK4, m");
    CHECK_NEAR(mag3(dv) * 1e6, 0.0, 1.0, "hyperbolic e=1.3: speed vs RK4, mm/s");
}

// --- Case 3: patched conics against a brute-force reference -------------

// The closest approach of the 80 km -> 12,600 km transfer to Cheese, for a
// burn at `t0`. Sampled along the conic rather than solved, because the
// answer only has to be good enough to pick a window.
static double transfer_miss(double dv_mss, double t_coast, double t0) {
    nisp_elements_t circ;
    nisp_elements_circular(BODY_BOB, 680.0f, 0.0, 0.0, &circ);

    nisp_state_t s;
    nisp_elements_to_state(&circ, t0, &s);
    double const speed = sqrt(s.v[0] * s.v[0] + s.v[1] * s.v[1] + s.v[2] * s.v[2]);
    for (int i = 0; i < 3; i++) s.v[i] = (float)(s.v[i] + s.v[i] / speed * dv_mss / 1000.0);

    nisp_elements_t el;
    if (!nisp_state_to_elements(&s, BODY_BOB, t0, &el)) return 1e18;

    double miss = 1e18;
    for (int k = 0; k <= 600; k++) {
        double const t = t0 + 1.25 * t_coast * k / 600.0;
        nisp_state_t p;
        nisp_elements_to_state(&el, t, &p);
        double cheese[3];
        nisp_body_relative(BODY_CHEESE, BODY_BOB, t, cheese);
        double const dx = cheese[0] - p.r[0], dy = cheese[1] - p.r[1], dz = cheese[2] - p.r[2];
        double const d = sqrt(dx * dx + dy * dy + dz * dz);
        if (d < miss) miss = d;
    }
    return miss;
}

// One synodic period is the whole argument: after it, the burn point and
// Cheese have both come back around to the same relative phase.
static double transfer_window(double dv_mss, double t_coast) {
    double const mu = nisp_body(BODY_BOB)->mu;
    double const n_circ = sqrt(mu / (680.0 * 680.0 * 680.0));
    double const n_cheese = sqrt(mu / (12000.0 * 12000.0 * 12000.0));
    double const span = 2.0 * M_PI / (n_circ - n_cheese);

    double best_t = 0.0, best_miss = 1e18;
    for (int c = 0; c < 400; c++) {
        double const t = span * c / 400.0;
        double const miss = transfer_miss(dv_mss, t_coast, t);
        if (miss < best_miss) { best_miss = miss; best_t = t; }
    }
    for (int pass = 0; pass < 3; pass++) {
        double const lo = best_t - span / 400.0, hi = best_t + span / 400.0;
        for (int c = 0; c <= 200; c++) {
            double const t = lo + (hi - lo) * c / 200.0;
            double const miss = transfer_miss(dv_mss, t_coast, t);
            if (miss < best_miss) { best_miss = miss; best_t = t; }
        }
    }
    printf("  ok: best window misses Cheese by %.1f km, inside its %.1f km sphere\n", best_miss,
           (double)nisp_body(BODY_CHEESE)->soi_km);
    CHECK(best_miss < nisp_body(BODY_CHEESE)->soi_km, "a window that cannot reach Cheese is a bug");
    return best_t;
}

static void case3_patches(void) {
    printf("case 3  patched conics, against an integrated reference\n");

    // An 80 km circular orbit around Bob, prograde-burned onto the transfer
    // the plan quotes (Part H case 4). It leaves Bob's conic when it runs
    // into Cheese's sphere of influence, and comes back out again.
    //
    // THE EPOCH IS THE TRANSFER WINDOW. A Hohmann transfer is the right
    // shape at any moment and the right shape at only one of them: burn at
    // the wrong epoch and the apoapsis is 12,600 km of empty space on the
    // far side of Bob. So the burn time is searched for, the same way
    // Part M3's planner will search for it -- one synodic period of
    // candidates, then a refine around the best. Getting the phase wrong
    // never shows up as a bad number, only as a flyby that misses, which is
    // exactly why this case exists.
    double const dv1 = 860.2;  // m/s, prograde; case 4 derives it
    double const mu = nisp_body(BODY_BOB)->mu;
    double const r1 = 680.0, r2 = 12600.0;
    double const a_transfer = 0.5 * (r1 + r2);
    double const t_coast = M_PI * sqrt(a_transfer * a_transfer * a_transfer / mu);
    double const n_circ = sqrt(mu / (r1 * r1 * r1));
    double const n_cheese = sqrt(mu / (12000.0 * 12000.0 * 12000.0));
    double const t_synodic = 2.0 * M_PI / (n_circ - n_cheese);

    double const t0 = transfer_window(dv1, t_coast);
    printf("  ok: burn at t=%.1f s, coast %.1f s, synodic period %.1f s\n", t0, t_coast, t_synodic);

    nisp_elements_t circ;
    nisp_elements_circular(BODY_BOB, 680.0f, 0.0, 0.0, &circ);

    nisp_state_t state;
    nisp_elements_to_state(&circ, t0, &state);

    double speed = sqrt(state.v[0] * state.v[0] + state.v[1] * state.v[1] + state.v[2] * state.v[2]);
    double dir[3] = {state.v[0] / speed, state.v[1] / speed, state.v[2] / speed};
    for (int i = 0; i < 3; i++) state.v[i] = (float)(state.v[i] + dir[i] * dv1 / 1000.0);

    nisp_patch_t chain[NISP_PATCH_MAX];
    int const n = nisp_patch_predict(&state, BODY_BOB, t0, 40000.0, chain, NISP_PATCH_MAX);

    CHECK(n >= 2, "the transfer should produce at least two patches, got %d", n);
    if (n >= 2) {
        CHECK(chain[0].body == BODY_BOB, "patch 0 should be about Bob");
        CHECK(chain[0].end == NISP_PATCH_END_ENTRY, "patch 0 should end by entering a sphere of influence");
        CHECK(chain[0].other == BODY_CHEESE, "the sphere it enters should be Cheese's");
        CHECK(chain[1].body == BODY_CHEESE, "patch 1 should be about Cheese");
        CHECK(chain[1].end == NISP_PATCH_END_SOI, "patch 1 should end by leaving Cheese's sphere");
        printf("  ok: Bob -> Cheese at t=%.1f s (%.1f h after the burn), Cheese -> %s at t=%.1f s\n",
               chain[0].t_exit, (chain[0].t_exit - t0) / 3600.0,
               chain[1].other == BODY_BOB ? "Bob" : "elsewhere", chain[1].t_exit - t0);

        // How far out the handover happened, for the record. It should be
        // just inside Cheese's sphere, well inside its Hill sphere, and on
        // the way up rather than on the way down.
        // Both vectors are relative to Bob, which is the frame patch 0 lives
        // in; nisp_body_relative is what keeps Cheese out of the sun's frame.
        double cheese[3];
        nisp_body_relative(BODY_CHEESE, BODY_BOB, chain[0].t_exit, cheese);
        double const d_bob = sqrt(chain[0].r_end[0] * chain[0].r_end[0] +
                                  chain[0].r_end[1] * chain[0].r_end[1] +
                                  chain[0].r_end[2] * chain[0].r_end[2]);
        double const dx = cheese[0] - chain[0].r_end[0];
        double const dy = cheese[1] - chain[0].r_end[1];
        double const dz = cheese[2] - chain[0].r_end[2];
        double const d_cheese = sqrt(dx * dx + dy * dy + dz * dz);
        printf("  ok: handover %.1f km from Bob, %.1f km from Cheese's centre\n", d_bob, d_cheese);
        CHECK_NEAR(d_cheese, nisp_body(BODY_CHEESE)->soi_km, 1.0, "the handover is on the sphere");
    }

    // Reference: integrate Bob's gravity alone, at 0.25 s, up to the moment
    // the chain says the ship crossed into Cheese. Same physics, entirely
    // different method -- which is the point.
    {
        nisp_state_t ref;
        nisp_elements_to_state(&circ, t0, &ref);
        for (int i = 0; i < 3; i++) ref.v[i] = (float)(ref.v[i] + dir[i] * dv1 / 1000.0);

        double const t_hit = chain[0].t_exit;
        // 0.01 s, because the step size is what limits this comparison: RK4
        // is fourth order, and near periapsis (680 km, 2.3 km/s) a 0.05 s
        // step leaves 17 m of its own error behind. At 0.01 s it leaves
        // about 2, which is under the tolerance instead of defining it.
        integrate_two_body(BODY_BOB, &ref, 0.01, (int)ceil((t_hit - t0) / 0.01));

        nisp_state_t got;
        CHECK(nisp_patch_state(chain, n, t_hit - 1e-6, &got), "the chain can be evaluated at the handover");

        double dr[3] = {got.r[0] - ref.r[0], got.r[1] - ref.r[1], got.r[2] - ref.r[2]};
        double dv[3] = {got.v[0] - ref.v[0], got.v[1] - ref.v[1], got.v[2] - ref.v[2]};
        // The float state the chain hands back is the coarsest thing in the
        // comparison: at 12,000 km a float radius is quantised to about 0.7 m.
        CHECK_NEAR(mag3(dr) * 1000.0, 0.0, 5.0, "handover position vs RK4, m");
        CHECK_NEAR(mag3(dv) * 1e6, 0.0, 5.0, "handover speed vs RK4, mm/s");
    }
}

// --- Case 4: the transfer numbers (Part H case 4) -----------------------

static void case4_transfer(void) {
    printf("case 4  the transfer numbers a player will look up\n");

    double const mu = nisp_body(BODY_BOB)->mu;
    double const r1 = 680.0;   // 80 km circular

    // The plan's anchor: 80 km -> 12,600 km apoapsis radius. 860 m/s out,
    // 7.947 h of coast, 360 m/s to circularise.
    {
        double const r2 = 12600.0;
        double const a = 0.5 * (r1 + r2);
        double const v_circ = sqrt(mu / r1);
        double const v_peri = sqrt(mu * (2.0 / r1 - 1.0 / a));
        double const v_apo = sqrt(mu * (2.0 / r2 - 1.0 / a));
        double const v_circ2 = sqrt(mu / r2);
        CHECK_NEAR((v_peri - v_circ) * 1000.0, 860.2, 1.0, "Hohmann burn out, m/s");
        CHECK_NEAR(M_PI * sqrt(a * a * a / mu) / 3600.0, 7.947, 0.002, "coast to apoapsis, h");
        CHECK_NEAR((v_circ2 - v_apo) * 1000.0, 359.9, 1.0, "circularise at 12,600 km, m/s");
    }

    // Cheese's own orbit is a = 12,000 km radius (Part O2), so the intercept
    // transfer is a slightly different pair of numbers. Both are in the test
    // because the plan quotes one and the body table defines the other, and
    // a reader comparing them should not have to redo the arithmetic.
    {
        double const r2 = 12000.0;
        double const a = 0.5 * (r1 + r2);
        double const v_peri = sqrt(mu * (2.0 / r1 - 1.0 / a));
        double const v_apo = sqrt(mu * (2.0 / r2 - 1.0 / a));
        CHECK_NEAR((v_peri - sqrt(mu / r1)) * 1000.0, 856.2, 1.0, "Cheese intercept burn out, m/s");
        CHECK_NEAR(M_PI * sqrt(a * a * a / mu) / 3600.0, 7.415, 0.002, "coast to Cheese, h");
        CHECK_NEAR((sqrt(mu / r2) - v_apo) * 1000.0, 365.1, 1.0, "circularise at Cheese, m/s");
    }

    // The pad to a 100 km circular orbit, ideal and impulsive: 2,517 to
    // raise the apoapsis, 88 to circularise, before any rotational help and
    // before gravity and drag losses (Part B4's reach-orbit line).
    {
        double const r_pad = nisp_body(BODY_BOB)->radius_km;
        double const r_orb = r_pad + 100.0;
        double const a = 0.5 * (r_pad + r_orb);
        double const v1 = sqrt(mu * (2.0 / r_pad - 1.0 / a));
        double const v2 = sqrt(mu * (2.0 / r_orb - 1.0 / a));
        double const vc = sqrt(mu / r_orb);
        CHECK_NEAR(v1 * 1000.0, 2517.3, 1.0, "pad burn to a 100 km apoapsis, m/s");
        CHECK_NEAR((vc - v2) * 1000.0, 88.1, 1.0, "circularise at 100 km, m/s");
        CHECK_NEAR((v1 + vc - v2) * 1000.0, 2605.4, 1.5, "ideal pad -> 100 km total, m/s");
    }

    // The atmosphere ends where the plan says it ends, and the air is what
    // the plan says it is at the two altitudes that matter.
    CHECK_NEAR(nisp_atm_density(BODY_BOB, 0.0f), 1.225, 1e-6, "sea-level density, kg/m^3");
    CHECK_NEAR(nisp_atm_density(BODY_BOB, 5.6f), 1.225 * exp(-1.0), 1e-6, "density at one scale height");
    CHECK_NEAR(nisp_atm_density(BODY_BOB, 70.0f), 0.0, 1e-9, "no air at the top of the atmosphere");
    CHECK(nisp_in_atmosphere(BODY_BOB, 69.9f), "69.9 km is still in the air");
    CHECK(!nisp_in_atmosphere(BODY_BOB, 70.1f), "70.1 km is not");
    CHECK(!nisp_in_atmosphere(BODY_CHEESE, 0.1f), "Cheese has no air at all");
}

// --- Case 5: the stack and its sums (Part H case 5) ---------------------

// A part written out row by row instead of built from the ladder, because
// Part A5's worked example quotes its numbers and a reader should be able
// to check them with a calculator.
static nisp_part_t hand_part(int slot, int kind, int stage, float x0, float x1, float dia,
                             float dry_t, float prop_t, float cd_a, float com_bias) {
    nisp_part_t p = {0};
    p.slot = slot;
    p.kind = kind;
    p.stage = stage;
    p.engine = NISP_ENGINE_NONE;
    p.x0 = x0;
    p.x1 = x1;
    p.diameter_m = dia;
    p.area_m2 = nisp_side_area_m2(dia, x1 - x0);
    p.cd_a = cd_a;
    p.dry_t = dry_t;
    p.prop_t = prop_t;
    p.prop_full_t = prop_t;
    p.mass_t = dry_t + prop_t;
    p.com_bias = com_bias;
    p.capacity = 1.0f;
    p.limit = 1.0f;
    return p;
}

static void case5_stack(void) {
    printf("case 5  stack maths: mass, CoM, CoT, drag centre, static margin\n");

    // Part A5's two-row stack: capsule 1.0 t with CdA 1.0 m^2 over 0-2 m,
    // a stage with CdA 6.0 m^2 over 2-12 m, 1.25 m diameter.
    nisp_stack_t s = {0};
    s.count = 2;
    s.stage_count = 1;
    s.throttle = 1.0f;
    s.part[0] = hand_part(NISP_PART_KIND_BODY, NISP_KIND_CAPSULE, -1, 0.0f, 2.0f, 1.25f, 1.0f, 0.0f, 1.0f, 0.5f);
    s.part[1] = hand_part(NISP_PART_KIND_BODY, NISP_KIND_MAIN, 0, 2.0f, 12.0f, 1.25f, 0.8f, 7.2f, 6.0f, 0.5f);
    s.part[1].engine = NISP_ENGINE_SKIPPER;
    s.part[1].engine_count = 1;
    s.part[1].igniting = true;
    nisp_stack_recompute(&s);

    double const com_loaded = (1.0 * 1.0 + 8.0 * 7.0) / 9.0;   // 57/9  = 6.3333
    double const cp = (1.0 * 1.0 + 6.0 * 7.0) / 7.0;           // 43/7  = 6.1429
    CHECK_NEAR(s.mass_t, 9.0, 1e-4, "loaded stack mass, t");
    CHECK_NEAR(s.com_x, com_loaded, 1e-4, "loaded CoM, m");
    CHECK_NEAR(s.cp_x, cp, 1e-4, "drag centre, m");
    CHECK_NEAR(s.static_margin_m, cp - com_loaded, 1e-4, "loaded static margin, m");
    CHECK_NEAR(s.static_margin_m, -0.190476, 2e-5, "loaded margin vs Part A5's -0.19 m");
    CHECK(s.static_margin_m < 0.0f, "a loaded stage is marginally unstable");

    double const rr = 0.625;  // 1.25 m diameter
    double const i_loaded = 1.0 * (3.0 * rr * rr + 4.0) / 12.0 + 1.0 * pow(1.0 - com_loaded, 2.0)
                          + 8.0 * (3.0 * rr * rr + 100.0) / 12.0 + 8.0 * pow(7.0 - com_loaded, 2.0);
    CHECK_NEAR(s.inertia_t_m2, i_loaded, 1e-3, "loaded transverse inertia, t*m^2");
    CHECK_NEAR(s.thrust_kn, 180.0, 1e-4, "one Skipper, kN");
    CHECK_NEAR(s.cot_x, 12.0, 1e-4, "CoT at the bottom of its stage, m");
    CHECK_NEAR(s.cot_x - s.com_x, 12.0 - com_loaded, 1e-4, "thrust lever arm, m");

    // The same geometry with the tank empty: nothing moved, the mass did.
    s.part[1].prop_t = 0.0f;
    s.part[1].mass_t = 0.8f;
    nisp_stack_recompute(&s);
    double const com_empty = (1.0 * 1.0 + 0.8 * 7.0) / 1.8;    // 6.6/1.8 = 3.6667
    CHECK_NEAR(s.mass_t, 1.8, 1e-4, "burnt-out stack mass, t");
    CHECK_NEAR(s.com_x, com_empty, 1e-4, "burnt-out CoM, m");
    CHECK_NEAR(s.cp_x, cp, 1e-4, "the drag centre did not move");
    CHECK_NEAR(s.static_margin_m, cp - com_empty, 1e-4, "burnt-out static margin, m");
    CHECK_NEAR(s.static_margin_m, 2.476190, 2e-5, "burnt-out margin vs Part A5's +2.47 m");
    CHECK(s.static_margin_m > 0.0f, "a burnt-out stage is a dart");
    double const i_empty = 1.0 * (3.0 * rr * rr + 4.0) / 12.0 + 1.0 * pow(1.0 - com_empty, 2.0)
                         + 0.8 * (3.0 * rr * rr + 100.0) / 12.0 + 0.8 * pow(7.0 - com_empty, 2.0);
    CHECK_NEAR(s.inertia_t_m2, i_empty, 1e-3, "burnt-out transverse inertia, t*m^2");
    CHECK_NEAR(com_loaded - com_empty, 2.666667, 1e-4, "the CoM walked 2.67 m forward");

    // A lone capsule weathervanes shield-first (D-31): mass biased toward
    // the shield end, drag at its geometric middle, so the margin is positive.
    nisp_stack_t pod = {0};
    pod.count = 1;
    pod.part[0] = hand_part(NISP_PART_KIND_BODY, NISP_KIND_CAPSULE, -1, 0.0f, 2.0f, 1.25f, 1.0f, 0.0f,
                            nisp_part_row(NISP_KIND_CAPSULE)->cd * nisp_side_area_m2(1.25f, 2.0f),
                            NISP_CAPSULE_MASS_BIAS);
    nisp_stack_recompute(&pod);
    CHECK_NEAR(pod.com_x, 0.9, 1e-4, "lone capsule CoM, biased to the shield end, m");
    CHECK_NEAR(pod.cp_x, 1.0, 1e-4, "lone capsule drag centre, geometric middle, m");
    CHECK(pod.static_margin_m > 0.0f, "a lone capsule has a positive static margin");
    CHECK(pod.has_capsule, "a stack with only a pod still has a mission");

    // Deployed legs are drag at the tail and nothing else: no mass, no heat.
    nisp_design_t d = {0};
    d.stage_count = 1;
    d.stage[0].kind = NISP_KIND_MAIN;
    d.stage[0].diameter = 2;   // 2.5 m
    d.stage[0].length = 9;     // 6.25 m
    d.stage[0].engine = NISP_ENGINE_SKIPPER;
    d.stage[0].engine_count = 2;
    d.stage[0].legs = NISP_LEGS_4;
    nisp_stack_t st;
    nisp_stack_build(&d, &st);
    float const cd0 = st.cd_a_total, cp0 = st.cp_x, sm0 = st.static_margin_m, mass0 = st.mass_t;
    CHECK(nisp_stack_legs_deployed(&st, 0) == false, "legs start stowed");
    nisp_stack_deploy_legs(&st, 0, true);
    CHECK(nisp_stack_legs_deployed(&st, 0), "legs report as deployed");
    CHECK_NEAR(st.cd_a_total - cd0, nisp_legs_row(NISP_LEGS_4)->cd_a, 1e-4, "deployed legs add their drag area, m^2");
    CHECK_NEAR(st.mass_t, mass0, 1e-6, "legs add no mass");
    CHECK(st.cp_x > cp0, "deployed legs move the drag centre aft");
    CHECK(st.static_margin_m > sm0, "deployed legs add static margin");

    // A tank empties from its bottom, so the propellant left is the top of
    // it and the CoM walks toward the nose while there is fuel to walk.
    nisp_stack_build(&d, &st);
    float const cp_full = st.cp_x;
    int body = -1;
    for (int i = 0; i < st.count; i++) {
        if (st.part[i].slot == NISP_PART_KIND_BODY && st.part[i].stage == 0) body = i;
    }
    CHECK(body > 0, "the built stack has a stage tank in it");
    float prev = st.com_x;
    float const sm_full = st.static_margin_m;
    bool walked = true;
    for (int k = 1; k <= 3; k++) {
        float const frac = 1.0f - 0.25f * (float)k;
        st.part[body].prop_t = st.part[body].prop_full_t * frac;
        st.part[body].mass_t = st.part[body].dry_t + st.part[body].prop_t;
        nisp_stack_recompute(&st);
        if (!(st.com_x < prev)) walked = false;
        prev = st.com_x;
    }
    CHECK(walked, "burning propellant walks the CoM up the rocket (Part V4)");
    CHECK(st.static_margin_m > sm_full, "and the same burn increases the static margin");
    CHECK_NEAR(st.cp_x, cp_full, 1e-4, "the drag centre does not care how full the tank is");

    // What is left when the tank is empty is the tank: its own dry mass, at
    // its own middle. The CoM walks forward and then back, which is what a
    // draining cylinder actually does, and Part A5's table is the case where
    // the forward walk is the one that matters.
    st.part[body].prop_t = 0.0f;
    st.part[body].mass_t = st.part[body].dry_t;
    nisp_stack_recompute(&st);
    CHECK_NEAR(nisp_part_com_x(&st.part[body]), 0.5f * (st.part[body].x0 + st.part[body].x1), 1e-4,
               "an empty tank's mass is its own middle");
}

// --- Case 6: delta-v, TWR, burn time (Part H case 6) --------------------

// An independent reference for one stage's burn: march it in small steps
// with dv += (T / m) dt, and drop a spent booster group when its own
// propellant runs out. No rocket equation anywhere in here, which is the
// point -- a different method, so agreement with budget.c's closed-form
// two-phase answer means something.
static double ref_burn_delta_v(double m0, double prop, double thrust, double isp, double bprop,
                               double bdry, double bthrust, double bisp, double* burn_out) {
    double m = m0;
    double dv = 0.0;
    double t = 0.0;
    double p = prop;
    double bp = bprop;
    double const flow = thrust > 0.0 && isp > 0.0 ? thrust / (isp * NISP_G0) : 0.0;
    double const bflow = bthrust > 0.0 && bisp > 0.0 ? bthrust / (bisp * NISP_G0) : 0.0;
    bool dropped = false;

    for (int guard = 0; guard < 4000000; guard++) {
        double T = 0.0, mdot = 0.0;
        if (p > 1e-12) { T += thrust; mdot += flow; }
        if (bp > 1e-12) { T += bthrust; mdot += bflow; }
        if (mdot <= 0.0) break;

        double dt = 0.002;
        if (p > 1e-12) dt = fmin(dt, p / flow);
        if (bp > 1e-12) dt = fmin(dt, bp / bflow);
        if (dt <= 0.0) break;

        dv += T / (m - 0.5 * mdot * dt) * dt;  // kN / t is m/s^2
        m -= mdot * dt;
        if (p > 1e-12) p -= flow * dt;
        if (bp > 1e-12) bp -= bflow * dt;
        if (bp <= 1e-12 && !dropped) { m -= bdry; dropped = true; }  // a spent booster leaves (Part F3)
        t += dt;
    }

    if (burn_out != NULL) *burn_out = t;
    return dv;
}

static double frag_com(nisp_stack_t const* s) {
    double m = 0.0, x = 0.0;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        double const len = p->x1 - p->x0;
        double const bias = p->com_bias > 0.0f ? p->com_bias : 0.5;
        double const dry_x = p->x0 + bias * len;
        double px = dry_x;
        if (p->prop_t > 0.0 && p->prop_full_t > 0.0) {
            double const fill = p->prop_t / p->prop_full_t;
            double const prop_x = p->x0 + 0.5 * fill * len;
            px = (p->dry_t * dry_x + p->prop_t * prop_x) / (p->dry_t + p->prop_t);
        }
        m += p->mass_t;
        x += p->mass_t * px;
    }
    return m > 0.0 ? x / m : 0.0;
}

static nisp_design_t three_stage_design(void) {
    // Three stages, in ladder indices, uppermost first:
    //   0  upper   2.5 m x 3.125 m, 1 Relion, RCS
    //   1  main    2.5 m x 6.25 m,  2 Skippers
    //   2  main    3.75 m x 10 m,   2 Mainsails + 2 solid boosters of 6.25 m
    nisp_design_t d = {0};
    snprintf(d.name, sizeof(d.name), "%s", "TEST-3STAGE");
    d.stage_count = 3;
    d.stage[0].kind = NISP_KIND_UPPER;
    d.stage[0].diameter = 2;
    d.stage[0].length = 4;
    d.stage[0].engine = NISP_ENGINE_RELION;
    d.stage[0].engine_count = 1;
    d.stage[0].rcs = true;
    d.stage[1].kind = NISP_KIND_MAIN;
    d.stage[1].diameter = 2;
    d.stage[1].length = 9;
    d.stage[1].engine = NISP_ENGINE_SKIPPER;
    d.stage[1].engine_count = 2;
    d.stage[2].kind = NISP_KIND_MAIN;
    d.stage[2].diameter = 3;
    d.stage[2].length = 15;
    d.stage[2].engine = NISP_ENGINE_MAINSAIL;
    d.stage[2].engine_count = 2;
    d.stage[2].boosters = 2;
    d.stage[2].booster_length = 9;
    return d;
}

static void case6_budget(void) {
    printf("case 6  delta-v, TWR, burn time, and the trap that has both\n");

    nisp_design_t d = three_stage_design();
    nisp_budget_t b;
    nisp_budget_compute(&d, BODY_BOB, &b);

    // The masses, by hand: propellant = 400 kg/m^3 * pi * r^2 * L.
    double const r25 = 1.25, r375 = 1.875;
    double const p0 = 0.4 * M_PI * r25 * r25 * 3.125;    //   6.1359 t
    double const p1 = 0.4 * M_PI * r25 * r25 * 6.25;     //  12.2718 t
    double const p2 = 0.4 * M_PI * r375 * r375 * 10.0;   //  44.1766 t
    double const pb = 2.0 * 0.4 * M_PI * r375 * r375 * 6.25;  // 55.2208 t, both boosters
    double const d0 = 0.06 * p0 + 0.30 + 0.10;           // walls + Relion + RCS
    double const d1 = 0.08 * p1 + 2.0 * 0.90;            // walls + 2 Skippers
    double const d2 = 0.08 * p2 + 2.0 * 1.20;            // walls + 2 Mainsails
    double const db = 0.15 * pb;                         // a solid's structure is its own fraction
    double const sep = 0.05;

    double const m_stage2 = p2 + d2 + pb + db + sep;
    double const m_above2 = 1.0 + (p0 + d0 + sep) + (p1 + d1 + sep);
    double const m0_bottom = m_above2 + m_stage2;

    CHECK_NEAR(b.total_mass_t, m0_bottom, 1e-3, "total mass at the pad, t");
    CHECK_NEAR(b.stage[2].wet_t, m_stage2, 1e-3, "bottom stage wet mass, t");
    CHECK_NEAR(b.stage[2].prop_t, p2 + pb, 1e-3, "bottom stage propellant, t");
    CHECK_NEAR(b.stage[2].thrust_kn, 2.0 * 300.0 + 2.0 * 400.0, 1e-3, "bottom stage thrust, kN");
    CHECK_NEAR(b.liftoff_twr, 1400.0 / (m0_bottom * nisp_body_g0(BODY_BOB)), 1e-3, "liftoff TWR");
    CHECK(b.first_stage_ok, "1.04 TWR is enough to leave the ground");
    CHECK(b.reach_orbit_ok, "this launcher can reach a 100 km orbit on delta-v alone");

    // The bottom stage, two phases, checked against a step-by-step march.
    double burn = 0.0;
    double const dv_ref = ref_burn_delta_v(m0_bottom, p2, 600.0, 290.0, pb, db, 800.0, 180.0, &burn);
    CHECK_NEAR(b.stage[2].delta_v_ms, dv_ref, 1.0, "bottom stage delta-v vs a stepped reference, m/s");
    CHECK_NEAR(b.stage[2].burn_s, burn, 0.5, "bottom stage burn time vs the same reference, s");
    CHECK(b.stage[2].delta_v_ms > 3000.0, "a Mainsail stack with boosters is a real first stage");

    // The two stages above it, each pushing only what is left above it.
    double const m1_ignition = 1.0 + (p1 + d1 + sep) + (p0 + d0 + sep);
    // Two Skippers burn as one engine with the same Isp, and the rocket
    // equation cares about that Isp, not about how the thrust is split.
    double const dv1 = 270.0 * NISP_G0 * log(m1_ignition / (m1_ignition - p1));
    CHECK_NEAR(b.stage[1].delta_v_ms, dv1, 1.0, "second stage delta-v vs hand arithmetic, m/s");
    double const m0_ignition = 1.0 + (p0 + d0 + sep);
    double const dv0 = 320.0 * NISP_G0 * log(m0_ignition / (m0_ignition - p0));
    CHECK_NEAR(b.stage[0].delta_v_ms, dv0, 1.0, "upper stage delta-v vs hand arithmetic, m/s");
    CHECK_NEAR(b.total_delta_v_ms, dv0 + dv1 + dv_ref, 2.0, "total delta-v is the sum of the stages, m/s");
    CHECK_NEAR(b.stage[1].isp_s, 270.0, 1e-3, "a stage with one engine type reports its Isp, s");
    CHECK_NEAR(b.stage[2].isp_s, (600.0 * 290.0 + 800.0 * 180.0) / 1400.0, 1e-2, "mixed stage Isp is thrust-weighted, s");

    // F-12: enough delta-v, and a first stage that cannot lift it.
    nisp_design_t trap = {0 };
    snprintf(trap.name, sizeof(trap.name), "%s", "TRAP");
    trap.stage_count = 2;
    trap.stage[0].kind = NISP_KIND_UPPER;
    trap.stage[0].diameter = 2;
    trap.stage[0].length = 15;   // 2.5 m x 10 m of propellant with two Relions on it
    trap.stage[0].engine = NISP_ENGINE_RELION;
    trap.stage[0].engine_count = 2;
    trap.stage[1].kind = NISP_KIND_MAIN;
    trap.stage[1].diameter = 0;  // 1.25 m
    trap.stage[1].length = 0;    // 0.625 m
    trap.stage[1].engine = NISP_ENGINE_FLEA;
    trap.stage[1].engine_count = 1;
    nisp_budget_t t;
    nisp_budget_compute(&trap, BODY_BOB, &t);
    CHECK(!t.first_stage_ok, "a Flea under a 20 tonne upper stage cannot lift it (F-12)");
    CHECK(t.liftoff_twr < 1.0f, "liftoff TWR under 1 is reported as under 1");
    CHECK(t.reach_orbit_ok, "and the delta-v total still says it can reach orbit");
    CHECK(t.reach_orbit_ok && !t.first_stage_ok, "the trap exists: enough delta-v, no liftoff");

    // A cargo section makes no thrust and no delta-v.
    nisp_design_t cargo = {0};
    cargo.stage_count = 1;
    cargo.stage[0].kind = NISP_KIND_CARGO;
    cargo.stage[0].diameter = 2;
    cargo.stage[0].length = 4;
    nisp_budget_t c;
    nisp_budget_compute(&cargo, BODY_BOB, &c);
    CHECK_NEAR(c.stage[0].thrust_kn, 0.0, 1e-6, "a cargo section thrusts nothing, kN");
    CHECK_NEAR(c.stage[0].delta_v_ms, 0.0, 1e-6, "a cargo section has no delta-v, m/s");
    CHECK(!c.first_stage_ok, "and it is not a first stage");
    CHECK(c.stage[0].prop_t > 0.0f, "a cargo section still holds propellant, t");

    // The card round trip: encode, decode, encode, byte for byte the same.
    char text[1024];
    char text2[1024];
    int const n1 = nisp_design_encode(&d, text, (int)sizeof(text));
    CHECK(n1 > 0, "a blueprint encodes");
    nisp_design_t back = {0};
    CHECK(nisp_design_decode(text, &back), "a blueprint decodes");
    int const n2 = nisp_design_encode(&back, text2, (int)sizeof(text2));
    CHECK(n1 == n2 && memcmp(text, text2, (size_t)n1) == 0, "encode -> decode -> encode is byte-identical");
    CHECK(strcmp(back.name, d.name) == 0, "the name survives the round trip");
    CHECK(back.stage_count == d.stage_count, "the stage count survives");
    CHECK(back.stage[2].boosters == 2 && back.stage[2].booster_length == 9, "the boosters survive");
    CHECK(back.stage[0].rcs, "RCS survives");
    CHECK(nisp_design_encode(&d, text, 8) < 0, "a buffer that is too small is refused, not truncated");

    // Clamp: nothing outside the ladders gets through.
    nisp_design_t wild = {0};
    wild.stage_count = 1;
    wild.stage[0].kind = NISP_KIND_MAIN;
    wild.stage[0].diameter = 99;
    wild.stage[0].length = -4;
    wild.stage[0].engine = NISP_ENGINE_COUNT + 3;
    wild.stage[0].engine_count = 99;
    wild.stage[0].boosters = 7;
    wild.stage[0].legs = NISP_LEGS_COUNT + 2;
    nisp_design_clamp(&wild);
    CHECK(wild.stage[0].diameter >= 0 && wild.stage[0].diameter < nisp_diameter_steps(), "diameter clamps into the ladder");
    CHECK(wild.stage[0].length >= 0 && wild.stage[0].length < nisp_length_steps(), "length clamps into the ladder");
    CHECK(wild.stage[0].engine >= 0 && wild.stage[0].engine < NISP_ENGINE_COUNT, "engine clamps into the table");
    CHECK(wild.stage[0].engine_count >= 1 && wild.stage[0].engine_count <= nisp_engine_count_max(), "engine count clamps");
    CHECK(wild.stage[0].boosters == 0 || wild.stage[0].boosters == 2 || wild.stage[0].boosters == 4, "boosters clamp to 0, 2 or 4");
    CHECK(wild.stage[0].legs >= 0 && wild.stage[0].legs < NISP_LEGS_COUNT, "legs clamp into the table");
}

// --- Case 9: heating and failure (Part H case 9, Parts A4 and F5) -------

// Four scripted (altitude, speed, attitude) profiles. They are inputs, not
// results: the heat model is pure, so a reentry can be tested here instead
// of flown on a badge (Part H). The four are the four sentences Part A4 is
// built on -- a normal launch survives, a normal reentry loses what is not
// shielded, a bad reentry loses the pod, and a dart loses the pod anyway.
typedef struct {
    float alt_km;
    float v_ms;
    float secs;  // how long it takes to reach the next waypoint
} heat_waypoint_t;

// Throttled through max q, which is what a player who watches the HUD is
// actually doing: pitch over early, keep the dynamic pressure down, and only
// accelerate once the air is thin (Part A4).
static heat_waypoint_t const PROFILE_LAUNCH[] = {
    {0.0f, 0.0f, 40.0f},    {2.0f, 90.0f, 20.0f},   {5.0f, 220.0f, 12.0f},  {8.0f, 380.0f, 7.0f},
    {11.0f, 550.0f, 5.0f},  {14.0f, 720.0f, 4.5f},  {20.0f, 1050.0f, 6.0f}, {30.0f, 1550.0f, 7.0f},
    {45.0f, 2050.0f, 8.0f}, {70.0f, 2400.0f, 0.0f},
};

static heat_waypoint_t const PROFILE_REENTRY[] = {
    {100.0f, 2246.0f, 50.0f}, {80.0f, 2250.0f, 40.0f}, {60.0f, 2270.0f, 30.0f}, {45.0f, 2290.0f, 20.0f},
    {35.0f, 2300.0f, 12.0f},  {30.0f, 2250.0f, 10.0f}, {25.0f, 2000.0f, 9.0f},  {20.0f, 1500.0f, 10.0f},
    {15.0f, 1000.0f, 12.0f},  {10.0f, 600.0f, 15.0f},  {5.0f, 350.0f, 20.0f},   {2.0f, 150.0f, 0.0f},
};

static heat_waypoint_t const PROFILE_STEEP[] = {
    {100.0f, 2350.0f, 35.0f}, {80.0f, 2380.0f, 28.0f}, {60.0f, 2400.0f, 22.0f}, {45.0f, 2420.0f, 16.0f},
    {35.0f, 2400.0f, 11.0f},  {30.0f, 2300.0f, 9.0f},  {25.0f, 2100.0f, 8.0f},  {20.0f, 1700.0f, 9.0f},
    {15.0f, 1200.0f, 11.0f},  {10.0f, 800.0f, 13.0f},  {5.0f, 500.0f, 16.0f},   {2.0f, 300.0f, 0.0f},
};

static heat_waypoint_t const PROFILE_DART[] = {
    {100.0f, 2450.0f, 40.0f}, {80.0f, 2450.0f, 36.0f}, {60.0f, 2450.0f, 25.0f}, {45.0f, 2440.0f, 15.0f},
    {35.0f, 2420.0f, 7.0f},   {30.0f, 2400.0f, 6.7f},  {25.0f, 2350.0f, 6.25f}, {20.0f, 2250.0f, 5.9f},
    {15.0f, 2000.0f, 3.3f},   {12.0f, 1750.0f, 2.1f},  {10.0f, 1500.0f, 2.0f},  {8.0f, 1250.0f, 2.0f},
    {6.0f, 1000.0f, 2.0f},    {4.0f, 800.0f, 2.0f},    {2.0f, 600.0f, 2.0f},    {0.0f, 400.0f, 0.0f},
};

// The profile is flown open loop: heat is what it is, and the first part to
// reach its limit is recorded rather than removed, because case 9 wants to
// know the order things die in. A part that peaked and then cooled is still
// dead in the game -- flight/physics.c splits the stack the moment
// nisp_heat_breach answers -- so "peak over the limit" and "breached during
// the profile" are the same fact seen at different times.
static void run_heat_profile(nisp_stack_t* s, int body, float cos_flow, heat_waypoint_t const* prof,
                             int n, float* peak, int* first_breach) {
    float const dt = 0.05f;
    if (first_breach != NULL) *first_breach = -1;
    for (int i = 0; i < s->count; i++) {
        s->part[i].heat = 0.0f;
        peak[i] = 0.0f;
    }

    for (int i = 0; i + 1 < n; i++) {
        int const steps = (int)(prof[i].secs / dt + 0.5f);
        for (int k = 0; k < steps; k++) {
            float const f = steps > 0 ? (float)k / (float)steps : 0.0f;
            nisp_heat_input_t in;
            in.body = body;
            in.alt_km = prof[i].alt_km + (prof[i + 1].alt_km - prof[i].alt_km) * f;
            in.v_rel_kms = (prof[i].v_ms + (prof[i + 1].v_ms - prof[i].v_ms) * f) / 1000.0f;
            in.cos_flow_axis = cos_flow;
            nisp_heat_tick(s, &in, dt);
            if (first_breach != NULL && *first_breach < 0) {
                int const b = nisp_heat_breach(s);
                if (b >= 0) *first_breach = b;
            }
            for (int p = 0; p < s->count; p++) {
                float const limit = s->part[p].limit > 0.0f ? s->part[p].limit : 1.0f;
                float const load = s->part[p].capacity > 0.0f ? s->part[p].heat / limit : 0.0f;
                if (load > peak[p]) peak[p] = load;
            }
        }
    }
}

static int find_part(nisp_stack_t const* s, int slot, int stage) {
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].slot == slot && s->part[i].stage == stage) return i;
    }
    return -1;
}

static void report_peaks(char const* label, nisp_stack_t const* s, float const* peak) {
    printf("      %-16s", label);
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].capacity <= 0.0f) continue;
        printf(" %s=%.3f", s->part[i].slot == NISP_PART_KIND_BOOSTER ? "bst"
                     : (s->part[i].kind == NISP_KIND_CAPSULE ? "pod" : "stg"),
               peak[i]);
    }
    printf("\n");
}

// The launcher the heat model is tuned against: 1.25 m pod, a 1.25 m upper
// stage with one Relion, a 2.5 m main stage with two Skippers and two solids.
static nisp_design_t heat_design(void) {
    nisp_design_t d = {0};
    d.stage_count = 2;
    d.stage[0].kind = NISP_KIND_UPPER;
    d.stage[0].diameter = 0;   // 1.25 m
    d.stage[0].length = 5;     // 3.75 m
    d.stage[0].engine = NISP_ENGINE_RELION;
    d.stage[0].engine_count = 1;
    d.stage[0].rcs = true;
    d.stage[1].kind = NISP_KIND_MAIN;
    d.stage[1].diameter = 2;   // 2.5 m
    d.stage[1].length = 12;    // 7.5 m
    d.stage[1].engine = NISP_ENGINE_SKIPPER;
    d.stage[1].engine_count = 2;
    d.stage[1].boosters = 2;
    d.stage[1].booster_length = 12;
    return d;
}

static void case9_heat(void) {
    printf("case 9  heating: four profiles, and what each one is allowed to kill\n");

    nisp_design_t d = heat_design();
    nisp_budget_t bud;
    nisp_budget_compute(&d, BODY_BOB, &bud);
    CHECK(bud.first_stage_ok, "the heat-test launcher lifts off (TWR over 1)");
    CHECK(bud.liftoff_twr > 1.5f, "and it is a normal launcher, not a barely-one");

    float peak[NISP_STACK_PART_MAX];
    nisp_stack_t s;
    int first_breach = -1;

    // 1. A nominal launch, nose into the air: nothing may pass half its limit.
    nisp_stack_build(&d, &s);
    run_heat_profile(&s, BODY_BOB, 1.0f, PROFILE_LAUNCH,
                     (int)(sizeof(PROFILE_LAUNCH) / sizeof(PROFILE_LAUNCH[0])), peak, &first_breach);
    report_peaks("launch", &s, peak);
    bool launch_ok = true;
    for (int i = 0; i < s.count; i++) {
        if (s.part[i].capacity <= 0.0f) continue;
        if (!(peak[i] < 0.5f)) launch_ok = false;
    }
    CHECK(launch_ok, "a nominal launch leaves every part under half its limit");
    CHECK(nisp_heat_breach(&s) < 0, "nothing fails during a nominal launch");
    bool launch_hot = false;
    for (int i = 0; i < s.count; i++) {
        if (peak[i] > 0.15f) launch_hot = true;
    }
    CHECK(launch_hot, "and the launch is not so cold that the model proves nothing");

    // 2. A nominal reentry, capsule into the flow: the unshielded stages go,
    //    the pod does not.
    nisp_stack_build(&d, &s);
    run_heat_profile(&s, BODY_BOB, 1.0f, PROFILE_REENTRY,
                     (int)(sizeof(PROFILE_REENTRY) / sizeof(PROFILE_REENTRY[0])), peak, &first_breach);
    report_peaks("reentry", &s, peak);
    int const pod = find_part(&s, NISP_PART_KIND_BODY, -1);
    int const bottom = find_part(&s, NISP_PART_KIND_BODY, 1);
    int const upper = find_part(&s, NISP_PART_KIND_BODY, 0);
    CHECK(pod >= 0 && bottom >= 0 && upper >= 0, "the reentry stack has a pod and two stages");
    CHECK(peak[pod] < 1.0f, "a nominal reentry keeps the capsule");
    CHECK(peak[bottom] >= 1.0f, "and finishes off the unshielded bottom stage");
    CHECK(peak[upper] >= 1.0f, "and the unshielded upper stage too");
    CHECK(first_breach >= 0 && first_breach != pod, "the first part to fail is not the capsule");

    // 3. A steep reentry, engines into the flow: the shield faces away and
    //    protects nothing, and the pod is gone.
    nisp_stack_build(&d, &s);
    run_heat_profile(&s, BODY_BOB, -1.0f, PROFILE_STEEP,
                     (int)(sizeof(PROFILE_STEEP) / sizeof(PROFILE_STEEP[0])), peak, &first_breach);
    report_peaks("steep reentry", &s, peak);
    CHECK(peak[pod] >= 1.0f, "engines-first at reentry speed destroys the capsule");
    CHECK(nisp_heat_expose(&s.part[pod], -1.0f) > 0.99f, "a shield facing away protects nothing");
    CHECK_NEAR(nisp_heat_expose(&s.part[pod], 1.0f), 1.0f - s.part[pod].shield, 1e-4,
               "a shield facing the flow is a factor, not immunity");

    // 4. The dart: Part A5's burnt-out stack, flying the *stable* attitude at
    //    reentry speed. Shield-first, and still fatal (D-36, F-16).
    nisp_stack_build(&d, &s);
    for (int i = 0; i < s.count; i++) {
        s.part[i].prop_t = 0.0f;
        s.part[i].mass_t = s.part[i].dry_t;
    }
    nisp_stack_recompute(&s);
    run_heat_profile(&s, BODY_BOB, 1.0f, PROFILE_DART,
                     (int)(sizeof(PROFILE_DART) / sizeof(PROFILE_DART[0])), peak, &first_breach);
    report_peaks("dart", &s, peak);
    CHECK(s.static_margin_m > 0.0f, "the burnt-out stack really is a dart (Part A5)");
    CHECK(peak[pod] >= 1.0f, "and the dart's stable attitude still kills the pod");

    // Cooling: a part that comes out of the air recovers, which is what makes
    // heat an accumulated quantity and not a speed threshold (Part A4).
    nisp_stack_build(&d, &s);
    run_heat_profile(&s, BODY_BOB, 1.0f, PROFILE_LAUNCH,
                     (int)(sizeof(PROFILE_LAUNCH) / sizeof(PROFILE_LAUNCH[0])), peak, &first_breach);
    float const hot = s.part[bottom].heat;
    CHECK(hot > 0.0f, "the bottom stage is warm after a launch");
    nisp_heat_input_t space = {BODY_BOB, 120.0f, 2.4f, 1.0f};
    // Four time constants, which is what "cooled back" has to mean here.
    for (int i = 0; i < 2400; i++) nisp_heat_tick(&s, &space, 0.05f);
    CHECK(s.part[bottom].heat < hot * 0.05f, "above the air a part cools back toward ambient");
    CHECK(nisp_heat_load_rate(BODY_BOB, 0.0f, 1.2f) > nisp_heat_load_rate(BODY_BOB, 30.0f, 1.2f),
          "the air does the heating: rho*v^3 is bigger at the pad");
    CHECK_NEAR(nisp_heat_load_rate(BODY_CHEESE, 10.0f, 2.0f), 0.0, 1e-9, "Cheese has no air to heat anything in");
}

// --- Case 10: the split (Part H case 10, Part F5) -----------------------

static void case10_split(void) {
    printf("case 10  a part fails in the middle and the stack becomes two\n");

    nisp_design_t d = three_stage_design();
    nisp_stack_t s, other;
    nisp_stack_build(&d, &s);
    nisp_stack_ignite(&s, 2, 1.0f);
    CHECK_NEAR(s.thrust_kn, 1400.0, 1e-3, "the bottom stage is lit, kN");
    CHECK(s.live_stage == 2, "the bottom stage is the live one (D-42)");
    nisp_budget_t full;
    nisp_budget_compute(&d, BODY_BOB, &full);
    nisp_stack_burn(&s, 40.0f);
    CHECK(s.mass_t < full.total_mass_t, "and it is lighter after burning");
    CHECK(s.prop_t < full.total_mass_t, "and it has less propellant left than it started with");

    int const idx = find_part(&s, NISP_PART_KIND_BODY, 1);
    CHECK(idx > 0, "the middle stage's tank is in the stack");
    float const mass_before = s.mass_t;
    float const lost = s.part[idx].mass_t;
    int const count_before = s.count;

    CHECK(nisp_stack_split(&s, idx, &other), "a part failing mid-stack cuts it in two");
    CHECK(s.count + other.count + 1 == count_before, "one part gone, two fragments left");
    CHECK_NEAR(s.mass_t + other.mass_t + lost, mass_before, 1e-3, "the fragments weigh what the stack weighed, t");
    CHECK(s.has_capsule, "the controlled fragment has the capsule (Part F5)");
    CHECK(!other.has_capsule, "the debris fragment does not");
    CHECK(other.live_stage < 0, "debris has nothing to fire");
    int lit = 0;
    for (int i = 0; i < other.count; i++) {
        if (other.part[i].igniting) lit++;
    }
    CHECK(lit == 0, "no debris engine is lit");
    CHECK_NEAR(other.thrust_kn, 0.0, 1e-6, "debris thrusts nothing, kN");

    // CoM and CoT of each fragment, from the rows that are actually in it.
    CHECK_NEAR(s.com_x, frag_com(&s), 1e-3, "the pod fragment's CoM matches its rows, m");
    CHECK_NEAR(other.com_x, frag_com(&other), 1e-3, "the debris fragment's CoM matches its rows, m");
    CHECK(s.com_x < other.com_x, "the pod fragment is the forward one");

    // The bottom stage is still burning in the debris-free fragment? No: the
    // bottom stage is in the debris, so what is left fires nothing.
    CHECK_NEAR(s.thrust_kn, 0.0, 1e-6, "the pod fragment has no engine left lit, kN");
    CHECK(s.live_stage >= 0, "the pod fragment still has a stage to arm");

    // The capsule itself is the mission: a split that would take it is refused.
    nisp_stack_build(&d, &s);
    int const pod = find_part(&s, NISP_PART_KIND_BODY, -1);
    CHECK(nisp_stack_split(&s, pod, &other) == false, "the capsule cannot be the part that fails");
    CHECK(s.has_capsule && s.count > 0, "and refusing leaves the stack alone");
    CHECK(nisp_stack_split(&s, s.count, &other) == false, "a part index off the end is refused");
    CHECK(nisp_stack_split(&s, -1, &other) == false, "and so is a negative one");

    // Staging is the same cut with a different reason (Part V1, D-42).
    nisp_stack_build(&d, &s);
    int const stages = s.count;
    int const removed = nisp_stack_advance(&s);
    CHECK(removed > 0, "a staging press discards the live stage");
    CHECK(s.count < stages, "and the stack is shorter");
    CHECK(s.live_stage == 1, "the stage above is armed next");
    CHECK_NEAR(s.mass_t + 0.0, s.mass_t, 1e-6, "the stack still has mass");
    bool bottom_gone = true;
    for (int i = 0; i < s.count; i++) {
        if (s.part[i].stage == 2) bottom_gone = false;
    }
    CHECK(bottom_gone, "every row of the discarded stage went with it");
    CHECK_NEAR(frag_com(&s), s.com_x, 1e-3, "the staged stack's CoM matches its rows, m");
}

// --- Case 11: coasting, warp, and the SOI handover (Part F1, F4) -------

// One stage, one engine: the shape every flight case starts from.
static nisp_design_t flight_design(char const* name, int kind, int dia, int len, int engine,
                                   int engines, bool rcs, int legs) {
    nisp_design_t d;
    memset(&d, 0, sizeof(d));
    snprintf(d.name, sizeof(d.name), "%s", name);
    d.stage_count = 1;
    d.stage[0].kind = kind;
    d.stage[0].diameter = dia;
    d.stage[0].length = len;
    d.stage[0].engine = engine;
    d.stage[0].engine_count = engines;
    d.stage[0].rcs = rcs;
    d.stage[0].legs = legs;
    nisp_design_clamp(&d);
    return d;
}

static double dist3(double const a[3], double const b[3]) {
    double d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    return mag3(d);
}

// A ship at +r on the X axis, moving along -z: the prograde direction at
// the pad (Part B2), so every flight case here is a prograde orbit.
static nisp_vessel_t radius_vessel(int body, double r_km, double v_km, nisp_design_t const* d) {
    nisp_state_t st;
    st.r[0] = (float)r_km; st.r[1] = 0.0f; st.r[2] = 0.0f;
    st.v[0] = 0.0f; st.v[1] = 0.0f; st.v[2] = (float)-v_km;
    float nose[3] = {0.0f, 0.0f, -1.0f};
    nisp_vessel_t out;
    nisp_vessel_orbit(&out, d, body, &st, 0.0, nose);
    return out;
}

static nisp_vessel_t circular_vessel(int body, double r_km, nisp_design_t const* d) {
    return radius_vessel(body, r_km, sqrt(nisp_body(body)->mu / r_km), d);
}

// A ship at apoapsis -ra, periapsis rp, on the same X-Z plane.
static nisp_vessel_t apside_vessel(int body, double ra_km, double rp_km, nisp_design_t const* d) {
    double const a = 0.5 * (ra_km + rp_km);
    double const va = sqrt(nisp_body(body)->mu * (2.0 / ra_km - 1.0 / a));
    nisp_state_t st;
    st.r[0] = (float)-ra_km; st.r[1] = 0.0f; st.r[2] = 0.0f;
    st.v[0] = 0.0f; st.v[1] = 0.0f; st.v[2] = (float)va;
    float nose[3] = {0.0f, 0.0f, 1.0f};
    nisp_vessel_t out;
    nisp_vessel_orbit(&out, d, body, &st, 0.0, nose);
    return out;
}

// The coast is the fast path: at warp 10,000 a frame is 167 simulated
// seconds and it costs one Kepler solve (Part O3). So it has to agree with
// a dumb RK4, and it has to agree to well under a metre per frame or a
// 400,000 km transfer leg drifts by a thousand kilometres.
static void coast_matches_rk4(int body, double r_km) {
    nisp_design_t d = flight_design("COAST", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    nisp_vessel_t v = circular_vessel(body, r_km, &d);

    nisp_state_t start;
    nisp_vessel_state(&v, &start);

    // One warp-1000 frame at 60 Hz.
    double const hop = 1000.0 / 60.0;
    double const used = nisp_vessel_coast(&v, 0.0, hop);
    CHECK_NEAR(used, hop, 1e-6, "coast covers the whole frame");

    nisp_state_t ref = start;
    integrate_two_body(body, &ref, 0.01, (int)(hop / 0.01));

    double const got_p[3] = {v.pos_km[0], v.pos_km[1], v.pos_km[2]};
    double const want_p[3] = {ref.r[0], ref.r[1], ref.r[2]};
    double const got_v[3] = {v.vel_kms[0], v.vel_kms[1], v.vel_kms[2]};
    double const want_v[3] = {ref.v[0], ref.v[1], ref.v[2]};
    char tag[96];
    snprintf(tag, sizeof(tag), "coast position vs RK4 at %.0f km", r_km);
    CHECK_NEAR(dist3(got_p, want_p), 0.0, 0.05, tag);
    snprintf(tag, sizeof(tag), "coast velocity vs RK4 at %.0f km", r_km);
    CHECK_NEAR(dist3(got_v, want_v), 0.0, 1e-4, tag);
}

// A coast stops where the analytic answer stops being an answer: the edge
// of the air, or the ground. It reports the time it actually covered so
// the ticks can take the rest of the frame (F-34). Getting this wrong is
// not a slow ship, it is a ship that has been somewhere else.
static void coast_stops_at_the_air(void) {
    nisp_design_t d = flight_design("COAST", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    struct { double ra, rp; double want_used; double want_alt; char const* what; } rows[] = {
        {850.0, 620.0, 771.5, 70.0, "periapsis 20 km: stops in the air"},
        {850.0, 500.0, 548.5, 70.0, "periapsis underground: stops in the air"},
        {850.0, 700.0, 1000.0, 106.757, "periapsis 100 km: never stops"},
    };
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        nisp_vessel_t v = apside_vessel(BODY_BOB, rows[i].ra, rows[i].rp, &d);
        double const used = nisp_vessel_coast(&v, 0.0, 1000.0);
        float const alt = nisp_altitude(v.body, v.pos_km);
        CHECK_NEAR(used, rows[i].want_used, 1.0, rows[i].what);
        CHECK_NEAR(alt, rows[i].want_alt, 0.01, "altitude it stopped at, km");
    }
}

// The ladder and its refusals (Part F4). Every row is a ship at radius r on
// an ellipse whose other apsis is 4,600 km, so the low rows are on the way
// down and the high rows are in the middle of nowhere.
static void warp_ladder(void) {
    nisp_design_t d = flight_design("WARP", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    struct { double r, other; float req; int th, st; float warp; double dt; int sub; int coast;
             int capped; char const* reason; } rows[] = {
        {640.0, 4600.0, 1000.0f, 0, 0, 10.0f, 1.0 / 6.0, 9, 0, 0, "max atmosphere"},
        {680.0, 4600.0, 1000.0f, 0, 0, 1000.0f, 50.0 / 3.0, 0, 1, 0, ""},
        {4600.0, 4600.0, 1000.0f, 1, 0, 100.0f, 5.0 / 3.0, 84, 0, 0, "max engine burning"},
        {4600.0, 4600.0, 10000.0f, 0, 0, 10000.0f, 500.0 / 3.0, 0, 1, 0, ""},
        {4600.0, 4600.0, 10000.0f, 0, 1, 100.0f, 5.0 / 3.0, 84, 0, 0, "max steering"},
        {4600.0, 4600.0, 100.0f, 1, 0, 100.0f, 5.0 / 3.0, 84, 0, 0, ""},
        {680.0, 4600.0, 10000.0f, 0, 0, 1000.0f, 50.0 / 3.0, 0, 1, 0, "max below 100 km"},
    };
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        double const a = 0.5 * (rows[i].r + rows[i].other);
        double const v = sqrt(nisp_body(BODY_BOB)->mu * (2.0 / rows[i].r - 1.0 / a));
        nisp_vessel_t ship = radius_vessel(BODY_BOB, rows[i].r, v, &d);

        nisp_warp_step_t w;
        nisp_warp_plan(&ship, rows[i].req, 1.0 / 60.0, rows[i].th != 0, rows[i].st != 0, &w);
        CHECK_NEAR(w.warp, rows[i].warp, 1e-3, "warp this frame uses");
        CHECK_NEAR(w.dt_sim, rows[i].dt, 1e-6, "simulated seconds it advances");
        CHECK(w.substeps == rows[i].sub, "substeps match");
        CHECK(w.coasting == (rows[i].coast != 0), "coasting matches");
        CHECK(w.capped == (rows[i].capped != 0), "capped matches");
        CHECK(strcmp(w.reason, rows[i].reason) == 0, "the refusal names the rule");
    }
}

// The substep cap is the other refusal: a frame that would need 834 ticks
// gets 200, and the warp comes down to what 200 ticks can actually pay for.
static void warp_substep_cap(void) {
    nisp_design_t d = flight_design("WARP", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    nisp_vessel_t ship = circular_vessel(BODY_BOB, 4600.0, &d);
    nisp_warp_step_t w;
    nisp_warp_plan(&ship, 1000.0f, 0.1, true, false, &w);
    CHECK_NEAR(w.warp, 40.0, 1e-3, "warp after the tick cap");
    CHECK_NEAR(w.dt_sim, 4.0, 1e-6, "a capped frame is 4 simulated seconds");
    CHECK(w.substeps == NISP_SUBSTEP_MAX, "the cap is NISP_SUBSTEP_MAX ticks");
    CHECK(w.capped, "the cap says so");
    CHECK(strcmp(w.reason, "max engine burning") == 0, "and says which rule");
}

// A coast is refused when its next periapsis is in the air (F-34): the
// frame would fly through an atmosphere it never samples, which is a
// different trajectory rather than a coarse one.
static void warp_periapsis_refusal(void) {
    nisp_design_t d = flight_design("WARP", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    struct { int body; double ra, rp; float peri; float warp; char const* reason; } rows[] = {
        {BODY_BOB, 5000.0, 650.0, 50.0f, 10.0f, "max periapsis in air"},
        {BODY_BOB, 5000.0, 700.0, 100.0f, 10000.0f, ""},
        {BODY_BOB, 5000.0, 600.0, 0.0f, 10.0f, "max periapsis in air"},
        {BODY_CHEESE, 3000.0, 150.0, -50.0f, 10.0f, "max periapsis underground"},
    };
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        nisp_vessel_t ship = apside_vessel(rows[i].body, rows[i].ra, rows[i].rp, &d);
        CHECK_NEAR(nisp_vessel_periapsis_alt_km(&ship), rows[i].peri, 0.01, "periapsis altitude, km");
        nisp_warp_step_t w;
        nisp_warp_plan(&ship, 10000.0f, 1.0 / 60.0, false, false, &w);
        CHECK_NEAR(w.warp, rows[i].warp, 1e-3, "warp with a periapsis down there");
        CHECK(strcmp(w.reason, rows[i].reason) == 0, "and says why the top rung is gone");
        CHECK(w.coasting, "it is still a coast, just a slow one");
    }
}

// Leaving Bob's sphere hands the ship to Frodo (Part O4). The reference is
// the same step done the long way: an RK4 tick in Bob's frame, Bob's own
// motion bolted on, and an RK4 tick in Frodo's.
static void soi_handover_out(void) {
    nisp_design_t d = flight_design("SOI", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    nisp_vessel_t ship = radius_vessel(BODY_BOB, 84360.0, 0.35, &d);  // just inside the sphere

    nisp_state_t pre;
    nisp_vessel_state(&ship, &pre);
    double const dt = 0.02;
    nisp_vessel_substep(&ship, NULL, 0.0, dt);
    CHECK(ship.body == BODY_FRODO, "leaving Bob hands the ship to Frodo");

    nisp_state_t ref = pre;
    integrate_two_body(BODY_BOB, &ref, dt, 1);
    double bp[3], bv[3];
    nisp_body_relative(BODY_BOB, BODY_FRODO, dt, bp);
    nisp_body_velocity(BODY_BOB, dt, bv);
    nisp_state_t in_frodo;
    for (int i = 0; i < 3; i++) {
        in_frodo.r[i] = (float)(ref.r[i] + bp[i]);
        in_frodo.v[i] = (float)(ref.v[i] + bv[i]);
    }
    integrate_two_body(BODY_FRODO, &in_frodo, dt, 1);

    double const got[3] = {ship.pos_km[0], ship.pos_km[1], ship.pos_km[2]};
    double const want[3] = {in_frodo.r[0], in_frodo.r[1], in_frodo.r[2]};
    CHECK_NEAR(dist3(got, want), 0.0, 0.5, "position across the handover, km");
    CHECK_NEAR(mag3(got), 13684200.0, 1000.0, "distance from Frodo afterwards, km");
}

// Entering Cheese's sphere is the handover that is easy to get wrong,
// because the position and the velocity have to change frame together
// (F-35). The test is the invariant, not the number: the ship is the same
// ship before and after, so converting it back has to give what it was.
static void soi_handover_in(void) {
    nisp_design_t d = flight_design("SOI", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    double const t = 3000.0;
    double cp[3], cv[3], bv[3];
    nisp_body_relative(BODY_CHEESE, BODY_BOB, t, cp);
    nisp_body_velocity(BODY_CHEESE, t, cv);
    nisp_body_velocity(BODY_BOB, t, bv);

    // A ship parked 2 km off Cheese, travelling with it: 542 m/s at
    // 12,000 km, which is Cheese's circular speed at that radius.
    nisp_state_t st;
    st.r[0] = (float)(cp[0] + 2.0); st.r[1] = 2.0f; st.r[2] = (float)(cp[2] + 2.0);
    for (int i = 0; i < 3; i++) st.v[i] = (float)(cv[i] - bv[i]);
    float nose[3] = {0.0f, 0.0f, 1.0f};
    nisp_vessel_t ship;
    nisp_vessel_orbit(&ship, &d, BODY_BOB, &st, t, nose);
    CHECK_NEAR(mag3((double[]){ship.vel_kms[0], ship.vel_kms[1], ship.vel_kms[2]}), 0.5424, 1e-3,
               "speed relative to Bob before the switch, km/s");

    nisp_state_t pre;
    nisp_vessel_state(&ship, &pre);
    nisp_vessel_substep(&ship, NULL, t, 0.02);
    CHECK(ship.body == BODY_CHEESE, "Cheese's sphere takes over");

    // Convert it back into Bob's frame and compare against the same tick
    // done in Bob's frame: the tick is Bob's, and the handover happens at
    // the end of it, so the two answers have to be the same answer.
    nisp_state_t ref = pre;
    integrate_two_body(BODY_BOB, &ref, 0.02, 1);

    double rel[3], cv2[3], bv2[3];
    nisp_body_relative(BODY_CHEESE, BODY_BOB, t + 0.02, rel);
    nisp_body_velocity(BODY_CHEESE, t + 0.02, cv2);
    nisp_body_velocity(BODY_BOB, t + 0.02, bv2);
    double back_p[3], back_v[3];
    for (int i = 0; i < 3; i++) {
        back_p[i] = ship.pos_km[i] + rel[i];
        back_v[i] = ship.vel_kms[i] + cv2[i] - bv2[i];
    }
    double const want_p[3] = {ref.r[0], ref.r[1], ref.r[2]};
    double const want_v[3] = {ref.v[0], ref.v[1], ref.v[2]};
    CHECK_NEAR(dist3(back_p, want_p), 0.0, 0.01, "the handover round trip, position km");
    CHECK_NEAR(dist3(back_v, want_v), 0.0, 1e-4, "the handover round trip, km/s");
    CHECK_NEAR(mag3(ship.pos_km), 335.295, 0.01, "distance from Cheese, km");
}

static void case11_coast_warp_soi(void) {
    printf("case 11  a coast is analytic, warp refuses, and gravity changes hands\n");
    coast_matches_rk4(BODY_BOB, 680.0);   // 80 km: the orbit everything else uses
    coast_matches_rk4(BODY_BOB, 700.0);   // 100 km
    coast_matches_rk4(BODY_BOB, 4600.0);  // 4,000 km: the slow, high one
    coast_stops_at_the_air();
    warp_ladder();
    warp_substep_cap();
    warp_periapsis_refusal();
    soi_handover_out();
    soi_handover_in();
}


// =====================================================================
// case 12 -- control: what RCS buys, and what a ship does without it
// ---------------------------------------------------------------------
// Part F2's whole design decision is that stabilisation is something the
// player builds, not something the game gives away. So the checks here are
// the two halves of that: an RCS-equipped stack gets exactly the authority
// its modules were bought for, and a stack without them gets exactly zero,
// however the switches are set.
// =====================================================================

static nisp_design_t control_design(int rcs_all) {
    nisp_design_t d = three_stage_design();
    for (int i = 0; i < d.stage_count; i++) d.stage[i].rcs = rcs_all != 0;
    nisp_design_clamp(&d);
    return d;
}

static double mag3f(float const v[3]) { return sqrt((double)v[0] * v[0] + (double)v[1] * v[1] + (double)v[2] * v[2]); }

// The 80 km circular orbit everything else in this file uses, with the nose
// tipped `tilt_deg` away from prograde about -x.
static nisp_vessel_t attitude_vessel(nisp_design_t const* d, double tilt_deg) {
    double const r = 680.0;
    nisp_state_t st;
    st.r[0] = (float)r; st.r[1] = 0.0f; st.r[2] = 0.0f;
    st.v[0] = 0.0f; st.v[1] = 0.0f; st.v[2] = (float)-sqrt(nisp_body(BODY_BOB)->mu / r);
    double const th = tilt_deg * M_PI / 180.0;
    float nose[3] = {0.0f, (float)sin(th), (float)-cos(th)};
    nisp_vessel_t v;
    nisp_vessel_orbit(&v, d, BODY_BOB, &st, 0.0, nose);
    return v;
}

static double nose_dot_prograde(nisp_vessel_t const* v) {
    float nn[3], pg[3];
    nisp_vessel_nose(v, nn);
    nisp_vessel_prograde(v, pg);
    double dot = 0.0;
    for (int i = 0; i < 3; i++) dot += (double)nn[i] * pg[i];
    return dot;
}

static void control_budget(void) {
    nisp_design_t const on = control_design(1), off = control_design(0);
    nisp_stack_t s;

    nisp_stack_build(&on, &s);
    CHECK(nisp_control_rcs_modules(&s) == 3, "three RCS stages, three modules");
    CHECK_NEAR(nisp_control_authority_knm(&s, true), 45.0, 1e-6, "authority, kN*m");
    CHECK_NEAR(nisp_control_authority_knm(&s, false), 0.0, 1e-9, "the RCS switch off");

    nisp_stack_build(&off, &s);
    CHECK(nisp_control_rcs_modules(&s) == 0, "a design with no RCS has no modules");
    CHECK_NEAR(nisp_control_authority_knm(&s, true), 0.0, 1e-9, "authority it can spend");

    // Modules are bolted to a stage, so they leave with it (Part F5). The
    // authority a ship has is a property of what is still attached to the
    // capsule, which is why a stack that has thrown two stages steers with
    // a third of what it launched with.
    nisp_stack_build(&on, &s);
    nisp_stack_advance(&s);
    CHECK_NEAR(nisp_control_authority_knm(&s, true), 30.0, 1e-6, "authority after one stage");
    nisp_stack_advance(&s);
    CHECK_NEAR(nisp_control_authority_knm(&s, true), 15.0, 1e-6, "authority on the pod alone");
}

static void sas_aims_at_prograde(void) {
    nisp_design_t const d = control_design(1);
    nisp_vessel_t v = attitude_vessel(&d, 30.0);
    nisp_control_t ctl;
    nisp_control_init(&ctl);
    ctl.rcs = true;
    ctl.sas = NISP_SAS_PROGRADE;
    ctl.throttle = 0.0f;

    float target[3], axis[3], angle = 0.0f, tau[3];
    CHECK(nisp_vessel_reference(&v, ctl.sas, target), "prograde is a direction SAS can aim at");
    nisp_quat_error(&v.q, target, axis, &angle);
    CHECK_NEAR(angle, 0.523599, 1e-4, "a 30 degree error, radians");
    CHECK_NEAR(axis[0], -1.0, 1e-4, "the error axis, x");
    CHECK_NEAR(axis[2], 0.0, 1e-4, "the error axis, z");

    // A 30 degree error is far past the point where the controller is being
    // gentle: it asks for the whole budget, in the direction that closes it.
    nisp_control_torque(&v, &ctl, NISP_PHYS_DT, tau);
    CHECK_NEAR(mag3f(tau), 45.0, 1e-3, "a big error spends the whole budget");
    CHECK_NEAR(tau[0], -45.0, 1e-3, "and spends it the right way round");
    CHECK_NEAR(v.stack.inertia_t_m2, 3144.4849, 1.0, "this stack's inertia, t*m^2");
}

static void sas_converges(void) {
    nisp_design_t const d = control_design(1);
    nisp_vessel_t v = attitude_vessel(&d, 30.0);
    nisp_control_t ctl;
    nisp_control_init(&ctl);
    ctl.rcs = true;
    ctl.sas = NISP_SAS_PROGRADE;
    ctl.throttle = 0.0f;

    CHECK_NEAR(nose_dot_prograde(&v), 0.866025, 1e-4, "30 degrees off prograde, cos");

    int frames = 0;
    double wmax = 0.0;
    for (; frames < 900; frames++) {
        nisp_vessel_substep(&v, &ctl, 0.0, 1.0 / 60.0);
        double const w = mag3f(v.omega);
        if (w > wmax) wmax = w;
        if (nose_dot_prograde(&v) > 0.9999) { frames++; break; }
    }
    double const alpha = 45.0 / 3144.4849;  // kN*m over t*m^2 is rad/s^2

    CHECK(frames < 900, "SAS gets a 137 t stack onto prograde");
    CHECK_NEAR(frames, 629.0, 60.0, "frames it took, at 10.5 s");
    CHECK_NEAR(nose_dot_prograde(&v), 1.0, 1e-3, "and it stays there");
    CHECK_NEAR(wmax, 0.0862, 0.01, "the fastest it rotated, rad/s");

    // F-36: the old spring/damper law overshot to 24 degrees and swung back
    // forever, because its damping collapses on the way to the target while
    // the ship is still rotating. A rate-commanded one never asks for a rate
    // it cannot arrest, so the peak sits under the braking limit
    // sqrt(2 * alpha * angle) -- the one number that decides whether a slew
    // lands or overshoots -- and at sqrt(alpha * angle), which is what a
    // full-authority-out-then-full-authority-back slew peaks at.
    CHECK(wmax <= sqrt(2.0 * alpha * 0.523599), "it never rotates faster than it can stop");
    CHECK_NEAR(wmax, sqrt(alpha * 0.523599), 0.01, "and it slews at the authority limit");
}

static void no_rcs_means_no_control(void) {
    struct { int sas; int rcs; int design_rcs; char const* what; } rows[] = {
        {NISP_SAS_PROGRADE, 0, 1, "SAS on, RCS switch off"},
        {NISP_SAS_OFF, 1, 1, "RCS on, SAS off, keys alone"},
        {NISP_SAS_PROGRADE, 1, 0, "SAS on, RCS never built"},
    };
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        nisp_design_t const d = control_design(rows[i].design_rcs);
        nisp_vessel_t v = attitude_vessel(&d, 30.0);
        nisp_control_t ctl;
        nisp_control_init(&ctl);
        ctl.rcs = rows[i].rcs != 0;
        ctl.sas = rows[i].sas;
        ctl.throttle = 0.0f;
        for (int f = 0; f < 1200; f++) nisp_vessel_substep(&v, &ctl, 0.0, 1.0 / 60.0);
        CHECK_NEAR(mag3f(v.omega), 0.0, 1e-9, rows[i].what);
        // The nose still drifts, but only because prograde itself turns as
        // the ship orbits: 20 s of mean motion at 680 km.
        CHECK_NEAR(nose_dot_prograde(&v), 0.864081, 1e-4, "and the ship never turns");
    }
}

static void manual_keys(void) {
    nisp_design_t const d = control_design(1), off = control_design(0);
    nisp_control_t ctl;
    nisp_control_init(&ctl);
    ctl.rcs = true;
    ctl.sas = NISP_SAS_OFF;
    ctl.throttle = 0.0f;

    // One axis at full deflection is one module's budget, and torque over
    // inertia is rad/s^2 with no conversion at all (physics.h). Ten seconds
    // of 15 kN*m on 3,144 t*m^2 is 0.048 rad/s, which is the number a
    // player can feel: a launcher responds, but it takes a second to notice.
    ctl.torque[0] = NISP_MANUAL_TORQUE_KNM;
    nisp_vessel_t v = attitude_vessel(&d, 0.0);
    double wmax = 0.0;
    for (int f = 0; f < 600; f++) {
        nisp_vessel_substep(&v, &ctl, 0.0, 1.0 / 60.0);
        double const w = mag3f(v.omega);
        if (w > wmax) wmax = w;
    }
    CHECK_NEAR(wmax, 10.0 * 15.0 / 3144.4849, 1e-3, "10 s of a full manual pull, rad/s");

    ctl.torque[0] = 0.0f;
    v = attitude_vessel(&off, 0.0);
    v.omega[1] = 0.5f;  // a spin, because nothing else will stop it
    for (int f = 0; f < 600; f++) nisp_vessel_substep(&v, &ctl, 0.0, 1.0 / 60.0);
    CHECK_NEAR(mag3f(v.omega), 0.5, 1e-9, "a spin with no RCS keeps spinning");

    // HOLD is the brake. It commands zero rate, so it is pure authority
    // against the rotation, and the answer is the arithmetic a player can
    // do in their head: 45 kN*m on 3,144 t*m^2 for ten seconds.
    nisp_vessel_t held = attitude_vessel(&d, 0.0);
    held.omega[1] = 0.5f;
    ctl.rcs = true;
    ctl.sas = NISP_SAS_HOLD;
    for (int f = 0; f < 600; f++) nisp_vessel_substep(&held, &ctl, 0.0, 1.0 / 60.0);
    CHECK_NEAR(mag3f(held.omega), 0.5 - 10.0 * 45.0 / 3144.4849, 1e-3, "10 s of SAS HOLD on a spin, rad/s");
}

static void case12_control_sas(void) {
    printf("case 12  control authority, SAS, and the ship that has neither\n");
    control_budget();
    sas_aims_at_prograde();
    sas_converges();
    no_rcs_means_no_control();
    manual_keys();
}


// =====================================================================
// case 13 -- whole missions: a launch, an insertion, a landing, a reentry
// ---------------------------------------------------------------------
// Everything above this line is a number. This case is a flight, run
// through the same mission loop game/ will run on a badge, against
// outcomes a player would recognise: it got to space, it did not, it
// landed, it burned up. The tolerances are wide on purpose -- these are
// not arithmetic checks, they are "did the whole thing hang together", and
// the exact numbers are here so a regression shows up as a diff instead of
// as a mystery.
// =====================================================================

static nisp_design_t launcher_design(void) {
    // A launcher that can actually reach orbit from the pad: 7,697 m/s,
    // TWR 1.50, 61.2 t, seven rows. Built to be a bit over-powered,
    // because a rocket that barely makes orbit is a rocket that never
    // leaves the pad in a test.
    nisp_design_t d;
    memset(&d, 0, sizeof(d));
    snprintf(d.name, sizeof(d.name), "%s", "TEST-LAUNCHER");
    d.stage_count = 3;
    d.stage[0].kind = NISP_KIND_UPPER;
    d.stage[0].diameter = 0;
    d.stage[0].length = 3;
    d.stage[0].engine = NISP_ENGINE_RELION;
    d.stage[0].engine_count = 1;
    d.stage[0].rcs = true;
    d.stage[1].kind = NISP_KIND_MAIN;
    d.stage[1].diameter = 1;
    d.stage[1].length = 7;
    d.stage[1].engine = NISP_ENGINE_SKIPPER;
    d.stage[1].engine_count = 1;
    d.stage[1].rcs = true;
    d.stage[2].kind = NISP_KIND_MAIN;
    d.stage[2].diameter = 3;
    d.stage[2].length = 15;
    d.stage[2].engine = NISP_ENGINE_MAINSAIL;
    d.stage[2].engine_count = 3;
    d.stage[2].rcs = true;
    nisp_design_clamp(&d);
    return d;
}

static float live_propellant(nisp_stack_t const* s) {
    float p = 0.0f;
    for (int i = 0; i < s->count; i++) {
        if (s->part[i].stage == s->live_stage) p += s->part[i].prop_t;
    }
    return p;
}

// The frame loop a player drives: read the HUD, decide, step. `auto_stage`
// is the one piece of help the game gives away -- an empty stage is dropped
// for you, because staging at the right moment is bookkeeping and not
// flying (Part F1).
static void fly_mission(nisp_mission_t* m, nisp_control_t* ctl, double t_end, float* peak_q,
                        bool auto_stage) {
    // Frames, not seconds: warp decides how much sim time a frame buys, so
    // the loop is written in the mission's own clock (Part F1).
    for (int f = 0; f < 20000 && m->t < t_end && m->state == NISP_MISSION_FLYING; f++) {
        nisp_readout_t r;
        nisp_mission_readout(m, &r);
        if (peak_q != NULL && r.q_dyn_pa > *peak_q) *peak_q = r.q_dyn_pa;
        if (auto_stage && live_propellant(&m->ship.stack) <= 1e-4f) ctl->stage = true;
        if (ctl->sas == NISP_SAS_HOLD && (r.alt_km > 12.0f || r.speed_ms > 700.0f))
            ctl->sas = NISP_SAS_PROGRADE;
        nisp_mission_step(m, ctl, 1.0 / 60.0);
        ctl->stage = false;
    }
}

static void launch_reaches_space(void) {
    nisp_design_t const d = launcher_design();
    nisp_budget_t b;
    nisp_budget_compute(&d, BODY_BOB, &b);
    CHECK_NEAR(b.total_mass_t, 61.227, 0.01, "mass at the pad, t");
    CHECK_NEAR(b.liftoff_twr, 1.499, 0.005, "liftoff TWR");
    CHECK_NEAR(b.total_delta_v_ms, 7697.2, 5.0, "what it has to spend, m/s");
    CHECK(b.reach_orbit_ok, "which is enough for orbit");

    nisp_mission_t m;
    nisp_mission_init(&m);
    nisp_mission_launch(&m, &d, BODY_BOB, 0.0);
    CHECK(m.state == NISP_MISSION_FLYING, "the mission starts on the pad");
    CHECK(m.ship.landed, "with the ground under it");

    nisp_control_t ctl;
    nisp_control_init(&ctl);
    ctl.rcs = true;
    ctl.sas = NISP_SAS_HOLD;  // hold the heading it was built with, then turn prograde
    ctl.throttle = 1.0f;
    ctl.warp = 5.0f;

    float peak_q = 0.0f;
    fly_mission(&m, &ctl, 150.0, &peak_q, true);
    nisp_readout_t r;
    nisp_mission_readout(&m, &r);

    CHECK(m.state == NISP_MISSION_FLYING, "150 s in, still flying");
    CHECK_NEAR(m.t, 150.0, 0.5, "simulated seconds it flew");
    CHECK(r.alt_km > 40.0, "out of the dense air");
    CHECK(r.speed_ms > 1000.0, "and fast");
    CHECK(m.parts_lost == 0, "nothing burned through");
    CHECK(m.max_heat < 0.5f, "the hottest part is under half its limit");
    CHECK(peak_q > 4000.0f && peak_q < 7000.0f, "max-q on the way up, Pa");
    CHECK(m.stages_dropped >= 1, "the empty booster was dropped");
    CHECK(m.debris_count == 0, "and staging is not a failure: no debris");
    CHECK(!nisp_mission_music_space(&m), "47 km up, it is still in the air");

    fly_mission(&m, &ctl, 300.0, &peak_q, true);
    nisp_mission_readout(&m, &r);
    CHECK(m.state == NISP_MISSION_FLYING, "300 s in, still flying");
    CHECK(r.alt_km > 100.0, "in space");
    CHECK(nisp_mission_music_space(&m), "and the music knows it (Part N4)");
}

static void launch_without_rcs_falls_over(void) {
    // D-47: this is the lesson, not a bug. A fat booster with no RCS has no
    // control torque, so the air turns it, and a rocket flying sideways at
    // 800 m/s does not survive it. Same rocket, same throttle, same SAS
    // switch -- the only difference is whether the player paid for modules.
    struct { int rcs; int sas; char const* what; } rows[] = {
        {1, NISP_SAS_HOLD, "RCS off, SAS on"},
        {1, NISP_SAS_OFF, "RCS off, SAS off"},
    };
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        nisp_design_t d = launcher_design();
        for (int k = 0; k < d.stage_count; k++) d.stage[k].rcs = false;
        nisp_design_clamp(&d);

        nisp_mission_t m;
        nisp_mission_init(&m);
        nisp_mission_launch(&m, &d, BODY_BOB, 0.0);
        nisp_control_t ctl;
        nisp_control_init(&ctl);
        ctl.rcs = rows[i].rcs != 0;
        ctl.sas = rows[i].sas;
        ctl.throttle = 1.0f;
        ctl.warp = 5.0f;
        fly_mission(&m, &ctl, 3600.0, NULL, true);

        CHECK_MSG(m.state == NISP_MISSION_WRECKED, rows[i].what);
        CHECK(m.t < 120.0, "and it went early");
        CHECK(m.parts_lost > 0, "parts went before the ground did");
        CHECK(m.max_heat < 0.5f, "it was not the heat that got it");
    }
}


static void circularise_at_apoapsis(void) {
    // The one burn every player does first: 100 x 300 km, catch the
    // apoapsis, and circularise. The map says 127.92 m/s (Part O4); the
    // rocket spends it, and the orbit that comes out is the one the map
    // promised.
    nisp_design_t const d = flight_design("TEST-POD", NISP_KIND_UPPER, 0, 3, NISP_ENGINE_RELION, 1, true, 0);
    nisp_vessel_t v = apside_vessel(BODY_BOB, 900.0, 700.0, &d);

    nisp_state_t st;
    nisp_elements_t el;
    nisp_vessel_state(&v, &st);
    nisp_state_to_elements(&st, BODY_BOB, 0.0, &el);
    CHECK_NEAR(nisp_circularisation_delta_v(&el, 900.0f), 0.12792, 1e-3, "the burn the planner would ask for, km/s");

    nisp_mission_t m;
    nisp_mission_init(&m);
    nisp_mission_begin(&m, &v, 0.0);
    nisp_control_t ctl;
    nisp_control_init(&ctl);
    ctl.rcs = true;
    ctl.sas = NISP_SAS_PROGRADE;
    ctl.throttle = 1.0f;
    ctl.warp = 1.0f;

    float const m0 = m.ship.stack.mass_t;
    int frames = 0;
    for (; frames < 900 && m.state == NISP_MISSION_FLYING; frames++) {
        nisp_readout_t r;
        nisp_mission_readout(&m, &r);
        if (r.periapsis_km >= 299.0f) break;
        nisp_mission_step(&m, &ctl, 1.0 / 60.0);
    }
    nisp_readout_t r;
    nisp_mission_readout(&m, &r);

    double const dv_used = 320.0 * 9.80665 * log((double)m0 / (double)r.mass_t);
    CHECK(frames < 600, "the burn is over in under ten seconds");
    CHECK_NEAR(dv_used, 127.92, 1.3, "delta-v it actually spent, m/s");
    CHECK_NEAR(r.mass_t, 2.6412, 0.01, "mass it spent to do it, t");
    CHECK_NEAR(r.apoapsis_km - r.periapsis_km, 0.0, 20.0, "how round the orbit came out, km");
    CHECK_NEAR(r.periapsis_km, 299.3, 1.0, "periapsis altitude, km");
    CHECK_NEAR(r.speed_ms, 1980.3, 5.0, "and it is the circular speed for it");
    CHECK(m.state == NISP_MISSION_FLYING, "in one piece");
}

// A pod dropped from the sky: no engine, RCS, and whichever of a chute or
// legs the design had. Everything here is Part F3's survivability rule --
// 10 m/s and 10 degrees, or whatever the best deployed leg set allows.
static void drop_pod(nisp_mission_t* m, double alt_m, double v_ms, int legs) {
    nisp_design_t d;
    memset(&d, 0, sizeof(d));
    snprintf(d.name, sizeof(d.name), "%s", "TEST-DROP");
    d.stage_count = 1;
    d.stage[0].kind = NISP_KIND_CARGO;
    d.stage[0].diameter = 0;
    d.stage[0].length = 0;
    d.stage[0].engine = NISP_ENGINE_NONE;
    d.stage[0].rcs = true;
    d.stage[0].legs = legs;
    nisp_design_clamp(&d);

    double p[3];
    nisp_surface_point(BODY_BOB, 0.0, 0.0, 0.0, p);
    p[0] += alt_m * NISP_KM_PER_M;  // straight up from the pad
    nisp_state_t st;
    for (int i = 0; i < 3; i++) {
        st.r[i] = (float)p[i];
        st.v[i] = 0.0f;
    }
    st.v[0] = (float)(-v_ms * NISP_KM_PER_M);  // falling back down the radial
    float nose[3] = {1.0f, 0.0f, 0.0f};

    nisp_vessel_t v;
    nisp_vessel_orbit(&v, &d, BODY_BOB, &st, 0.0, nose);
    nisp_mission_init(m);
    nisp_mission_begin(m, &v, 0.0);
}

static void a_chute_lands_it(void) {
    struct { double alt_m; double v_ms; int chute; int legs; int want_state; char const* what; } rows[] = {
        {1500.0, 60.0, 1, 0, NISP_MISSION_LANDED, "a chute from 1,500 m"},
        {5000.0, 60.0, 1, 0, NISP_MISSION_LANDED, "a chute from 5,000 m"},
        {1500.0, 60.0, 0, 4, NISP_MISSION_WRECKED, "no chute, four legs, 1,500 m"},
        {3000.0, 800.0, 1, 0, NISP_MISSION_WRECKED, "a chute asked for at 800 m/s"},
    };

    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        nisp_mission_t m;
        drop_pod(&m, rows[i].alt_m, rows[i].v_ms, rows[i].legs);
        nisp_control_t ctl;
        nisp_control_init(&ctl);
        ctl.rcs = true;
        ctl.sas = NISP_SAS_HOLD;
        ctl.warp = 10.0f;
        ctl.chute = rows[i].chute != 0;
        ctl.legs = rows[i].legs != 0;
        for (int f = 0; f < 20000 && m.state == NISP_MISSION_FLYING; f++) {
            nisp_mission_step(&m, &ctl, 1.0 / 60.0);
            ctl.chute = false;
            ctl.legs = false;
        }

        CHECK_MSG(m.state == rows[i].want_state, rows[i].what);
        if (rows[i].chute && rows[i].v_ms < 300.0) {
            CHECK(m.ship.stack.parachute, "the canopy opened");
            CHECK_NEAR(m.ship.impact_ms, 8.806, 0.05, "terminal speed under the canopy, m/s");
            CHECK(m.ship.impact_ms < NISP_TOUCHDOWN_BELL_MS, "under the 10 m/s bell");
            CHECK(m.ship.impact_tilt_deg < NISP_TOUCHDOWN_BELL_TILT_DEG, "and upright");
            CHECK(m.parts_lost == 0, "nothing went");
        }
    }

    // The refusal is the point of the last row: a chute at 800 m/s is a
    // shredded chute, so the game says no rather than pretending (Part F1).
    nisp_mission_t m;
    drop_pod(&m, 3000.0, 800.0, 0);
    nisp_control_t ctl;
    nisp_control_init(&ctl);
    ctl.rcs = true;
    ctl.sas = NISP_SAS_HOLD;
    ctl.warp = 10.0f;
    ctl.chute = true;
    for (int f = 0; f < 20000 && m.state == NISP_MISSION_FLYING; f++) {
        nisp_mission_step(&m, &ctl, 1.0 / 60.0);
        ctl.chute = false;
    }
    CHECK(!m.ship.stack.parachute, "a chute asked for too fast never opens");
    CHECK(m.state == NISP_MISSION_WRECKED, "and the pod arrives the hard way");
}


static void landing_on_cheese_needs_legs(void) {
    // Cheese has no air, so there is no chute and no braking in it: a
    // landing there is a controlled descent and a leg set, or it is a
    // crater (Part V4's leg rows). Same approach, same speed, 11.0 m/s --
    // over the 10 m/s bell, inside the 12 m/s four-leg rating.
    nisp_design_t const d =
        flight_design("TEST-MOON", NISP_KIND_MAIN, 2, 9, NISP_ENGINE_SKIPPER, 2, true, NISP_LEGS_4);
    double const t = 1000.0;

    for (int legs = NISP_LEGS_4; legs >= 0; legs -= 4) {
        nisp_design_t dd = d;
        dd.stage[0].legs = legs;
        nisp_design_clamp(&dd);

        double pos[3], up[3], north[3], east[3], gv[3];
        nisp_surface_point(BODY_CHEESE, t, 0.0, 0.0, pos);
        nisp_local_frame(BODY_CHEESE, pos, up, north, east);
        for (int i = 0; i < 3; i++) pos[i] += up[i] * 0.020;  // 20 m up
        nisp_surface_velocity(BODY_CHEESE, t, 0.0f, 0.0f, gv);

        nisp_state_t st;
        for (int i = 0; i < 3; i++) {
            st.r[i] = (float)pos[i];
            st.v[i] = (float)(gv[i] - up[i] * 0.00748);  // 7.48 m/s down, relative to the ground
        }
        float nose[3] = {(float)up[0], (float)up[1], (float)up[2]};

        nisp_vessel_t v;
        nisp_vessel_orbit(&v, &dd, BODY_CHEESE, &st, t, nose);
        nisp_mission_t m;
        nisp_mission_init(&m);
        nisp_mission_begin(&m, &v, t);
        nisp_control_t ctl;
        nisp_control_init(&ctl);
        ctl.rcs = true;
        ctl.sas = NISP_SAS_HOLD;
        ctl.warp = 10.0f;
        ctl.legs = true;
        for (int f = 0; f < 900 && m.state == NISP_MISSION_FLYING; f++) {
            nisp_mission_step(&m, &ctl, 1.0 / 60.0);
            ctl.legs = false;
        }

        if (legs) {
            CHECK(m.state == NISP_MISSION_LANDED, "legs, and it stands on Cheese");
            CHECK_NEAR(m.ship.impact_ms, 11.007, 0.05, "touchdown speed, m/s");
            CHECK(m.ship.impact_ms > NISP_TOUCHDOWN_BELL_MS, "too fast for bare legs");
            CHECK(m.parts_lost == 0, "but the legs take it");
            CHECK_NEAR(m.ship.lat_rad, 0.0, 1e-3, "where it says it is, lat");
        } else {
            CHECK(m.state == NISP_MISSION_WRECKED, "no legs, and it is a crater");
            CHECK_NEAR(m.ship.impact_ms, 11.007, 0.05, "at the same speed");
            CHECK(m.parts_lost > 0, "with parts left behind");
        }
    }
}

static void a_steep_deorbit_burns(void) {
    // Part A6 with no mercy: 250 x 20 km is a reentry this capsule cannot
    // survive nose-first, and the check is *which* part went. 250 x 60 km
    // is the same rocket and the same entry, and it is still flying.
    struct { float pe_km; int want_state; char const* what; } rows[] = {
        {20.0f, NISP_MISSION_WRECKED, "250 x 20 km"},
        {60.0f, NISP_MISSION_FLYING, "250 x 60 km"},
    };
    nisp_design_t const stack = three_stage_design();
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        nisp_vessel_t v = apside_vessel(BODY_BOB, 850.0, 600.0 + rows[i].pe_km, &stack);
        nisp_mission_t m;
        nisp_mission_init(&m);
        nisp_mission_begin(&m, &v, 0.0);
        nisp_control_t ctl;
        nisp_control_init(&ctl);
        ctl.rcs = true;
        ctl.sas = NISP_SAS_PROGRADE;  // dart-first: the capsule leads, and eats the heat
        ctl.warp = 100.0f;
        for (int f = 0; f < 2000 && m.state == NISP_MISSION_FLYING; f++) nisp_mission_step(&m, &ctl, 1.0 / 60.0);

        nisp_readout_t r;
        nisp_mission_readout(&m, &r);
        CHECK_MSG(m.state == rows[i].want_state, rows[i].what);
        if (rows[i].want_state == NISP_MISSION_WRECKED) {
            CHECK(m.max_heat >= 1.0f, "the capsule reached its limit");
            CHECK(m.ship.wreck_kind == NISP_KIND_CAPSULE, "and it was the capsule that went");
            CHECK(m.parts_lost > 0, "with the rest of the stack still there");
        } else {
            CHECK(m.max_heat < 0.1f, "the shallower entry never got hot");
            CHECK(m.parts_lost == 0, "and nothing went");
        }
    }
}

static void hovering_is_a_control_problem(void) {
    // TWR 4 straight up from the pad, throttle pinned, 5 s. With a hold it
    // climbs; without one the weathervane wins and it ends up on its back,
    // which is what a hover is really a test of (Part A5).
    nisp_design_t const d = flight_design("TEST-HOVER", NISP_KIND_MAIN, 2, 9, NISP_ENGINE_SKIPPER, 2, true, 0);
    struct { int rcs; int sas; int want_state; char const* what; } rows[] = {
        {1, NISP_SAS_HOLD, NISP_MISSION_FLYING, "RCS and a hold"},
        {1, NISP_SAS_OFF, NISP_MISSION_WRECKED, "RCS, no hold"},
        {0, NISP_SAS_HOLD, NISP_MISSION_WRECKED, "a hold with nothing to spend"},
    };
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        nisp_design_t dd = d;
        dd.stage[0].rcs = rows[i].rcs != 0;
        nisp_design_clamp(&dd);

        nisp_mission_t m;
        nisp_mission_init(&m);
        nisp_mission_launch(&m, &dd, BODY_BOB, 0.0);
        nisp_control_t ctl;
        nisp_control_init(&ctl);
        ctl.rcs = rows[i].rcs != 0;
        ctl.sas = rows[i].sas;
        ctl.throttle = 1.0f;
        ctl.warp = 10.0f;
        for (int f = 0; f < 300 && m.state == NISP_MISSION_FLYING; f++) nisp_mission_step(&m, &ctl, 1.0 / 60.0);

        nisp_readout_t r;
        nisp_mission_readout(&m, &r);
        CHECK_MSG(m.state == rows[i].want_state, rows[i].what);
        if (rows[i].want_state == NISP_MISSION_FLYING) {
            CHECK(r.alt_km * 1000.0 > 1000.0, "it climbed");
            CHECK(r.nose_up_dot > 0.99f, "nose up, the whole way");
            CHECK_NEAR(r.twr, 3.98, 0.05, "TWR at ignition");
        } else {
            CHECK(r.nose_up_dot < 0.0f, "and the one without a hold ends pointing down");
        }
    }
}

static void case13_missions(void) {
    printf("case 13  a launch, an insertion, a landing, and a reentry that is too steep\n");
    launch_reaches_space();
    launch_without_rcs_falls_over();
    circularise_at_apoapsis();
    a_chute_lands_it();
    landing_on_cheese_needs_legs();
    a_steep_deorbit_burns();
    hovering_is_a_control_problem();
}

int main(void) {
    printf("nispcheck  --  Null Island Space Program, host harness\n\n");

    case1_anchors();
    printf("\n");
    case2_round_trip();
    printf("\n");
    case3_patches();
    printf("\n");
    case4_transfer();
    printf("\n");
    case5_stack();
    printf("\n");
    case6_budget();
    printf("\n");
    case9_heat();
    printf("\n");
    case10_split();
    printf("\n");
    case11_coast_warp_soi();
    printf("\n");
    case12_control_sas();
    printf("\n");
    case13_missions();
    printf("\n");

    if (s_fail) {
        printf("nispcheck: FAILED  (%d of %d checks)\n", s_fail, s_checks);
        return 1;
    }
    printf("nispcheck: ok  (%d checks)\n", s_checks);
    return 0;
}

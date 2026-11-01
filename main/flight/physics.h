#pragma once
// =====================================================================
//  Null Island Space Program  --  one rigid body, one tick (Part F1, V3)
// ---------------------------------------------------------------------
//  The state a flight is made of: where, how fast, which way it points,
//  how fast it is turning, and what it is made of. Everything else in
//  flight/ is a way of advancing or reading this.
//
//  ONE BODY (D-22). N parts, one position, one velocity, one attitude,
//  one angular velocity. Nothing flexes and nothing has its own state
//  except heat, which rocket/heat keeps per part because "which part
//  burns" is not a question a whole-body model can answer.
//
//  THE FRAME. Position and velocity are relative to the body whose sphere
//  of influence the ship is in (sim/patches.h), in that body's non-rotating
//  frame. The ground turns inside that frame, which is why the air's
//  velocity is `omega x r` and not zero (Part A2), and why a launch east
//  starts with 174.9 m/s it did not have to buy.
//
//  THE ATTITUDE is a unit quaternion, vehicle -> frame (common/quat.h).
//  Vehicle axes are fixed by Part V3: +z is the nose, and the stack axis
//  of rocket/stack -- measured from the nose toward the tail -- is local
//  -z. A part at stack coordinate x sits at local (0, 0, -x).
//
//  UNITS, and the one place they are mixed:
//
//      position, velocity   km, km/s        (sim/units.h, double)
//      attitude, rate       quaternion, rad/s (float)
//      thrust               kN              (rocket/parts.c, as an engineer writes it)
//      mass                 tonnes          (ditto)
//      drag, torque         N, N*m          (the air's units)
//      inertia              t*m^2
//
//  kN per tonne is m/s^2 and N*m per t*m^2 is rad/s^2 -- the 1,000s cancel
//  in pairs, which is why there is no unit conversion in the torque path at
//  all, and one factor of 1e6 in the drag path (N -> kN). That is written
//  out in physics.c at each of the three places it is used.
//
//  PURE: no engine, no BSP, no filesystem. Host-compilable (Part H), which
//  is what lets nispcheck case 13 fly a whole mission without a badge.
// =====================================================================

#include <stdbool.h>

#include "common/quat.h"
#include "flight/control.h"
#include "rocket/stack.h"
#include "sim/kepler.h"

// The physics tick (Part F1). Warp multiplies simulated time, never this.
#define NISP_PHYS_DT 0.02

// Touchdown without a deployed leg set: a capsule on its engine bell, or a
// stage on its tank bottom. Part A3's number for Bob -- 10 m/s, near
// vertical -- and a leg set is never worse than it (D-45).
#define NISP_TOUCHDOWN_BELL_MS 10.0f
#define NISP_TOUCHDOWN_BELL_TILT_DEG 10.0f

// The torque guards (F-30). Every velocity- or angle-dependent torque in
// this model is a spring or a damper, and both have a stability limit in dt:
// a restoring torque is stable while its natural frequency fits the tick, a
// damping torque is stable while it cannot reverse the rate it is fighting.
//
// The weathervane is the case that proves the point. A 1.0 t pod under a
// chute at 200 m/s is 8,000 kN*m of righting moment on 0.43 t*m^2 of
// inertia: a natural frequency of 136 rad/s, three ticks per period, and an
// answer that is a number rather than a trajectory. Capped at four rad/s of
// natural frequency the ship aligns itself with the air in about a second,
// which is what a capsule under a chute actually does.
#define NISP_TORQUE_OMEGA_MAX 4.0f  // rad/s of natural frequency per tick budget

typedef struct {
    // Where and how fast, relative to `body` (sim/patches.h).
    double pos_km[3];
    double vel_kms[3];
    int    body;

    // Which way and how it is turning.
    nisp_quat_t q;      // vehicle -> frame
    float       omega[3];  // rad/s, vehicle axes

    nisp_stack_t stack;

    bool  alive;   // the capsule is intact: this is the mission
    bool  landed;  // resting on the surface, co-rotating with it
    float lat_rad, lon_rad;  // where it landed

    // What ended it, for the mission log (Part F5, 9.3).
    float impact_ms;
    float impact_tilt_deg;
    int   wreck_kind;  // NISP_KIND_* of the part that went, -1 while alive
    int   wreck_stage; // its design stage, -1 for the capsule
} nisp_vessel_t;

// What a tick decided, if anything.
typedef enum {
    NISP_HIT_NONE = 0,
    NISP_HIT_SPLIT,   // a part burned through: `other` is the new fragment
    NISP_HIT_LOST,    // the capsule itself went, in the air (Part F5)
    NISP_HIT_LANDED,  // soft touchdown
    NISP_HIT_CRASHED  // hard touchdown
} nisp_hit_t;

// --- Putting a ship somewhere ------------------------------------------

// On the pad, pointing up, co-rotating with the ground. The launch case:
// the ship starts with the 174.9 m/s the pad has, because that is what a
// ship sitting on a turning planet has.
void nisp_vessel_pad(nisp_vessel_t* v, nisp_design_t const* d, int body, double t);

// Anywhere else on the ground, at `lat_rad` / `lon_rad`: what a craft that
// landed and was left there is when you come back for it. Same pose rule as
// the pad -- on the surface, nose up, carrying the ground's own velocity --
// and the same `landed` flag. The pad is this with the pad's coordinates.
void nisp_vessel_ground(nisp_vessel_t* v, nisp_design_t const* d, int body, double t, float lat_rad,
                        float lon_rad);

// On a conic, at `state`, with the nose along `nose_dir` (unit, frame
// axes). nispcheck's flight cases use this instead of flying a launch.
void nisp_vessel_orbit(nisp_vessel_t* v, nisp_design_t const* d, int body,
                       nisp_state_t const* state, double t, float const nose_dir[3]);

// Copy a vessel (the stack comes with it). Used to hand a fragment to the
// debris list.
void nisp_vessel_copy(nisp_vessel_t const* src, nisp_vessel_t* dst);

// --- Reading it ---------------------------------------------------------

void nisp_vessel_state(nisp_vessel_t const* v, nisp_state_t* out);
void nisp_vessel_nose(nisp_vessel_t const* v, float out[3]);  // unit, frame axes

// The periapsis of the conic this vessel is on, as an altitude above its
// current body: negative when that periapsis is below the surface, 0 when
// there is no conic to ask. flight/warp.c asks before it commits a frame,
// because a coast is analytic and a fast frame would otherwise dive through
// an atmosphere it never sampled (Part F4).
float nisp_vessel_periapsis_alt_km(nisp_vessel_t const* v);

// The direction a SAS mode wants the nose to point, in frame axes. False
// when the mode has no answer here (no orbit to define prograde on).
bool nisp_vessel_reference(nisp_vessel_t const* v, int mode, float out[3]);

// The torque the control system commands, kN*m, vehicle axes: SAS's PD or
// the manual keys, clamped to the RCS budget (Part F2). Implemented in
// flight/control.c, declared here because it is part of the force model.
void nisp_control_torque(nisp_vessel_t const* v, nisp_control_t const* ctl, float dt_s, float out[3]);

// The HUD's numbers, computed once per frame rather than per substep.
typedef struct {
    float alt_km;
    float speed_ms;    // orbital: relative to the body
    float v_rel_ms;    // through the air, or over the ground
    float q_dyn_pa;
    float rho;
    bool  has_air, in_air;

    float mass_t;
    float thrust_kn;
    float twr;         // thrust over weight, at this altitude
    float accel_ms2;   // what the throttle is actually doing

    float static_margin_m;  // the HUD's one aerodynamic number (Part A5)
    float heat_max;         // hottest part, as a fraction of its limit
    int   heat_part;

    float apoapsis_km, periapsis_km;  // altitudes; 0 when the conic has none
    float period_s;
    float incl_deg;
    bool  escaped;     // hyperbolic with respect to `body`

    // Seconds to the next apoapsis / periapsis passage (plan 6.1). Always
    // forward: nisp_time_to_apsis wraps into [0, period). -1 when the apsis
    // does not exist, which is apoapsis on a hyperbola.
    float time_to_ap_s, time_to_pe_s;

    float nose_up_dot;  // nose . local up: +1 straight up, -1 straight down
    float authority_knm;  // what the control system can spend right now
} nisp_readout_t;

void nisp_vessel_readout(nisp_vessel_t const* v, double t, nisp_readout_t* out);

// --- Advancing it -------------------------------------------------------

// One dt_phys tick: forces, Euler, Verlet, burn, heat, SOI. This is the
// only place the ship's state changes while it is thrusting or in air.
void nisp_vessel_substep(nisp_vessel_t* v, nisp_control_t const* ctl, double t, double dt_s);

// A coast: analytic two-body propagation along the current conic, with the
// attitude integrated and no forces. Warp 1000 costs the same as warp 1
// here (Part F1), which is the entire point of the split.
// Returns the simulated time it actually covered, which is less than
// `dt_s` when the analytic answer runs into the air or the ground; the
// caller spends the remainder in substeps.
double nisp_vessel_coast(nisp_vessel_t* v, double t, double dt_s);

// The reference directions a SAS mode can lock to, in frame axes.
// Prograde / retrograde are the velocity; normal is the orbit's angular
// momentum; radial is the local vertical.
void nisp_vessel_prograde(nisp_vessel_t const* v, float out[3]);
void nisp_vessel_normal(nisp_vessel_t const* v, float out[3]);
void nisp_vessel_radial_out(nisp_vessel_t const* v, float out[3]);

// --- The things the player asks for -------------------------------------

void nisp_vessel_stage(nisp_vessel_t* v);       // discard the live stage (Part F3)
void nisp_vessel_chute(nisp_vessel_t* v, bool deploy);
void nisp_vessel_legs(nisp_vessel_t* v);        // deploy the live stage's set

// Heat failure and ground contact, checked after each substep. Fills
// `other` when a fragment broke off. Part F5 and Part A3.
nisp_hit_t nisp_vessel_resolve(nisp_vessel_t* v, double t, nisp_vessel_t* other);

// The touchdown rating the fragment that is about to touch down has: the
// best deployed leg set attached to it, or the bell rating (D-45).
void nisp_vessel_touchdown_rating(nisp_vessel_t const* v, float* rated_v_ms, float* rated_tilt_deg);

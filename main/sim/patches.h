#pragma once
// =====================================================================
//  Null Island Space Program  --  patched conics
// ---------------------------------------------------------------------
//  The three-body problem has no closed form, so the game does what every
//  real mission planner does: at any moment exactly one body's gravity is
//  in charge, and the handover between them happens at the sphere of
//  influence (Part O4). Inside Bob's SOI the ship is on a conic about Bob;
//  cross the boundary and it is on a conic about Cheese, computed from the
//  state it had at the boundary. Each leg is exact; the seams are the
//  approximation, and they are the approximation Kerbal players already
//  understand.
//
//  ONE REFERENCE BODY PER LEG. A patch's state is relative to its own
//  body, which is what the map draws and what the planner reasons about.
//  Converting a leg to the inertial frame is a position and a velocity
//  subtraction against sim/ephemeris.c, and it happens only where a leg has
//  to be joined to the next one.
//
//  THE CHAIN IS CAPPED at NISP_PATCH_MAX (4) legs and by a time horizon.
//  Both caps are frame-budget decisions (Part G): a map that predicted the
//  ship's position for the next four centuries would be correct and useless.
//  When a cap binds, the last patch says so rather than pretending to end.
// =====================================================================

#include <stdbool.h>

#include "sim/kepler.h"

#define NISP_PATCH_MAX 4

// How many samples a SOI-entry scan takes along a leg. Coarse enough to be
// cheap, fine enough that a 2,400 km sphere is not jumped over between
// samples at the speeds a transfer reaches.
#define NISP_PATCH_SCAN 128

typedef struct {
    int    body;            // the body in charge for this leg
    nisp_elements_t el;     // the conic, relative to that body

    double t_enter;
    double t_exit;          // the horizon when the leg was cut, not ended

    // Why the leg ended.
    //   SOI   : it crossed out of `body`'s sphere of influence
    //   ENTRY : another body's sphere of influence swallowed it
    //   LIMIT : the horizon or the patch cap ran out
    enum { NISP_PATCH_END_LIMIT = 0, NISP_PATCH_END_SOI, NISP_PATCH_END_ENTRY } end;

    int    other;           // the body involved in `end`, or BODY_COUNT
    double r_end[3];        // km, relative to `body`
    double v_end[3];        // km/s, same frame
    bool   cut;             // the cap bound: the leg really continues
} nisp_patch_t;

// Predict forward from a state. `state` is relative to `body` at time `t`.
// Fills up to `max` patches and returns how many were written (>= 1).
int nisp_patch_predict(nisp_state_t const* state, int body, double t, double horizon_s,
                       nisp_patch_t* out, int max);

// The state at time `t` somewhere along a predicted chain. False when `t`
// is past the end of the chain -- the caller decides whether that means
// "clamp" or "re-predict".
bool nisp_patch_state(nisp_patch_t const* chain, int count, double t, nisp_state_t* out);

// Where a conic about `body` runs into another body's SOI, if it does before
// `t0 + horizon`. Returns false when nothing is hit.
//
// This one is a scan and a bisection rather than a closed form, and that is
// honest: "will I run into Cheese" is genuinely a three-body question, and
// a search along a two-body answer is how a map screen answers it without
// pretending to be a propagator.
bool nisp_soi_entry(nisp_elements_t const* el, double t0, double horizon_s,
                    int* hit_body, double* t_hit, double* r_hit);

// The radius at which a conic about `body` crosses `body`'s own SOI, as a
// true anomaly. Exact, because r(nu) = p / (1 + e cos nu) inverts in one
// line -- which is why leaving a sphere of influence is solved rather than
// searched for, while entering someone else's is searched.
bool nisp_soi_exit_true_anomaly(nisp_elements_t const* el, double* nu_out);

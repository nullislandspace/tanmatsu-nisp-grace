#pragma once
// =====================================================================
//  Null Island Space Program  --  what a design can actually do (Part V5)
// ---------------------------------------------------------------------
//  Three numbers per stage and one total, recomputed on every edit in
//  Engineering, because they are the entire feedback loop the builder has:
//
//      delta-v   Kostya's equation, stage by stage
//      TWR       thrust over weight, at the surface of the body it lifts from
//      burn      how long the stage fires
//
//  STAGES ARE COUNTED THE WAY A ROCKET BURNS: bottom first. The mass a
//  stage has to accelerate is everything above it plus itself -- capsule,
//  the stages above it, its own tank, its own boosters, every separator
//  still attached -- and the mass it ends with is everything above it after
//  it has been dropped (D-42).
//
//  BOOSTERS MAKE A STAGE TWO PHASES, and the arithmetic does them as two
//  phases: a solid booster burns with its stage and leaves when it is dry,
//  so the stage's burn is "everything firing" up to the moment one of them
//  runs out, then "what is left firing". For a constant-thrust, constant
//  mass-flow phase delta-v is (T / mdot) * ln(m0 / m1), which is Kostya's
//  equation with the Isp averaging folded out -- so a mixed stage is exact
//  rather than an average of two engines.
//
//  THE TRAP THIS FILE EXISTS TO CATCH (F-12): a stack with a fine total
//  delta-v and a first stage under 1.0 TWR is a rocket that never leaves the
//  ground. `first_stage_ok` is that number, and Engineering gives it a card
//  of its own.
//
//  WHAT THIS DOES NOT SAY: whether the design survives the air it flies
//  through. Part ratings are data and belong on a card; what the assembled
//  rocket does with them on a trajectory the player is flying is not (D-37).
//
//  PURE: no engine, no BSP, no filesystem. Host-compilable (Part H).
// =====================================================================

#include <stdbool.h>

#include "rocket/design.h"
#include "rocket/stack.h"

// The ideal impulsive cost of the pad to a 100 km circular orbit on Bob:
// 2,517 m/s to raise the apoapsis, 88 to circularise, before the 174.9 m/s
// the pad already has and before gravity and drag losses. nispcheck case 4
// is where this number comes from; the Engineering card is the one that
// quotes it (Part B4).
#define NISP_REACH_ORBIT_MS 2605.0f

typedef struct {
    int   stage;             // design stage index
    char  name[24];          // the part row's name, for the card
    char  engine[16];        // the engine it fires, or "(none)"

    float delta_v_ms;
    float twr;               // at the body's surface, at ignition of this stage
    float burn_s;
    float thrust_kn;
    float isp_s;

    float wet_t;             // this stage's own mass, full
    float dry_t;
    float prop_t;

    int   engine_count;
    int   boosters;
    int   legs;
    bool  rcs;
    bool  liftable;          // twr >= 1: this stage can lift the stack it sits on
} nisp_stage_budget_t;

typedef struct {
    int stage_count;
    nisp_stage_budget_t stage[NISP_DESIGN_STAGE_MAX];  // indexed by design index

    float total_delta_v_ms;
    float total_mass_t;      // everything, full, at the pad
    float liftoff_twr;       // the bottom stage's TWR, which is the one that matters
    int   part_count;        // rows a stack would build: bodies, boosters, separators
    bool  first_stage_ok;    // the bottom stage can lift
    bool  reach_orbit_ok;    // total delta-v against NISP_REACH_ORBIT_MS
    float reach_orbit_ms;
} nisp_budget_t;

// `body` is whose surface gravity the TWRs are divided by -- BODY_BOB for a
// launch, BODY_CHEESE for a landing stage.
void nisp_budget_compute(nisp_design_t const* d, int body, nisp_budget_t* out);

// The same numbers for a stack that is already flying, where some stages are
// gone and some tanks are half empty. `out` keeps the stage slots that are
// still attached; the rest are zeroed.
void nisp_budget_stack(nisp_stack_t const* s, int body, nisp_budget_t* out);


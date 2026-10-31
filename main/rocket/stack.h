#pragma once
// =====================================================================
//  Null Island Space Program  --  a blueprint turned into a body (Part V3, V4)
// ---------------------------------------------------------------------
//  rocket/design is what the player drew; rocket/stack is what it weighs,
//  where that weight sits, where the thrust comes from, and where the air
//  pushes on it. Every one of those is a sum over the same short list of
//  parts, recomputed whenever the list or its masses change -- on staging,
//  on a burn-through, and every physics tick while fuel burns.
//
//  ONE RIGID BODY (D-22). There are no joints here and no part-level
//  contacts: N parts become one body plus the handful of sums that say
//  how it behaves. The only per-part state that is not a sum is heat
//  (rocket/heat), because "which part burns" is a question a whole-body
//  model cannot answer.
//
//  THE AXIS is one-dimensional and measured from the nose: x = 0 at the
//  capsule's tip, growing toward the tail. CoM, CoT, the drag centre and
//  the static margin are all numbers along that axis, which is why the
//  whole vehicle model is a pass over <= 20 rows rather than a solver.
//
//  WHERE THE BURNING FUEL SITS: a stage empties from its bottom, so the
//  propellant left is the top `fill` fraction of the tank and its centre
//  walks toward the nose as it burns. That is what a solid motor actually
//  does, it is what Part V4 says happens ("the CoM walks up the rocket"),
//  and it is the effect the flight model cares about most: a stage that is
//  half spent has already moved the CoM forward, and one that is empty has
//  moved it all the way (Part A5's worked table).
//
//  STAGES IGNITE BOTTOM-UP. The lowest live stage is the one burning; a
//  staging press discards it and arms the one above. That is the order a
//  rocket works in and the order a KSP player already knows (D-42).
//
//  PURE: no engine, no BSP, no filesystem. Host-compilable (Part H).
// =====================================================================

#include <stdbool.h>

#include "rocket/design.h"
#include "rocket/parts.h"

// Capsule + one separator per stage + one stage body + one booster group
// per stage: 6 stages is 19 rows. The cap is generous, not tight.
#define NISP_STACK_PART_MAX 28
#define NISP_SEPARATOR_SPAN_M 0.125f

typedef enum {
    NISP_PART_KIND_BODY = 0,   // a stage's tank + engines, or the capsule
    NISP_PART_KIND_BOOSTER,    // a stage's side boosters, as one group
    NISP_PART_KIND_SEPARATOR,  // a stage boundary (D-21)
    NISP_PART_KIND_LEGS        // deployed legs: drag only, no mass, no heat
} nisp_part_slot_t;

typedef struct {
    int   slot;        // nisp_part_slot_t
    int   kind;        // nisp_part_row_t index (NISP_KIND_*)
    int   stage;       // design stage index; -1 for the capsule

    int   engine;      // nisp_engine_id_t, or NISP_ENGINE_NONE
    int   engine_count;
    bool  igniting;

    float x0, x1;      // span along the axis, metres from the nose
    float diameter_m;
    float area_m2;     // wetted side area: what the heat model reads
    float cd_a;        // drag area: drag, aero torque, static margin

    float mass_t;      // dry + propellant left
    float dry_t;
    float prop_t;      // remaining
    float prop_full_t;

    // Where the dry mass sits along the span, as a fraction of it from x0.
    // 0.5 is the geometric middle, which is right for a tank; the capsule
    // biases toward its shield end, which is what makes a lone pod
    // weathervane shield-first (D-31).
    float com_bias;

    // Heat (Part A4), copied from the row so a fragment can be tuned per
    // part later without touching the registry.
    float heat;
    float capacity;
    float limit;
    float shield;
} nisp_part_t;

typedef struct {
    nisp_part_t part[NISP_STACK_PART_MAX];
    int count;

    // Which design stages are still attached, and which one is burning.
    int  stage_count;                        // design stages at build time
    int  live_stage;                         // -1 when nothing is left to fire
    int  legs_id[NISP_DESIGN_STAGE_MAX];     // nisp_legs_id_t per stage, 0 for none
    bool legs_deployed[NISP_DESIGN_STAGE_MAX];

    // The sums (Part V4). Valid after nisp_stack_recompute.
    float mass_t;
    float prop_t;
    float com_x;      // centre of mass, along the axis
    float cot_x;      // centre of thrust, over the IGNITING engines only
    float cp_x;       // drag centre
    float static_margin_m;   // cp_x - com_x: positive is a dart (Part A5)
    float inertia_t_m2;      // transverse moment of inertia, t*m^2
    float cd_a_total;
    float throttle;          // 0..1, applied to every armed engine
    float thrust_kn;         // the igniting engines, this instant
    float isp_s;             // thrust-weighted, for the HUD's accel number
    bool  has_capsule;
} nisp_stack_t;

// What one stage of a blueprint weighs, before any of it is attached to
// anything: its own propellant, its own dry structure (tank walls, engines,
// RCS, legs), its side boosters and its separator. rocket/budget and
// nisp_stack_build both read this, so the two can never disagree about what
// a stage weighs (Part V2).
typedef struct {
    float prop_t;        // the stage's own tank
    float dry_t;         // walls + engines + RCS + legs
    float boosters_prop_t;
    float boosters_dry_t;
    float separator_t;
} nisp_stage_mass_t;

void nisp_stage_masses(nisp_stage_t const* st, nisp_stage_mass_t* out);

// Build the stack the blueprint describes: capsule on top, then every
// stage below it, each with its separator and its booster group.
void nisp_stack_build(nisp_design_t const* d, nisp_stack_t* out);

// Rederive every sum from the parts as they are now. Cheap; call it after
// any mass change, any staging, any split, and once per physics tick.
void nisp_stack_recompute(nisp_stack_t* s);

// Where one part's mass sits right now (the burning-propellant rule above).
float nisp_part_com_x(nisp_part_t const* p);
// How full a part's tank is, 0..1.
float nisp_part_fill(nisp_part_t const* p);

// The stage a part belongs to, and the lowest design stage still attached.
int nisp_stack_live_stage(nisp_stack_t const* s);

// Throttle 0..1. Ignition is per stage: a stage's engines and its side
// boosters ignite together (Part V1). An armed stage at throttle 0 is "engines
// off": nothing burns, and the next press of the throttle lights it again.
void nisp_stack_ignite(nisp_stack_t* s, int stage, float throttle);
void nisp_stack_cut(nisp_stack_t* s);  // throttle to zero, nothing discarded

// Burn propellant for `dt_s` at the current throttle, and drop any booster
// group that has run dry (Part F3: boosters leave on their own burn-out).
// Returns true when a booster group left.
bool nisp_stack_burn(nisp_stack_t* s, float dt_s);

// The staging press: discard the lowest live stage -- its separator, its
// tank and its boosters -- and arm the stage above. Returns how many parts
// left, 0 when there is nothing above to arm.
int nisp_stack_advance(nisp_stack_t* s);

// A burn-through (Part F5): the part at `index` is gone, and the stack is
// cut at it. The controlled fragment (the one with the capsule) stays in
// `s`; the other range is written to `other` and is debris from here on.
// Returns false when the capsule itself was the part that failed.
bool nisp_stack_split(nisp_stack_t* s, int index, nisp_stack_t* other);

// Drop a range of parts out of a stack (used by staging and by debris).
int nisp_stack_drop_stage(nisp_stack_t* s, int stage);

void nisp_stack_deploy_legs(nisp_stack_t* s, int stage, bool deployed);
bool nisp_stack_legs_deployed(nisp_stack_t const* s, int stage);

// The stage a part index belongs to (-1 for the capsule).
int nisp_part_stage(nisp_stack_t const* s, int index);

// The hottest part, and its normalised load (0..1 of its limit).
int   nisp_stack_hottest(nisp_stack_t const* s);
float nisp_stack_heat_load(nisp_stack_t const* s);


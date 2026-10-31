#pragma once
// =====================================================================
//  Null Island Space Program  --  the part registry (PART[])
// ---------------------------------------------------------------------
//  Every number a rocket is made of lives here, as a row. A new engine,
//  a new leg set, a heavier separator is a row edit in this file and not
//  an edit in five files (Part L principle 2).
//
//  THE LADDERS. Diameters and lengths are not free numbers: a diameter is
//  one of four sizes and a length is one of sixteen steps. The Engineering
//  sheet edits ladders rather than typing decimals (Part B2), and a stack
//  built from ladders is a stack whose drawing always fits the sheet.
//
//  THE MASS MODEL is one cylinder (Part V2):
//
//      propellant = pi (d/2)^2 * L * 400 kg/m^3
//      dry        = dry_fraction * propellant        (+ engines, RCS, legs)
//
//  `dry_fraction` is the only place a stage's structure is expressed, and
//  it is per stage type: a booster's casing is heavier than an upper
//  stage's thin walls.
//
//  THE FOUR COLUMNS the flight model reads and the blueprint draws:
//
//      capacity  how much heat the part swallows before it goes (Part A4)
//      limit     where `heat` counts as "this part is gone"
//      shield    0 for everything except the capsule (~0.9, and only when
//                the shield faces the flow -- a factor, not immunity, D-36)
//      cd        drag area per m^2 of side area; CdA = cd * side area,
//                used for drag, for the aero torque and for the static
//                margin (Part A5)
//
//  These are tuning targets, and nispcheck case 9 is what tunes them: "a
//  normal launch survives, a bad reentry does not" is a property of these
//  columns and nothing else (F-12, F-16).
//
//  PURE: no engine, no BSP, no filesystem. Host-compilable (Part H).
// =====================================================================

#include <stdbool.h>

// --- The ladders (Part V2) ---------------------------------------------

// The capsule's integrated parachute (Part A3): drag area in m^2, and the
// dynamic pressure in Pa below which it is not worth deploying -- a chute
// streamed in vacuum does nothing, and the key should say so.
//
// The 300 m^2 is sized backwards from the number it has to produce, not
// forwards from a canopy drawing (F-29): Part A3's soft touchdown is 10 m/s,
// and a chute has to actually deliver it. 300 m^2 of drag area -- a 20 m
// canopy, which is what a real capsule drags -- gives a 1.0 t pod 7.3 m/s at
// sea level and a 5.0 t fragment 16 m/s, which is why a heavy fragment needs
// legs or a retro burn instead of a key press.
//
// The upper gate is the same rule from the other side: a chute that opens at
// 800 m/s is a wreck either way, and it is also the one thing in this model
// whose drag is stiffer than the physics tick (Part F1, F-30).
#define NISP_CHUTE_CDA_M2 300.0f
#define NISP_CHUTE_MIN_Q  40.0f
#define NISP_CHUTE_MAX_V_MS 300.0f

// How many canopies a pod may carry (D-51). One is the default and the
// number the paragraph above was sized backwards from, so a rocket built
// before this existed descends exactly as it did. Each extra canopy is the
// same area more, and a few tens of kilos more.
#define NISP_CHUTE_MAX        3
#define NISP_CHUTE_EXTRA_MASS_T 0.05f  // per canopy after the first

// The canopy area a pod of `chutes` parachutes actually drags. 1..NISP_CHUTE_MAX;
// anything else is read as one.
float nisp_chute_cd_a(int chutes);

// The pod's mass with the two options the player can give it. rocket/stack
// and rocket/budget both read this, so the drawing's numbers and the flight's
// numbers cannot disagree (Part V2).
float nisp_pod_mass_t(bool pod_rcs, int chutes);

#define NISP_DIAMETER_COUNT 4
#define NISP_LENGTH_STEP_M 0.625f
#define NISP_LENGTH_MAX_STEPS 16  // 0.625 m .. 10 m

// Propellant density, kg/m^3. One number for every tank in the game.
#define NISP_PROP_DENSITY 400.0f

// A ladder index is what the editor stores; the metre is what the maths
// uses. Both are 0-based, so a ladder index is always a valid array index.
float nisp_diameter_m(int ladder);
float nisp_length_m(int step);
int  nisp_diameter_steps(void);   // NISP_DIAMETER_COUNT
int  nisp_length_steps(void);     // NISP_LENGTH_MAX_STEPS

// --- Engines -----------------------------------------------------------

typedef enum {
    NISP_ENGINE_FLEA = 0,   // small upper stage
    NISP_ENGINE_RELION,     // vacuum upper stage
    NISP_ENGINE_SKIPPER,    // first stage workhorse
    NISP_ENGINE_MAINSAIL,   // heavy first stage
    NISP_ENGINE_BOOSTER,    // solid: side boosters only
    NISP_ENGINE_NONE,       // a cargo section, an adapter, a fuel-only stage
    NISP_ENGINE_COUNT
} nisp_engine_id_t;

typedef struct {
    char const* name;
    float thrust_kn;  // per engine
    float isp_s;      // per engine
    float mass_t;     // per engine; 0 for a solid, whose mass is its propellant
    bool  solid;      // burns its own cylinder; no throttle, no tank
} nisp_engine_row_t;

nisp_engine_row_t const* nisp_engine_row(int id);  // never NULL

// How many of one engine a stage may carry (the editor's ladder).
int nisp_engine_count_max(void);

// --- RCS, legs, separators ---------------------------------------------

// One RCS module per stage that has them: a torque budget, not a thrust.
#define NISP_RCS_MASS_T 0.10f
#define NISP_RCS_TORQUE_KNM 15.0f  // total, both axes, at full authority

typedef enum {
    NISP_LEGS_NONE = 0,
    NISP_LEGS_3,
    NISP_LEGS_4,
    NISP_LEGS_6,
    NISP_LEGS_COUNT
} nisp_legs_id_t;

typedef struct {
    int   count;
    float mass_t;        // the whole set
    float rated_v_ms;    // the touchdown speed this set is rated for
    float rated_tilt_deg;
    float cd_a;          // extra drag area at the tail, deployed only (Part V7)
} nisp_legs_row_t;

nisp_legs_row_t const* nisp_legs_row(int id);  // id 0 is a real row: none

// One separator per stage boundary, implicit or not (D-21).
#define NISP_SEPARATOR_MASS_T 0.05f

// --- Part rows ---------------------------------------------------------

typedef enum {
    NISP_KIND_CAPSULE = 0,  // the top of every stack; never in the legend list
    NISP_KIND_MAIN,         // engine + propellant: the workhorse
    NISP_KIND_UPPER,        // vacuum engine, thin walls, small diameter
    NISP_KIND_BOOSTER,      // solid: heavy thrust, burns out on its own
    NISP_KIND_CARGO,        // no engine: payload, fuel-only tank, adapter
    NISP_KIND_COUNT
} nisp_part_kind_t;

typedef struct {
    char const* name;       // the legend label
    char const* abbr;       // the sheet's dimension callout

    int   engine;           // the engine this row defaults to
    bool  engines_allowed;  // false for cargo: an engine is a choice there
    int   max_engines;

    float dry_fraction;     // dry mass / propellant mass
    float cd;               // CdA = cd * side area, m^2 per m^2

    // Heat (Part A4). rate = K * rho v^3 * area * expose / capacity, and
    // the part is gone when heat >= limit. `capacity` is this many units
    // per tonne of the part, so a 44 tonne tank out-masses a 1 tonne pod
    // instead of both being told they are equally tough.
    float capacity_per_t;
    float limit;
    float shield;           // 0 except the capsule
} nisp_part_row_t;

nisp_part_row_t const* nisp_part_row(int kind);  // never NULL

// The scrollable legend (Part B3): the four buildable rows, in the order
// the sheet lists them. The capsule is not in it -- it is always there.
int                nisp_legend_count(void);
int                nisp_legend_kind(int i);

// The capsule is one fixed row: 1.0 t, 2.0 m tall, its mass biased toward
// the shield end so a lone capsule weathervanes shield-first (D-31).
#define NISP_CAPSULE_MASS_T 1.0f
#define NISP_CAPSULE_LENGTH_M 2.0f
// Where the dry mass sits along the capsule, as a fraction of its length
// from the nose. 0.45 is toward the shield end, which is what puts the CoM
// ahead of the drag centre.
#define NISP_CAPSULE_MASS_BIAS 0.45f

// --- The cylinder maths (Part V2) --------------------------------------

// Propellant mass, tonnes, for a cylinder of these dimensions.
float nisp_propellant_mass_t(float diameter_m, float length_m);
// Its wetted side area, m^2 -- the area the heat model reads.
float nisp_side_area_m2(float diameter_m, float length_m);


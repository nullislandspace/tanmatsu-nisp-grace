// =====================================================================
//  Null Island Space Program  --  the part registry, as rows (see parts.h)
// =====================================================================

#include <math.h>

#include "rocket/parts.h"

// --- The ladders -------------------------------------------------------

static float const DIAMS[NISP_DIAMETER_COUNT] = {1.25f, 1.875f, 2.5f, 3.75f};

int nisp_diameter_steps(void) { return NISP_DIAMETER_COUNT; }
int nisp_length_steps(void) { return NISP_LENGTH_MAX_STEPS; }

float nisp_diameter_m(int ladder) {
    if (ladder < 0) ladder = 0;
    if (ladder >= NISP_DIAMETER_COUNT) ladder = NISP_DIAMETER_COUNT - 1;
    return DIAMS[ladder];
}

float nisp_length_m(int step) {
    if (step < 0) step = 0;
    if (step >= NISP_LENGTH_MAX_STEPS) step = NISP_LENGTH_MAX_STEPS - 1;
    return NISP_LENGTH_STEP_M * (float)(step + 1);
}

// --- Engines -----------------------------------------------------------

static nisp_engine_row_t const ENGINES[NISP_ENGINE_COUNT] = {
    //  name        thrust  Isp   dry mass  solid
    {"Flea",     30.0f,  240.0f, 0.15f, false},
    {"Relion",   60.0f,  320.0f, 0.30f, false},
    {"Skipper", 180.0f,  270.0f, 0.90f, false},
    {"Mainsail",300.0f,  290.0f, 1.20f, false},
    {"Booster", 400.0f,  180.0f, 0.00f, true},   // sized: its mass is its propellant
    {"(none)",    0.0f,    0.0f, 0.00f, false},
};

nisp_engine_row_t const* nisp_engine_row(int id) {
    if (id < 0 || id >= NISP_ENGINE_COUNT) id = NISP_ENGINE_NONE;
    return &ENGINES[id];
}

int nisp_engine_count_max(void) { return 4; }

// --- Legs --------------------------------------------------------------

static nisp_legs_row_t const LEGS[NISP_LEGS_COUNT] = {
    // count  mass   rated v  rated tilt  deployed drag
    {0, 0.00f,  0.0f,  0.0f, 0.0f},
    {3, 0.60f,  8.0f, 15.0f, 1.2f},
    {4, 1.00f, 12.0f, 20.0f, 1.6f},
    {6, 1.80f, 20.0f, 25.0f, 2.4f},
};

float nisp_chute_cd_a(int chutes) {
    if (chutes < 1) chutes = 1;
    if (chutes > NISP_CHUTE_MAX) chutes = NISP_CHUTE_MAX;
    return NISP_CHUTE_CDA_M2 * (float)chutes;
}

float nisp_pod_mass_t(bool pod_rcs, int chutes) {
    if (chutes < 1) chutes = 1;
    if (chutes > NISP_CHUTE_MAX) chutes = NISP_CHUTE_MAX;
    return NISP_CAPSULE_MASS_T + (pod_rcs ? NISP_RCS_MASS_T : 0.0f) +
           NISP_CHUTE_EXTRA_MASS_T * (float)(chutes - 1);
}

nisp_legs_row_t const* nisp_legs_row(int id) {
    if (id < 0 || id >= NISP_LEGS_COUNT) id = NISP_LEGS_NONE;
    return &LEGS[id];
}

// --- Part rows ---------------------------------------------------------

// `capacity_per_t` is the heat model's thermal mass per tonne of the part
// (rocket/heat.c), and every number in the column is tuned by nispcheck
// case 9 rather than guessed: a nominal launch must peak near a third of
// the limit for every part, a nominal reentry must finish off the
// unshielded stages, and neither may finish off the pod. The capsule is the
// fragile one on purpose -- it is small, its shield is a factor of 0.1 and
// not a wall (D-36), and case 9's fourth profile is the reentry that proves
// the difference.
static nisp_part_row_t const PARTS[NISP_KIND_COUNT] = {
    //  name            abbr   engine          eng? max dry   cd     cap/t  limit shield
    {"Command pod",    "POD", NISP_ENGINE_NONE, false, 1, 0.00f, 0.40f, 0.053f, 1.0f, 0.90f},
    {"Main stage",     "MAIN", NISP_ENGINE_SKIPPER, true, 4, 0.08f, 0.50f, 0.089f, 1.0f, 0.00f},
    {"Upper stage",    "UPR", NISP_ENGINE_RELION, true, 2, 0.06f, 0.45f, 0.161f, 1.0f, 0.00f},
    {"Booster stage",  "BST", NISP_ENGINE_BOOSTER, true, 2, 0.15f, 0.55f, 0.087f, 1.0f, 0.00f},
    {"Cargo section",  "CRG", NISP_ENGINE_NONE, false, 0, 0.08f, 0.50f, 0.089f, 1.0f, 0.00f},
};

nisp_part_row_t const* nisp_part_row(int kind) {
    if (kind < 0 || kind >= NISP_KIND_COUNT) kind = NISP_KIND_MAIN;
    return &PARTS[kind];
}

static int const LEGEND[NISP_KIND_COUNT - 1] = {
    NISP_KIND_MAIN, NISP_KIND_UPPER, NISP_KIND_BOOSTER, NISP_KIND_CARGO,
};

int nisp_legend_count(void) { return NISP_KIND_COUNT - 1; }

int nisp_legend_kind(int i) {
    if (i < 0 || i >= nisp_legend_count()) return NISP_KIND_MAIN;
    return LEGEND[i];
}

// --- The cylinder maths ------------------------------------------------

float nisp_propellant_mass_t(float diameter_m, float length_m) {
    float const r = 0.5f * diameter_m;
    return (float)M_PI * r * r * length_m * NISP_PROP_DENSITY * 0.001f;  // kg -> t
}

float nisp_side_area_m2(float diameter_m, float length_m) {
    return (float)M_PI * diameter_m * length_m;
}

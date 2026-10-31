#pragma once
// =====================================================================
//  Null Island Space Program  --  a rocket, as a blueprint (Part V1, V6)
// ---------------------------------------------------------------------
//  A design is a stack described by numbers, built top-down: one capsule
//  that is always there and always on top, then stages below it. Nothing
//  here knows about drawing or flying -- it is the player's intent, and
//  rocket/stack turns it into masses.
//
//  STAGES ARE STORED TOP FIRST. stage[0] hangs directly under the capsule,
//  stage[stage_count-1] is the bottom of the rocket. That is the order the
//  sheet draws them and the order the player adds them.
//
//  A DESIGN IS A FILE, not a save slot (D-23): /sd/nisp/designs/<name>.nbp,
//  the same tagged `key=value` line format settings.txt uses -- forward
//  compatible, unknown keys ignored, hand-editable on the card. The same
//  rocket should be launchable from any slot, and a slot should be able to
//  have flown several.
//
//  IT MUST BE NAMED BEFORE IT CAN BE LAUNCHED (the user's rule): the sheet
//  disables its Launch item while `name` is empty, and the name is what the
//  mission and the save record.
//
//  THE FILE CODE IS THE ONE PLACE in main/rocket that touches the card, and
//  it is compiled two ways: stdio on the host (so nispcheck can round-trip
//  a blueprint byte-for-byte) and the same stdio calls on the badge, which
//  exports fopen/fread/fwrite but not remove/rename -- hence common/vfs_compat.
// =====================================================================

#include <stdbool.h>

#include "rocket/parts.h"

#define NISP_DESIGN_NAME_MAX 24
#define NISP_DESIGN_STAGE_MAX 6

typedef struct {
    int  kind;            // NISP_KIND_MAIN / UPPER / BOOSTER / CARGO
    int  diameter;        // ladder index, 0..NISP_DIAMETER_COUNT-1
    int  length;          // ladder step, 0..NISP_LENGTH_MAX_STEPS-1
    int  engine;          // nisp_engine_id_t; NISP_ENGINE_NONE for a cargo section
    int  engine_count;    // 1..nisp_engine_count_max()
    bool rcs;
    int  boosters;        // 0 / 2 / 4 side boosters on this stage
    int  booster_length;  // ladder step: how tall a booster is
    int  legs;            // nisp_legs_id_t
} nisp_stage_t;

typedef struct {
    char name[NISP_DESIGN_NAME_MAX + 1];  // empty until the player names it
    int  stage_count;

    // The capsule is always there and never a stage, but it is not a fixed
    // part either (D-51): RCS pods and 1..3 parachutes are the player's
    // call, and they are the only two things Engineering can change about
    // the pod. Both are the default (no RCS, one chute) when a blueprint
    // says nothing, which is what keeps an old file an old rocket.
    bool pod_rcs;
    int  pod_chutes;  // 1..NISP_CHUTE_MAX

    nisp_stage_t stage[NISP_DESIGN_STAGE_MAX];  // 0 = uppermost
} nisp_design_t;

// A starter rocket: one main stage, Skipper, 2.5 m x 7 steps. Named only
// when the player says so.
void nisp_design_init(nisp_design_t* d);

// The rocket the game flies when nobody has built one yet: named, two
// stages, RCS and legs. The pad default and the test bench's ship.
void nisp_design_demo(nisp_design_t* d);

// Every field forced back into its ladder. Called after any edit, so no
// caller can hand a stack a diameter of 3.14.
void nisp_design_clamp(nisp_design_t* d);

bool nisp_design_named(nisp_design_t const* d);
// Letters, digits, space, dash, underscore; 1..NISP_DESIGN_NAME_MAX long.
bool nisp_design_name_ok(char const* name);

// Insert a stage of `kind` directly below index `at`, so the stage that
// was at `at` stays where it is and the new one hangs under it. `at < 0`
// means the capsule: the new stage goes directly under it, at index 0. An
// `at` past the bottom appends. False when the stack is full.
bool nisp_design_insert(nisp_design_t* d, int at, int kind);
// Remove the stage at `at`. False when the stack would be left with none.
bool nisp_design_remove(nisp_design_t* d, int at);

// --- Serialization (pure, and what nispcheck case 6 round-trips) -------

// The canonical text, NUL-terminated, into `buf`. Returns the length, or
// -1 when `cap` is too small. Encode(decode(text)) == text for anything
// this function wrote, which is the round trip the test checks.
int nisp_design_encode(nisp_design_t const* d, char* buf, int cap);
bool nisp_design_decode(char const* text, nisp_design_t* out);

// --- The card ----------------------------------------------------------

// "/sd/nisp/designs/<name>.nbp". Writes, and returns false without leaving
// a half-written file behind.
bool nisp_design_save(nisp_design_t const* d);
bool nisp_design_load(char const* name, nisp_design_t* out);
bool nisp_design_exists(char const* name);
bool nisp_design_rename(char const* from, char const* to);
bool nisp_design_delete(char const* name);

// Names only, sorted, up to `max`. Returns how many (<= max).
int nisp_design_list(char names[][NISP_DESIGN_NAME_MAX + 1], int max);

// How many designs are on the card.
int nisp_design_count(void);


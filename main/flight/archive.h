#pragma once
// =====================================================================
//  Null Island Space Program  --  the craft you left out there (plan 9.4)
// ---------------------------------------------------------------------
//  A mission you leave behind becomes a record: enough to put the ship back
//  where it was, and nothing else. It is not a save file and it is not a
//  second physics sim -- it is the difference between "I went back to the
//  Spaceport and my rocket is gone" and "I went back and my rocket is still
//  where I left it".
//
//  WHY ELEMENTS AND NOT A STATE VECTOR: a state vector is true at one
//  instant, and a craft left in orbit is at a different instant every time
//  you come back for it. Elements carry their own epoch (sim/kepler.h), so
//  a record picked up an hour later is propagated to now rather than pasted
//  at the position it had when you left. Same reason the map draws orbits
//  (D-17). The record keeps the mission's own clock alongside them, so the
//  sky and the craft are put back at the same instant.
//
//  WHY A DESIGN NAME AND NOT A COPY OF THE BLUEPRINT: the blueprint is
//  already on the card (D-11: nothing flies that was not saved first). A
//  record that also carried the design would be a second copy that can
//  disagree with the one the player can still edit.
//
//  WHAT A RECORD DOES NOT KEEP: heat. A craft left in vacuum is cold and a
//  craft left on the ground has had all day to cool, so nisp_stack_build's
//  fresh blueprint is the honest answer rather than a snapshot of a number
//  that is only interesting while something is burning.
//
//  PURE: no engine, no BSP, no filesystem (tools/hostpurity.sh). The save
//  layer (save/save.c) is what puts these on the card; the menus only read
//  them; main.c is the one that decides when a flight becomes a record.
// =====================================================================

#include <stdbool.h>

#include "flight/mission.h"
#include "rocket/design.h"
#include "sim/kepler.h"

// Four is a decision, not a limit: the Spaceport picker has to fit on one
// panel, and a player who has left more than four craft out there has a
// different problem than a menu (Part N1).
#define NISP_ARCHIVE_MAX 4

typedef struct {
    bool used;

    char vessel[NISP_DESIGN_NAME_MAX + 1];  // what the player called the ship
    char design[NISP_DESIGN_NAME_MAX + 1];  // the blueprint it was built from

    // The clock this record belongs to. Restoring puts the craft at `t` and
    // moves the world clock there with it, so the sky matches the ship.
    double t;

    int  body;        // whose sphere of influence it is in (sim/patches.h)
    bool has_orbit;   // false for a craft that is down: lat/lon is its place
    nisp_elements_t el;

    int  state;       // nisp_mission_state_t
    bool airborne;    // ever left the pad (mission.h)
    bool landed;
    float lat_rad, lon_rad;

    // The rocket as it was left, by DESIGN stage index (rocket/design.h:
    // 0 is the uppermost). A stage that was jettisoned is simply not
    // attached, and a stage that was half burned is half full.
    bool  attached[NISP_DESIGN_STAGE_MAX];
    float fill[NISP_DESIGN_STAGE_MAX];  // 0..1
    bool  legs[NISP_DESIGN_STAGE_MAX];

    bool chute;

    // The controls as the mission last had them, so picking a craft up does
    // not silently reset the player's SAS mode or throttle dial.
    bool  ignited;
    float throttle;
    int   sas;
    bool  rcs;

    // The flight log, so a craft picked up again still knows its own bests.
    float max_ap_km;
    float max_speed_ms;
    float max_heat;
    int   stages_dropped;
    int   parts_lost;
} nisp_flight_t;

typedef struct {
    nisp_flight_t flight[NISP_ARCHIVE_MAX];
    int count;  // dense: 0..count-1 are the used ones
} nisp_archive_t;

void nisp_archive_init(nisp_archive_t* a);
int  nisp_archive_count(nisp_archive_t const* a);

// The record at `i`, or NULL. Read-only use: the menus ask what is out there.
nisp_flight_t const* nisp_archive_get(nisp_archive_t const* a, int i);

// Turn a live mission into a record. False when there is nothing to leave
// behind -- a wreck, a craft still sitting on the pad, or one on a radial
// hop that defines no orbit -- and leaves the archive untouched.
bool nisp_archive_add(nisp_archive_t* a, nisp_mission_t const* m, char const* design_name);

// Drop the record at `i`. The list stays dense.
bool nisp_archive_remove(nisp_archive_t* a, int i);

// Put the craft back: build it from `design`, restore the stages that are
// still attached and the propellant they have left, and start a mission on
// it at the record's own clock. Consumes the record on success -- a craft
// cannot be out there and in your hands at the same time. `design` is the
// caller's business (nisp_design_load), because this file does not touch
// the card.
bool nisp_archive_restore(nisp_archive_t* a, int i, nisp_design_t const* design, nisp_mission_t* m);

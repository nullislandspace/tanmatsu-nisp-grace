#pragma once
// =====================================================================
//  Null Island Space Program  --  the save slots
// ---------------------------------------------------------------------
//  Three slots, owned by the engine's save framework (se_save.h): it
//  keeps the directory, the NBT wrapping and the peek header; this file
//  keeps the game's own schema and the two questions the menus ask --
//  what is in a slot, and what to do about it.
//
//  A save holds a SEED and a pose, not a world. The island is rebuilt
//  from the seed (common/rng.h), so a save is a sentence rather than a
//  megabyte -- and it stays that way however big the world gets, because
//  only what the player CHANGED will ever need to be written down.
//
//  Deleting a slot reaches into the engine's file naming: se_save.c
//  spells a slot as "<dir>/save<slot>.bin". That is not public API, so
//  it lives here, in one line, where an engine change can find it.
// =====================================================================

#include <stdbool.h>
#include <stdint.h>

#include "se_save.h"

#define NISP_SAVE_SLOTS SE_SAVE_SLOT_COUNT
#define NISP_NAME_MAX   24

typedef struct {
    float x, y, z;
    float yaw, pitch;  // engine convention: positive pitch looks DOWN (se_scene.c)
} nisp_pose_t;

typedef struct {
    char     name[NISP_NAME_MAX];
    uint32_t seed;
    nisp_pose_t pose;
    float flown;  // metres of thrust logged -- the only progress kept so far
} nisp_game_t;

// What a slot says about itself without loading it.
typedef struct {
    bool     exists;
    char     info[SE_SAVE_INFO_MAX];
    int64_t  timestamp;
    uint32_t seed;  // read out of the save itself, so a slot list can show it
    char     name[NISP_NAME_MAX];  // ... and what the player called it
} nisp_slot_t;

// Create the save directory and register the schema. Call once, after the
// engine has booted.
void save_init(void);

bool save_slot_exists(int slot);
// 0 on success, -1 if the slot is missing or unreadable.
int save_slot_peek(int slot, nisp_slot_t* out);

// A fresh game in `slot`, written to disk immediately so the slot exists
// before the player has flown anywhere. Fills `game` with what was created.
bool save_new(int slot, char const* name, uint32_t seed, nisp_game_t* game);

bool save_load(int slot, nisp_game_t* game);
bool save_write(int slot, nisp_game_t const* game);
bool save_rename(int slot, char const* name);
bool save_delete(int slot);

// "2026-10-30 14:05" for a slot list. Empty string for a timestamp of 0.
char const* save_stamp(char* buf, int cap, int64_t timestamp);

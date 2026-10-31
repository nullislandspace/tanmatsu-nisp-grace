#pragma once
// =====================================================================
//  Null Island Space Program  --  a flight, start to finish (Parts F1-F5)
// ---------------------------------------------------------------------
//  The mission is the thing that owns a ship: it holds the sim clock, the
//  vessel, the debris its failures produced, and the outcome. game/ and
//  map/ read it; nothing in it knows about drawing.
//
//  ONE FRAME OF IT, in order:
//
//    1. the player's one-shot commands (stage, chute, legs, cut)
//    2. the throttle onto the stage that is armed
//    3. flight/warp decides what this frame is allowed to simulate
//    4. coast it analytically, or tick it at dt_phys, resolving heat
//       failure and ground contact after every tick
//    5. the same for the debris, which is never controlled but is always
//       simulated (Part F5)
//    6. the mission log, and the music context with its hysteresis band
//       (Part N4)
//
//  THE CONTROLLED BODY IS ALWAYS THE CAPSULE (D-33): every command in the
//  struct acts on `ship`, and `debris` is what the failures left behind.
//  Debris is ballistic, still heated, allowed to fail once, and capped at
//  NISP_DEBRIS_MAX because eight bodies is a frame-budget decision, not a
//  physics one (Part G).
//
//  PURE: no engine, no BSP, no filesystem. Host-compilable (Part H), so
//  nispcheck case 13 can fly a whole mission -- launch, orbit, deorbit,
//  chute, touchdown -- without a badge in the loop.
// =====================================================================

#include <stdbool.h>

#include "flight/control.h"
#include "flight/physics.h"
#include "flight/warp.h"
#include "rocket/design.h"

#define NISP_DEBRIS_MAX 8

// The hysteresis band (Part N4): space above 70 km, silence below 60.
// Without the band an orbit whose periapsis dips into the air restarts the
// MP3 decoder every pass, which is audible as a stutter.
#define NISP_MUSIC_SPACE_KM 70.0f
#define NISP_MUSIC_AIR_KM 60.0f

typedef enum {
    NISP_MISSION_FLYING = 0,
    NISP_MISSION_LANDED,  // down, intact, where it says it is
    NISP_MISSION_WRECKED  // the capsule went, in the air or on impact
} nisp_mission_state_t;

typedef struct {
    nisp_vessel_t ship;
    nisp_vessel_t debris[NISP_DEBRIS_MAX];
    int debris_count;

    double t;         // the sim clock, seconds
    double t_launch;  // the sim clock this flight started on: T+ is t - t_launch
    int    state;

    // What the player called the rocket (plan 6.1). Copied from the design at
    // launch, because a fragment of a broken rocket is still the same ship,
    // and the HUD names the ship rather than the save slot.
    char vessel_name[NISP_DESIGN_NAME_MAX + 1];

    // What the save slot remembers afterwards (9.3): the numbers a flight
    // is judged by, and the only "score" the game has.
    float max_ap_km;
    float max_speed_ms;
    float max_heat;
    int   stages_dropped;
    int   parts_lost;

    // Part N4, decided here so the audio layer only has to obey it.
    bool music_space;

    // What the controls are set to, as the mission last obeyed them: the
    // HUD and the map report the SAS mode and the RCS switch without either
    // of them owning nisp_control_t or having to be the one that reads keys.
    int  sas;
    bool rcs;

    // The stage key has two jobs and this says which one this press is
    // (D-56). Cold means the armed stage is not burning: the next press of
    // the stage key LIGHTS it rather than dropping it, which is what a
    // launch starts with -- a rocket on the pad, a throttle dialled up
    // first, and one key to let go. Lit means the next press separates, as
    // it always has.
    //
    // `throttle` is the dial, not what the engines are getting: cold
    // engines are burning nothing whatever the dial says, and the HUD has
    // to show the dial the player is holding.
    bool  ignited;
    float throttle;

    // What the last frame cost, for the profiler and the HUD.
    nisp_warp_step_t last_warp;

    // A command that was refused, in the mission's own words, and only for
    // the frame it was refused on (plan 6.5). The player pressed something;
    // the answer is the reason, not silence.
    char last_refusal[96];
} nisp_mission_t;

void nisp_mission_init(nisp_mission_t* m);

// On the pad, engines COLD, holding the throttle the player sets but not
// obeying it until the stage key says so (D-56). `t` is the absolute sim
// clock the launch happens at -- the pad is on a turning planet in a solar
// system that is somewhere, and a launch in daylight is chosen with
// nisp_local_noon (sim/ephemeris.h, plan 6.1) rather than assumed.
void nisp_mission_launch(nisp_mission_t* m, nisp_design_t const* d, int body, double t);

// Adopt a vessel that is already somewhere: the resume path (9.3) restores a
// ship from its save slot, and nispcheck's flight cases start a mission on an
// orbit instead of flying a launch to get there. The log starts empty either
// way; `t` is the sim clock the mission inherits.
void nisp_mission_begin(nisp_mission_t* m, nisp_vessel_t const* v, double t);

// One rendered frame. `ctl` is read, and its one-shot flags are consumed.
void nisp_mission_step(nisp_mission_t* m, nisp_control_t* ctl, double frame_dt);

// The music context: true when this mission wants `music/space/`.
bool nisp_mission_music_space(nisp_mission_t const* m);

char const* nisp_mission_state_name(int state);

// The ship's readout, for the HUD and the map.
void nisp_mission_readout(nisp_mission_t const* m, nisp_readout_t* out);

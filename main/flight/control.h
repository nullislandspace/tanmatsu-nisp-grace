#pragma once
// =====================================================================
//  Null Island Space Program  --  what the player asks the ship to do (Part F2)
// ---------------------------------------------------------------------
//  One struct, filled every frame from the key bindings, read once per
//  physics substep. It is the whole interface between "the player pressed
//  something" and "the ship did something": game/input turns keys into
//  these fields, flight/physics turns these fields into forces.
//
//  THREE CONTROL MODES, one struct field apiece:
//
//    manual   -- the keys are torque commands and nothing is stabilised.
//                This is the default, and it is the mode where a rocket
//                with no RCS is flown by the air or not at all.
//    SAS      -- a damped attitude hold. It aims the nose at a reference
//                direction (its own heading, prograde, retrograde, the
//                orbit normal, radial out/in) and fights rotation. Its
//                authority is the RCS budget, and no more (Part F2).
//    auto     -- steer to an armed maneuver node. That is step 8.3, and
//                the mode is here now so the struct does not have to
//                change when it arrives.
//
//  THE HONEST FAILURE MODE: no RCS modules on the ship means no control
//  torque, so the ship stops responding and the air decides its attitude.
//  There is no free stabilisation and no hidden reaction wheel, because a
//  player who never finds out that RCS is a thing they must buy is a
//  player who has not been taught anything (Part F2).
//
//  PURE: no engine, no BSP, no keys. Host-compilable (Part H).
// =====================================================================

#include <stdbool.h>

#include "rocket/stack.h"

typedef enum {
    NISP_SAS_OFF = 0,  // manual
    NISP_SAS_HOLD,     // hold the heading it has, and stop rotating
    NISP_SAS_PROGRADE,
    NISP_SAS_RETROGRADE,
    NISP_SAS_NORMAL,   // the orbit's angular-momentum direction
    NISP_SAS_RADIAL_OUT,
    NISP_SAS_RADIAL_IN,
    NISP_SAS_BURN,     // an armed node's burn vector (step 8.3)
    NISP_SAS_COUNT
} nisp_sas_mode_t;

char const* nisp_sas_name(int mode);

typedef struct {
    float throttle;  // 0..1, the target the stack is driven to
    bool  cut;       // engines off, nothing discarded
    bool  stage;     // one-shot: discard the live stage (consumed by mission)
    bool  chute;     // one-shot: deploy the parachute
    bool  legs;      // one-shot: deploy the live stage's leg set

    int  sas;      // nisp_sas_mode_t
    bool rcs;      // the RCS switch: off means no control torque at all
    float torque[3];  // manual command, kN*m, vehicle axes: pitch, yaw, roll

    float warp;  // requested warp; flight/warp decides what is allowed (Part F4)
} nisp_control_t;

// Full deflection of one manual axis. Equal to the RCS budget so a manual
// pull is "all the authority there is", which is what a player expects from
// a key that is either pressed or not.
#define NISP_MANUAL_TORQUE_KNM 15.0f

// SAS commands a rotation RATE and then chases it (F-36), because the
// torque a spring wants and the torque an RCS has are three orders of
// magnitude apart on a launcher. Two numbers, both frame-rate independent:
//
//   APPROACH_S  the settling time the SAS aims for. The commanded rate is
//               the angle error over this, so a small pod turns in about
//               this long and a big one turns as fast as its RCS can.
//   RATE_GAIN   1/s, how hard the rate loop chases the commanded rate. It
//               is deliberately stiff -- the torque it asks for saturates
//               at the RCS budget, which is what makes the slew bang-bang
//               and time-optimal. It stays integrable while RATE_GAIN *
//               dt_phys < 2, which at NISP_PHYS_DT leaves a factor of 10.
#define NISP_SAS_APPROACH_S 2.0f  // seconds to close an attitude error
#define NISP_SAS_RATE_GAIN 8.0f   // 1/s on the rate error

void nisp_control_init(nisp_control_t* c);

// The torque the control system is allowed to spend, kN*m: one budget per
// RCS-equipped stage that is still attached to this fragment (Part V4's
// "a table torque budget per module"). RCS modules are bolted to the hull,
// so a module two stages up still pushes on the same rigid body -- which is
// why the count is over attached stages and not over the armed one. Debris
// has no control at all (Part F5), which is why this reads a stack.
float nisp_control_authority_knm(nisp_stack_t const* s, bool rcs_on);

// How many RCS modules this fragment still has: the design stages that both
// have modules and still have a part in this stack. 0 for a stack with none.
int nisp_control_rcs_modules(nisp_stack_t const* s);

// Is the control system asking for torque this frame? SAS on, or a manual
// deflection, with authority to spend it. It matters beyond the torque
// itself: an analytic coast is only the right answer while nothing is
// pushing on the ship, and a SAS that is fighting is pushing (F-31).
bool nisp_control_active(nisp_stack_t const* s, nisp_control_t const* ctl);

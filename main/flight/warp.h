#pragma once
// =====================================================================
//  Null Island Space Program  --  time, on a ladder (Part F4)
// ---------------------------------------------------------------------
//  Warp multiplies simulated time. It never changes the physics tick: at
//  warp `w` a frame advances `w * dt_frame` of simulated time, and that is
//  covered either by one analytic propagation (coasting above the air,
//  Part O3) or by `ceil(w * dt_frame / dt_phys)` substeps.
//
//  THE TWO PLACES A BIG dt IS A WRONG TRAJECTORY rather than a slow one,
//  and both are refused rather than approximated:
//
//    inside an atmosphere  -- warp 10 is the ceiling. At warp 100 a
//                             0.02 s tick is 5 s of flight through air
//                             whose density changes by the kilometre, and
//                             the ship teleports through the part of the
//                             descent where the heating happens.
//    with an engine burning -- warp 100, which is the largest warp the
//                             substep cap covers at a 60 Hz frame. A
//                             warp-1000 burn would need 834 ticks; the cap
//                             is 200, and dropping the warp is honest in a
//                             way that dropping the accuracy is not.
//
//  10,000 exists for one job: sitting at apoapsis 3 M km waiting for
//  Cheese to get there. It needs 100 km of altitude and no air, because
//  at 10,000 a frame is 167 simulated seconds and nothing that touches
//  the air should be moved that fast.

//  A COAST IS ALSO REFUSED WHEN ITS NEXT PERIAPSIS IS IN THE AIR
//  (F-34). Being above the air right now is not the test: a coast is
//  analytic, so a fast frame dives through an atmosphere it never
//  samples and comes out the other side -- a different trajectory,
//  not a coarse one. So a periapsis below the top of the atmosphere
//  (or below the ground, on an airless body) caps the frame at warp
//  10, which is the same answer KSP gives you when it greys out the
//  warp button.
//
//  STEERING COSTS THE SAME AS BURNING (F-31). A coast is analytic, which
//  means it applies no forces at all -- so a ship whose SAS is fighting a
//  rotation cannot coast, and the frame is paid for in ticks instead. The
//  alternative, a SAS that silently stops working the moment the engines
//  cut, is a bug a player would never be able to name.
//
//  PURE: no engine, no BSP. Host-compilable (Part H).
// =====================================================================

#include <stdbool.h>

#include "flight/physics.h"

// The ladder, and the tick cap (Part F1).
#define NISP_WARP_COUNT 6
#define NISP_SUBSTEP_MAX 200

float nisp_warp_level(int index);  // 1, 5, 10, 100, 1000, 10000
int   nisp_warp_levels(void);
int  nisp_warp_index_for(float warp);  // the nearest rung at or below

// What one frame is going to cost, and what it is allowed to ask for.
typedef struct {
    float warp;      // the warp actually used, after the refusals
    double dt_sim;   // simulated seconds this frame advances
    int   substeps;  // physics ticks it costs; 0 when the frame coasts
    bool  coasting;  // analytic: warp 1000 costs what warp 1 costs
    bool  capped;    // the substep cap bound, so the warp came down
    char  reason[32];  // why the next rung is not offered; empty when it is
} nisp_warp_step_t;

// `frame_dt` is the real time the frame is covering (1/60 on the badge),
// `thrusting` is whether an engine is lit and `steering` whether the control
// system is asking for torque (flight/control.h). A coast is only on the
// table when neither is. The vessel says where it is.
void nisp_warp_plan(nisp_vessel_t const* v, float requested, double frame_dt, bool thrusting,
                    bool steering, nisp_warp_step_t* out);

// The ceiling for a ship at this moment, as a warp value.
float nisp_warp_ceiling(nisp_vessel_t const* v, bool thrusting, bool steering);

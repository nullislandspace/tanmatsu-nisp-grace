#pragma once
// =====================================================================
//  Null Island Space Program  --  attitude, as a quaternion (Part V3, F1)
// ---------------------------------------------------------------------
//  A rigid body in this game has one attitude: which way its nose points.
//  Three Euler angles cannot hold that (they gimbal-lock at straight up or
//  down, which is exactly where a launch is), and a direction cosine matrix
//  is nine floats that have to be re-orthonormalised every tick. A unit
//  quaternion is four floats, drifts only in length, and is fixed by one
//  normalisation -- which is why it is what the physics tick stores.
//
//  THE ROTATION IS vehicle -> world, where "world" is the inertial frame of
//  whatever body's sphere of influence the vehicle is in (sim/units.h). The
//  vehicle's own axes are fixed by Part V3: local +z is the nose, so the
//  stack axis of rocket/stack -- measured from the nose toward the tail --
//  is local -z. A part at stack coordinate x sits at local (0, 0, -x).
//
//  PURE: no engine, no BSP (Part H).
// =====================================================================

#include <stdbool.h>

typedef struct {
    float w, x, y, z;
} nisp_quat_t;

void nisp_quat_identity(nisp_quat_t* q);
void nisp_quat_normalize(nisp_quat_t* q);

// out = a followed by b (that is, b applied to what a produced).
void nisp_quat_mul(nisp_quat_t const* a, nisp_quat_t const* b, nisp_quat_t* out);

// Vehicle -> world, and world -> vehicle.
void nisp_quat_rotate(nisp_quat_t const* q, float const v[3], float out[3]);
void nisp_quat_rotate_inv(nisp_quat_t const* q, float const v[3], float out[3]);

// A rotation of `angle` radians about a unit world axis.
void nisp_quat_from_axis_angle(float const axis[3], float angle, nisp_quat_t* out);

// The shortest rotation that takes local +z onto `dir` (unit). Roll about
// the nose is left to whatever the body was already doing, which is the
// honest answer: nothing in this game decides roll except RCS and the air.
void nisp_quat_from_dir(float const dir[3], nisp_quat_t* out);

// One integration step of q' = 0.5 * q x omega, with omega in vehicle axes.
// First order in dt, which at dt_phys 0.02 s and the slew rates an RCS can
// produce is some orders of magnitude below anything that matters, and the
// renormalisation is what keeps the drift from accumulating.
void nisp_quat_integrate(nisp_quat_t* q, float const omega[3], float dt);

// The axis/angle that rotates the vehicle's nose onto `target` (unit, world).
// The axis is in *vehicle* axes, which is what a torque command wants.
void nisp_quat_error(nisp_quat_t const* q, float const target[3], float axis[3], float* angle_rad);

// The vehicle's nose, in world axes.
void nisp_quat_nose(nisp_quat_t const* q, float out[3]);

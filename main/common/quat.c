// =====================================================================
//  Null Island Space Program  --  quaternion maths (see quat.h)
// =====================================================================

#include "common/quat.h"

#include <stddef.h>
#include <math.h>

#define QUAT_EPS 1e-6f

void nisp_quat_identity(nisp_quat_t* q) {
    if (q == NULL) return;
    q->w = 1.0f;
    q->x = q->y = q->z = 0.0f;
}

void nisp_quat_normalize(nisp_quat_t* q) {
    if (q == NULL) return;
    float const n = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (n < QUAT_EPS) {
        nisp_quat_identity(q);
        return;
    }
    float const k = 1.0f / n;
    q->w *= k;
    q->x *= k;
    q->y *= k;
    q->z *= k;
}

void nisp_quat_mul(nisp_quat_t const* a, nisp_quat_t const* b, nisp_quat_t* out) {
    if (a == NULL || b == NULL || out == NULL) return;
    nisp_quat_t r;
    r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    r.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    r.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    r.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
    *out = r;
}

// v' = q v q*, written out so it is one pass over nine multiplies.
void nisp_quat_rotate(nisp_quat_t const* q, float const v[3], float out[3]) {
    if (q == NULL || v == NULL || out == NULL) return;
    float const tx = 2.0f * (q->y * v[2] - q->z * v[1]);
    float const ty = 2.0f * (q->z * v[0] - q->x * v[2]);
    float const tz = 2.0f * (q->x * v[1] - q->y * v[0]);
    out[0] = v[0] + q->w * tx + (q->y * tz - q->z * ty);
    out[1] = v[1] + q->w * ty + (q->z * tx - q->x * tz);
    out[2] = v[2] + q->w * tz + (q->x * ty - q->y * tx);
}

void nisp_quat_rotate_inv(nisp_quat_t const* q, float const v[3], float out[3]) {
    if (q == NULL) return;
    nisp_quat_t const c = {q->w, -q->x, -q->y, -q->z};
    nisp_quat_rotate(&c, v, out);
}

void nisp_quat_from_axis_angle(float const axis[3], float angle, nisp_quat_t* out) {
    if (axis == NULL || out == NULL) return;
    float const n = sqrtf(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (n < QUAT_EPS || angle == 0.0f) {
        nisp_quat_identity(out);
        return;
    }
    float const k = 1.0f / n;
    float const h = 0.5f * angle;
    out->w = cosf(h);
    out->x = axis[0] * k * sinf(h);
    out->y = axis[1] * k * sinf(h);
    out->z = axis[2] * k * sinf(h);
}

// Local +z is (0, 0, 1), so the rotation wanted is the one that carries
// (0,0,1) onto `dir`: an axis perpendicular to both, by the angle between
// them. The two degenerate cases are the two ends of the nose axis, and
// both get an explicit answer rather than a division by a zero cross
// product.
void nisp_quat_from_dir(float const dir[3], nisp_quat_t* out) {
    if (dir == NULL || out == NULL) return;
    float const n = sqrtf(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (n < QUAT_EPS) {
        nisp_quat_identity(out);
        return;
    }
    float const d[3] = {dir[0] / n, dir[1] / n, dir[2] / n};

    if (d[2] > 0.999999f) {
        nisp_quat_identity(out);
        return;
    }
    if (d[2] < -0.999999f) {  // nose exactly backwards: a half turn about +x
        out->w = 0.0f;
        out->x = 1.0f;
        out->y = out->z = 0.0f;
        return;
    }

    // axis = z x d, and the angle between them. sin(h) and cos(h) straight
    // from the half-angle identities, so there is no atan2 and no acos.
    float const ax = -d[1], ay = d[0];
    float const an = sqrtf(ax * ax + ay * ay);
    float const h = 0.5f * atan2f(an, d[2]);
    float const k = 1.0f / an;
    out->w = cosf(h);
    out->x = ax * k * sinf(h);
    out->y = ay * k * sinf(h);
    out->z = 0.0f;
}

void nisp_quat_integrate(nisp_quat_t* q, float const omega[3], float dt) {
    if (q == NULL || omega == NULL || dt <= 0.0f) return;
    // q + 0.5 * dt * q x (0, omega), then back onto the unit shell.
    float const hw = -0.5f * dt * (q->x * omega[0] + q->y * omega[1] + q->z * omega[2]);
    float const hx = 0.5f * dt * (q->w * omega[0] + q->y * omega[2] - q->z * omega[1]);
    float const hy = 0.5f * dt * (q->w * omega[1] + q->z * omega[0] - q->x * omega[2]);
    float const hz = 0.5f * dt * (q->w * omega[2] + q->x * omega[1] - q->y * omega[0]);
    q->w += hw;
    q->x += hx;
    q->y += hy;
    q->z += hz;
    nisp_quat_normalize(q);
}

void nisp_quat_error(nisp_quat_t const* q, float const target[3], float axis[3], float* angle_rad) {
    float nose[3];
    nisp_quat_nose(q, nose);

    float const tn = sqrtf(target[0] * target[0] + target[1] * target[1] + target[2] * target[2]);
    if (tn < QUAT_EPS) {
        axis[0] = axis[1] = axis[2] = 0.0f;
        if (angle_rad != NULL) *angle_rad = 0.0f;
        return;
    }
    float const t[3] = {target[0] / tn, target[1] / tn, target[2] / tn};

    float c = nose[0] * t[0] + nose[1] * t[1] + nose[2] * t[2];
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;

    float cx = nose[1] * t[2] - nose[2] * t[1];
    float cy = nose[2] * t[0] - nose[0] * t[2];
    float cz = nose[0] * t[1] - nose[1] * t[0];
    float const cn = sqrtf(cx * cx + cy * cy + cz * cz);

    if (cn < QUAT_EPS) {
        // Aligned, or exactly opposed. Opposed needs *some* axis, and any
        // axis perpendicular to the nose will do; +x is as good as any.
        axis[0] = 1.0f;
        axis[1] = axis[2] = 0.0f;
        if (angle_rad != NULL) *angle_rad = (c < 0.0f) ? 3.1415927f : 0.0f;
        return;
    }

    // The axis has to be in vehicle axes for a torque command to use it.
    float const w[3] = {cx / cn, cy / cn, cz / cn};
    nisp_quat_rotate_inv(q, w, axis);
    if (angle_rad != NULL) *angle_rad = atan2f(cn, c);
}

void nisp_quat_nose(nisp_quat_t const* q, float out[3]) {
    float const v[3] = {0.0f, 0.0f, 1.0f};
    nisp_quat_rotate(q, v, out);
}

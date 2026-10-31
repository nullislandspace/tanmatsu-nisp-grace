// =====================================================================
//  Null Island Space Program  --  keys and SAS into torque (see control.h)
// =====================================================================

#include "flight/control.h"

#include <math.h>
#include <stddef.h>

#include "common/quat.h"
#include "flight/physics.h"
#include "rocket/parts.h"

static char const* const SAS_NAMES[NISP_SAS_COUNT] = {
    "off", "hold", "prograde", "retrograde", "normal", "radial out", "radial in", "burn",
};

char const* nisp_sas_name(int mode) {
    if (mode < 0 || mode >= NISP_SAS_COUNT) return "?";
    return SAS_NAMES[mode];
}

void nisp_control_init(nisp_control_t* c) {
    if (c == NULL) return;
    c->throttle = 0.0f;
    c->cut = false;
    c->stage = false;
    c->chute = false;
    c->legs = false;
    c->sas = NISP_SAS_OFF;
    c->rcs = true;
    c->torque[0] = c->torque[1] = c->torque[2] = 0.0f;
    c->warp = 1.0f;
}

int nisp_control_rcs_modules(nisp_stack_t const* s) {
    if (s == NULL) return 0;

    // A module counts while the stage it is bolted to is still part of this
    // fragment: staging and a burn-through both take its modules with the
    // stage they remove (Part F5).
    bool attached[NISP_DESIGN_STAGE_MAX] = {false};
    for (int i = 0; i < s->count; i++) {
        int const stage = s->part[i].stage;
        if (stage >= 0 && stage < NISP_DESIGN_STAGE_MAX) attached[stage] = true;
    }
    int n = 0;
    for (int i = 0; i < s->stage_count && i < NISP_DESIGN_STAGE_MAX; i++) {
        if (s->rcs[i] && attached[i]) n++;
    }
    return n;
}

float nisp_control_authority_knm(nisp_stack_t const* s, bool rcs_on) {
    if (s == NULL || !rcs_on) return 0.0f;
    return NISP_RCS_TORQUE_KNM * (float)nisp_control_rcs_modules(s);
}

bool nisp_control_active(nisp_stack_t const* s, nisp_control_t const* ctl) {
    if (ctl == NULL) return false;
    if (nisp_control_authority_knm(s, ctl->rcs) <= 0.0f) return false;
    if (ctl->sas != NISP_SAS_OFF) return true;
    return ctl->torque[0] != 0.0f || ctl->torque[1] != 0.0f || ctl->torque[2] != 0.0f;
}

static float mag3f(float const v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

static void clamp_mag(float v[3], float max_mag) {
    float const m = mag3f(v);
    if (m > max_mag && m > 0.0f) {
        float const k = max_mag / m;
        for (int i = 0; i < 3; i++) v[i] *= k;
    }
}

void nisp_control_torque(nisp_vessel_t const* v, nisp_control_t const* ctl, float dt_s, float out[3]) {
    if (out == NULL) return;
    out[0] = out[1] = out[2] = 0.0f;
    if (v == NULL || ctl == NULL) return;

    // No RCS on the stage in charge means no control torque, full stop:
    // what is left is the air's opinion (Part A5) and the player's staging.
    float const authority = nisp_control_authority_knm(&v->stack, ctl->rcs);
    if (authority <= 0.0f) return;

    float const it = v->stack.inertia_t_m2 > 1e-4f ? v->stack.inertia_t_m2 : 1e-4f;

    float cmd[3] = {0.0f, 0.0f, 0.0f};

    if (ctl->sas == NISP_SAS_OFF) {
        // Manual: the keys are the torque, and the budget is all there is.
        for (int i = 0; i < 3; i++) cmd[i] = ctl->torque[i];
    } else {
        float target[3];
        if (nisp_vessel_reference(v, ctl->sas, target)) {
            // SAS COMMANDS A ROTATION RATE, not a spring force (F-36).
            //
            // A spring/damper on the angle looks right on paper and is
            // wrong here, because the RCS budget is nothing like the torque
            // a spring wants. 1,200 kN*m/rad on a 30 degree error asks for
            // 628 kN*m; a three-stage stack has 45. The command saturates,
            // the ship slews at full authority, the angle term collapses on
            // the way to the target while the ship is still rotating at
            // 0.13 rad/s, and the damping that is left is not enough to
            // stop it -- so it overshoots and swings back forever. That is
            // a limit cycle, and it is what F-30 looked like at scale.
            //
            // The fix is the switching law a real RCS runs: the budget and
            // the inertia say the biggest angular acceleration there is
            // (alpha_max), which says the fastest rate that can still be
            // arrested before the target is overshot (sqrt(2 * alpha *
            // angle)). Command that rate -- or the gentle one, angle over
            // the approach time, whichever is slower -- and chase it. Full
            // authority on the way up, full authority on the way down, and
            // no overshoot to damp because the profile never asks for a
            // rate it cannot stop.
            float axis[3];
            float angle = 0.0f;
            nisp_quat_error(&v->q, target, axis, &angle);

            float const alpha_max = authority / it;  // kN*m / t*m^2 == rad/s^2
            float const w_arrest = sqrtf(2.0f * alpha_max * angle);
            float const w_cruise = angle / NISP_SAS_APPROACH_S;
            float const w_cmd = w_arrest < w_cruise ? w_arrest : w_cruise;

            // The rate loop acts on all three body axes, so a rotation that
            // has nothing to do with the error still gets damped. HOLD aims
            // at the nose it already has, so its commanded rate is zero and
            // it does nothing but stop the rotation -- which is what a
            // stability assist that holds a heading actually is.
            float err[3];
            for (int i = 0; i < 3; i++) err[i] = w_cmd * axis[i] - v->omega[i];
            for (int i = 0; i < 3; i++) cmd[i] = it * NISP_SAS_RATE_GAIN * err[i];

            // A rate loop is a damper, and a damper may remove a rotation
            // and may not reverse it in one tick (F-30). The stiffness that
            // is right for a 20 ms physics tick is four times too much for
            // the 0.5 s attitude step a coast uses, so the torque is capped
            // at what this dt can spend on this rate error.
            float const dt = dt_s > 1e-4f ? dt_s : 1e-4f;
            clamp_mag(cmd, it * mag3f(err) / dt);
        }
    }

    // The budget is a magnitude, not a per-axis allowance: a SAS that is
    // fighting hard in one axis has nothing left for the others, which is
    // the way a real RCS runs out (Part F2).
    clamp_mag(cmd, authority);

    out[0] = cmd[0];
    out[1] = cmd[1];
    out[2] = cmd[2];
}

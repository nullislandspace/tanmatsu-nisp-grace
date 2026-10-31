#include <math.h>
#include <string.h>

#include "sim/ephemeris.h"
#include "sim/patches.h"

// =====================================================================
//  Leaving a sphere of influence is solved; entering one is searched.
//
//  Leaving is easy because r(nu) = p / (1 + e cos nu) inverts in one line:
//  the true anomaly where the conic crosses the sphere is acos((p/soi - 1)/e),
//  and the time to get there is one Kepler solve. No stepping, no drift.
//
//  Entering someone else's sphere is not, because "will I run into Cheese"
//  depends on where Cheese will be, and a conic about Bob does not know
//  that. So the map scans the leg against the moving body and bisects the
//  interval where the distance crosses the sphere. It is a search over an
//  exact answer, which is the honest way to answer a three-body question
//  with a two-body propagator.
// =====================================================================

bool nisp_soi_exit_true_anomaly(nisp_elements_t const* el, double* nu_out) {
    nisp_body_t const* b = nisp_body(el->body);
    double const soi = b->soi_km;
    if (soi <= 0.0) return false;              // the root never hands over
    if (el->e < 1e-9) return false;            // a circle never crosses anything

    double const c = (el->p_km / soi - 1.0) / el->e;
    if (c > 1.0) return false;                 // apoapsis is inside the sphere
    if (c < -1.0) return false;                // periapsis is outside it
    if (nu_out) *nu_out = acos(c);
    return true;
}

// The next time at or after `t_after` that the conic is at true anomaly `nu`.
static double forward_time(nisp_elements_t const* el, double nu, double t_after) {
    double t = nisp_time_at_true(el, nu);
    if (el->e < 1.0 && el->period_s > 0.0) {
        while (t <= t_after + 1e-9) t += el->period_s;
    }
    return t;
}

bool nisp_soi_entry(nisp_elements_t const* el, double t0, double horizon_s,
                    int* hit_body, double* t_hit, double* r_hit) {
    if (horizon_s <= 0.0) return false;

    int const base = el->body;
    double best_t = t0 + horizon_s;
    int best = BODY_COUNT;

    for (int cand = 0; cand < BODY_COUNT; cand++) {
        if (cand == base) continue;
        nisp_body_t const* cb = nisp_body(cand);
        if (cb->soi_km <= 0.0f) continue;

        nisp_state_t s;
        double bp[3], cp[3];
        nisp_elements_to_state(el, t0, &s);
        nisp_body_position(base, t0, bp);
        nisp_body_position(cand, t0, cp);
        double prev[3] = {bp[0] + s.r[0], bp[1] + s.r[1], bp[2] + s.r[2]};
        double prev_d = sqrt((prev[0] - cp[0]) * (prev[0] - cp[0]) +
                             (prev[1] - cp[1]) * (prev[1] - cp[1]) +
                             (prev[2] - cp[2]) * (prev[2] - cp[2]));
        double prev_t = t0;

        for (int i = 1; i <= NISP_PATCH_SCAN; i++) {
            double const t = t0 + horizon_s * i / (double)NISP_PATCH_SCAN;
            nisp_elements_to_state(el, t, &s);
            nisp_body_position(base, t, bp);
            nisp_body_position(cand, t, cp);
            double const px = bp[0] + s.r[0], py = bp[1] + s.r[1], pz = bp[2] + s.r[2];
            double const d = sqrt((px - cp[0]) * (px - cp[0]) +
                                  (py - cp[1]) * (py - cp[1]) +
                                  (pz - cp[2]) * (pz - cp[2]));

            if (prev_d > cb->soi_km && d <= cb->soi_km) {
                // Bisect the interval. 40 halvings is 1e-12 of the interval,
                // which is far past the point where the endpoints differ.
                double lo = prev_t, hi = t;
                for (int k = 0; k < 40; k++) {
                    double const mid = 0.5 * (lo + hi);
                    nisp_elements_to_state(el, mid, &s);
                    nisp_body_position(base, mid, bp);
                    nisp_body_position(cand, mid, cp);
                    double const mx = bp[0] + s.r[0], my = bp[1] + s.r[1], mz = bp[2] + s.r[2];
                    double const md = sqrt((mx - cp[0]) * (mx - cp[0]) +
                                           (my - cp[1]) * (my - cp[1]) +
                                           (mz - cp[2]) * (mz - cp[2]));
                    if (md > cb->soi_km) lo = mid; else hi = mid;
                }
                if (hi < best_t) { best_t = hi; best = cand; }
                break;
            }
            prev_d = d;
            prev_t = t;
        }
    }

    if (best >= BODY_COUNT) return false;

    // The state at the boundary, relative to the body being entered.
    nisp_state_t s;
    nisp_elements_to_state(el, best_t, &s);
    double bp[3], cp[3], bv[3], cv[3];
    nisp_body_position(base, best_t, bp);
    nisp_body_position(best, best_t, cp);
    nisp_body_velocity(base, best_t, bv);
    nisp_body_velocity(best, best_t, cv);
    for (int i = 0; i < 3; i++) {
        double const rel = bp[i] + s.r[i] - cp[i];
        if (r_hit) r_hit[i] = rel;
    }
    if (hit_body) *hit_body = best;
    if (t_hit) *t_hit = best_t;
    return true;
}

// Move a state from being relative to `from` to being relative to `to`.
static void handover(int from, int to, double t, nisp_state_t const* rel, nisp_state_t* out) {
    double fp[3], tp[3], fv[3], tv[3];
    nisp_body_position(from, t, fp);
    nisp_body_position(to, t, tp);
    nisp_body_velocity(from, t, fv);
    nisp_body_velocity(to, t, tv);
    for (int i = 0; i < 3; i++) {
        out->r[i] = (float)(fp[i] + rel->r[i] - tp[i]);
        out->v[i] = (float)(fv[i] + rel->v[i] - tv[i]);
    }
}

int nisp_patch_predict(nisp_state_t const* state, int body, double t, double horizon_s,
                       nisp_patch_t* out, int max) {
    if (max > NISP_PATCH_MAX) max = NISP_PATCH_MAX;
    if (max < 1) return 0;
    if (horizon_s <= 0.0) horizon_s = 1.0;

    double const t_limit = t + horizon_s;

    nisp_elements_t el;
    if (!nisp_state_to_elements(state, body, t, &el)) {
        memset(out, 0, sizeof(*out));
        out[0].body = body;
        out[0].t_enter = t;
        out[0].t_exit = t_limit;
        out[0].end = NISP_PATCH_END_LIMIT;
        out[0].other = BODY_COUNT;
        out[0].cut = true;
        for (int i = 0; i < 3; i++) {
            out[0].r_end[i] = state->r[i];
            out[0].v_end[i] = state->v[i];
        }
        return 1;
    }

    double t_cur = t;
    nisp_state_t cur = *state;
    int n = 0;

    while (n < max) {
        double t_exit = t_limit;
        int end = NISP_PATCH_END_LIMIT;
        int other = BODY_COUNT;

        double nu;
        if (nisp_soi_exit_true_anomaly(&el, &nu)) {
            double const tc = forward_time(&el, nu, t_cur);
            if (tc < t_exit) {
                t_exit = tc;
                end = NISP_PATCH_END_SOI;
                other = nisp_body(el.body)->parent;
            }
        }

        int hit;
        double t_hit;
        double r_hit[3];
        if (nisp_soi_entry(&el, t_cur, t_exit - t_cur, &hit, &t_hit, r_hit)) {
            if (t_hit < t_exit) {
                t_exit = t_hit;
                end = NISP_PATCH_END_ENTRY;
                other = hit;
            }
        }

        if (t_exit <= t_cur) t_exit = t_cur + 1e-3;  // never a zero-length leg

        nisp_state_t at_exit;
        nisp_elements_to_state(&el, t_exit, &at_exit);

        out[n].body    = el.body;
        out[n].el      = el;
        out[n].t_enter = t_cur;
        out[n].t_exit  = t_exit;
        out[n].end     = end;
        out[n].other   = other;
        out[n].cut     = (end == NISP_PATCH_END_LIMIT);
        for (int i = 0; i < 3; i++) {
            out[n].r_end[i] = at_exit.r[i];
            out[n].v_end[i] = at_exit.v[i];
        }
        n++;

        if (end == NISP_PATCH_END_LIMIT || other >= BODY_COUNT) break;

        nisp_state_t next;
        handover(el.body, other, t_exit, &at_exit, &next);
        if (!nisp_state_to_elements(&next, other, t_exit, &el)) break;
        t_cur = t_exit;
        cur = next;
    }

    (void)cur;
    return n;
}

bool nisp_patch_state(nisp_patch_t const* chain, int count, double t, nisp_state_t* out) {
    for (int i = 0; i < count; i++) {
        if (t >= chain[i].t_enter && t <= chain[i].t_exit) {
            nisp_elements_to_state(&chain[i].el, t, out);
            return true;
        }
    }
    return false;
}

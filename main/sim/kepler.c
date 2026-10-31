#include <math.h>
#include <string.h>

#include "sim/kepler.h"

// =====================================================================
//  Kepler's equation, twice (Part O3)
//
//  E - e sin E = M for an ellipse, H - e sinh H = M for an escape. Both
//  are Newton on a monotone function, both seeded from something already
//  close, and both converge in a handful of iterations across the whole
//  range this game can produce.
//
//  The elliptic seed is M itself. For e near 1 and M near 0 that seed is
//  poor -- but four Newton steps from it land within 1e-10 rad at e=0.99,
//  and 0.99 is far past anything a rocket here produces.
//
//  The hyperbolic seed is the large-M asymptote, M ~ e sinh H - H giving
//  H ~ asinh((M + e)/e). A flyby trajectory spends almost all of its
//  simulated time near periapsis, where M is small and the seed is nearly
//  exact, and the rest far away, where the asymptote is nearly exact.
//
//  asinh() itself is not in the graceloader export table (sinh, cosh, exp
//  and log are), so it is built here out of log and sqrt. Mirroring
//  through the odd symmetry keeps log(a + sqrt(a*a+1)) from cancelling
//  away for negative arguments.
// =====================================================================

static double hyper_asinh(double x) {
    if (x != x) return x;
    double const a = x < 0.0 ? -x : x;
    double const r = log(a + sqrt(a * a + 1.0));
    return x < 0.0 ? -r : r;
}

double nisp_kepler_elliptic(double M, double e) {
    if (e < 0.0) e = 0.0;
    if (e > 0.999999) e = 0.999999;

    double E = M;
    for (int i = 0; i < 4; i++) {
        double const f  = E - e * sin(E) - M;
        double const fp = 1.0 - e * cos(E);
        double const d  = f / (fp != 0.0 ? fp : 1e-12);
        E -= d;
        if (fabs(d) < 1e-12) break;
    }
    // A fifth guarded step catches the rare start where Newton overshoots.
    for (int i = 0; i < 6; i++) {
        double const f  = E - e * sin(E) - M;
        if (fabs(f) < 1e-12) break;
        double const fp = 1.0 - e * cos(E);
        E -= f / (fp != 0.0 ? fp : 1e-12);
    }
    return E;
}

double nisp_kepler_hyperbolic(double M, double e) {
    if (e < 1.000001) e = 1.000001;

    double H = hyper_asinh((M + e) / e);
    for (int i = 0; i < 8; i++) {
        double const f  = e * sinh(H) - H - M;
        double const fp = e * cosh(H) - 1.0;
        double const d  = f / (fp != 0.0 ? fp : 1e-12);
        H -= d;
        if (fabs(d) < 1e-12) break;
    }
    return H;
}

// --- Mean anomaly ------------------------------------------------------

static double mean_motion(nisp_elements_t const* el) {
    double const a = fabs(el->a);
    if (a <= 0.0 || el->mu <= 0.0) return 0.0;
    return sqrt(el->mu / (a * a * a));
}

double nisp_mean_at(nisp_elements_t const* el, double t) {
    double M = el->m0 + mean_motion(el) * (t - el->t0);
    if (el->e < 1.0) {
        M = fmod(M, 2.0 * M_PI);
        if (M > M_PI) M -= 2.0 * M_PI;
        if (M < -M_PI) M += 2.0 * M_PI;
    }
    return M;
}

// --- Anomaly <-> true anomaly ------------------------------------------

double nisp_true_at(nisp_elements_t const* el, double t) {
    double const M = nisp_mean_at(el, t);
    if (el->e < 1.0) {
        double const E = nisp_kepler_elliptic(M, el->e);
        return atan2(sqrt(1.0 - el->e * el->e) * sin(E), cos(E) - el->e);
    }
    double const H = nisp_kepler_hyperbolic(M, el->e);
    return atan2(sqrt(el->e * el->e - 1.0) * sinh(H), el->e - cosh(H));
}

double nisp_time_at_true(nisp_elements_t const* el, double nu) {
    double M;
    if (el->e < 1.0) {
        double const E = atan2(sqrt(1.0 - el->e * el->e) * sin(nu), el->e + cos(nu));
        M = E - el->e * sin(E);
    } else {
        double denom = 1.0 + el->e * cos(nu);
        if (denom <= 1e-9) denom = 1e-9;  // past the asymptote: no such point
        double const sh = sqrt(el->e * el->e - 1.0) * sin(nu) / denom;
        double const H = hyper_asinh(sh);
        M = el->e * sinh(H) - H;
    }

    double n = mean_motion(el);
    if (n <= 0.0) return el->t0;

    double dt = (M - el->m0) / n;
    if (el->e < 1.0) {  // the nearest passage, not the first one ever
        double const period = 2.0 * M_PI / n;
        while (dt < -0.5 * period) dt += period;
        while (dt > 0.5 * period) dt -= period;
    }
    return el->t0 + dt;
}

double nisp_time_to_apsis(nisp_elements_t const* el, double t, bool apoapsis) {
    double const target = apoapsis ? M_PI : 0.0;
    double M;
    if (el->e < 1.0) {
        M = nisp_mean_at(el, t);
        double n = mean_motion(el);
        if (n <= 0.0) return 0.0;
        double dt = (target - M) / n;
        double const period = 2.0 * M_PI / n;
        while (dt < 0.0) dt += period;
        return dt;
    }
    // A hyperbola passes periapsis once. Apoapsis does not exist.
    if (apoapsis) return -1.0;
    double n = mean_motion(el);
    if (n <= 0.0) return 0.0;
    double const M_now = nisp_mean_at(el, t);
    return (0.0 - M_now) / n;
}

// --- The perifocal frame -----------------------------------------------

void nisp_perifocal(nisp_elements_t const* el, nisp_perifocal_t* pf) {
    // Node line: the intersection of the orbit plane with X-Z, at angle
    // `lan` from +x measured about +y.
    double const N[3] = {cos(el->lan), 0.0, -sin(el->lan)};

    // Angular momentum: +y tipped by `inc` about the node line.
    double const s = sin(el->inc), c = cos(el->inc);
    double const Hv[3] = {sin(el->lan) * s, c, cos(el->lan) * s};

    // Periapsis: the node line turned by `aop` in the orbit plane. The
    // Rodrigues term along H vanishes because H is perpendicular to N.
    double const cross[3] = {Hv[1] * N[2] - Hv[2] * N[1],
                             Hv[2] * N[0] - Hv[0] * N[2],
                             Hv[0] * N[1] - Hv[1] * N[0]};
    double const sa = sin(el->aop), ca = cos(el->aop);
    double const P[3] = {N[0] * ca + cross[0] * sa,
                         N[1] * ca + cross[1] * sa,
                         N[2] * ca + cross[2] * sa};

    pf->H[0] = Hv[0]; pf->H[1] = Hv[1]; pf->H[2] = Hv[2];
    pf->P[0] = P[0];  pf->P[1] = P[1];  pf->P[2] = P[2];
    // Q is a quarter turn ahead along the motion: h x p, which is the
    // direction a prograde orbit travels 90 degrees after periapsis.
    pf->Q[0] = pf->H[1] * pf->P[2] - pf->H[2] * pf->P[1];
    pf->Q[1] = pf->H[2] * pf->P[0] - pf->H[0] * pf->P[2];
    pf->Q[2] = pf->H[0] * pf->P[1] - pf->H[1] * pf->P[0];
}

// --- Elements -> state -------------------------------------------------

void nisp_elements_to_state(nisp_elements_t const* el, double t, nisp_state_t* out) {
    nisp_perifocal_t pf;
    nisp_perifocal(el, &pf);

    double const nu = nisp_true_at(el, t);
    double const cn = cos(nu), sn = sin(nu);
    double denom = 1.0 + el->e * cn;
    if (denom < 1e-9) denom = 1e-9;
    double const r = el->p_km / denom;
    double const sp = sqrt(el->mu / el->p_km);

    for (int i = 0; i < 3; i++) {
        out->r[i] = (float)(r * (cn * pf.P[i] + sn * pf.Q[i]));
        out->v[i] = (float)(sp * (-sn * pf.P[i] + (el->e + cn) * pf.Q[i]));
    }
}

// --- State -> elements -------------------------------------------------

void nisp_elements_derive(nisp_elements_t* el) {
    // A parabolic orbit is the seam between the two branches, and the
    // anomaly solvers both misbehave exactly there. Nudge it to whichever
    // side its own energy is on rather than pretending it is neither.
    if (fabs(el->e - 1.0) < 1e-6) el->e = (el->a >= 0.0) ? (1.0 - 1e-6) : (1.0 + 1e-6);

    el->p_km      = fabs(el->a) * fabs(1.0 - el->e * el->e);
    // Both branches measure periapsis the same way: |a| times the distance
    // from the focus to the vertex. On a hyperbola a is negative and
    // (1 - e) is negative, and the two signs cancel.
    el->periapsis_km = fabs(el->a) * fabs(1.0 - el->e);
    if (el->e < 1.0) {
        el->apoapsis_km = el->a * (1.0 + el->e);
        el->period_s    = 2.0 * M_PI * sqrt(el->a * el->a * el->a / el->mu);
    } else {
        el->apoapsis_km = 0.0;  // there is none
        el->period_s    = 0.0;
    }
}

bool nisp_state_to_elements(nisp_state_t const* state, int body, double t, nisp_elements_t* out) {
    double const rx = state->r[0], ry = state->r[1], rz = state->r[2];
    double const vx = state->v[0], vy = state->v[1], vz = state->v[2];

    double const r = sqrt(rx * rx + ry * ry + rz * rz);
    double const v2 = vx * vx + vy * vy + vz * vz;
    if (r < 1e-6 || v2 < 1e-12) return false;

    double const mu = nisp_body(body)->mu;

    // h = r x v
    double const hx = ry * vz - rz * vy;
    double const hy = rz * vx - rx * vz;
    double const hz = rx * vy - ry * vx;
    double const h = sqrt(hx * hx + hy * hy + hz * hz);
    if (h < 1e-9) return false;  // a radial trajectory defines no plane

    double const energy = 0.5 * v2 - mu / r;
    out->body = body;
    out->mu   = mu;
    out->t0   = t;
    out->a    = (fabs(energy) < 1e-12) ? -mu / (1e-12) : -mu / (2.0 * energy);
    out->p_km = h * h / mu;
    // e from the energy and the angular momentum. p = h^2/mu, so the
    // usual 1 + 2*eps*h^2/mu^2 is 1 + 2*eps*p/mu here -- one division,
    // and no chance of squaring the wrong quantity.
    // The argument is clamped before the square root, not the result after
    // it (F-33). A circular orbit is the case: its 1 + 2*eps*p/mu is exactly
    // zero in theory and -1e-15 in float, sqrt of that is NaN, and a NaN
    // compares false against 0.0 -- so the guard that used to sit here never
    // fired and the NaN propagated a whole flight instead.
    double e2 = 1.0 + 2.0 * energy * out->p_km / mu;
    if (!(e2 > 0.0)) e2 = 0.0;
    out->e = sqrt(e2);

    // Inclination from +y, the reference normal.
    double sh = hy / h;
    if (sh > 1.0) sh = 1.0;
    if (sh < -1.0) sh = -1.0;
    double inc = acos(sh);
    if (inc < 0.0) inc = 0.0;
    if (inc > M_PI) inc = M_PI;
    out->inc = inc;

    // Node line = y x h. Straight up is the degenerate case: no node, so
    // measure the periapsis angle from +x instead.
    double nx = hz, nz = -hx;
    double nlen = sqrt(nx * nx + nz * nz);
    double lan = 0.0;
    double N[3] = {1.0, 0.0, 0.0};
    if (nlen > 1e-9) {
        nx /= nlen; nz /= nlen;
        lan = atan2(hx, hz);  // N = (cos lan, 0, -sin lan)
        N[0] = nx; N[2] = nz;
    }
    out->lan = lan;

    // Eccentricity vector: points at periapsis, magnitude e.
    double const cx = vy * hz - vz * hy;
    double const cy = vz * hx - vx * hz;
    double const cz = vx * hy - vy * hx;
    double ex = cx / mu - rx / r;
    double ey = cy / mu - ry / r;
    double ez = cz / mu - rz / r;

    // Q0 = H x N, the in-plane quarter turn ahead of the node line.
    double const Hn[3] = {hx / h, hy / h, hz / h};
    double const q0[3] = {Hn[1] * N[2] - Hn[2] * N[1],
                          Hn[2] * N[0] - Hn[0] * N[2],
                          Hn[0] * N[1] - Hn[1] * N[0]};
    double const along = ex * N[0] + ey * N[1] + ez * N[2];
    double const ahead = ex * q0[0] + ey * q0[1] + ez * q0[2];
    // The same formula in both branches. An equatorial orbit has no node
    // line, but N defaults to +x and q0 = H x N is still the in-plane
    // quarter turn, so the angle is still measured in the orbit plane. The
    // tempting shortcut -- atan2(ey, ex) -- reads the eccentricity vector
    // out of the X-Y plane instead of the X-Z one and silently puts the
    // periapsis 90 degrees away (F-20).
    out->aop = atan2(ahead, along);

    double const cross[3] = {Hn[1] * N[2] - Hn[2] * N[1],
                             Hn[2] * N[0] - Hn[0] * N[2],
                             Hn[0] * N[1] - Hn[1] * N[0]};
    double const sa = sin(out->aop), ca = cos(out->aop);

    // Where the ship actually is, measured from periapsis. P is the
    // periapsis direction just derived and Q a quarter turn ahead of it,
    // so this is the same pair nisp_elements_to_state writes from.
    double const P[3] = {N[0] * ca + cross[0] * sa,
                         N[1] * ca + cross[1] * sa,
                         N[2] * ca + cross[2] * sa};
    double const Q[3] = {Hn[1] * P[2] - Hn[2] * P[1],
                         Hn[2] * P[0] - Hn[0] * P[2],
                         Hn[0] * P[1] - Hn[1] * P[0]};
    double const nu = atan2(rx * Q[0] + ry * Q[1] + rz * Q[2],
                            rx * P[0] + ry * P[1] + rz * P[2]);
    if (out->e < 1.0) {
        double const E = atan2(sqrt(1.0 - out->e * out->e) * sin(nu), out->e + cos(nu));
        out->m0 = E - out->e * sin(E);
    } else {
        double const denom = 1.0 + out->e * cos(nu);
        double const sh = sqrt(out->e * out->e - 1.0) * sin(nu) / (denom > 1e-9 ? denom : 1e-9);
        double const Hh = hyper_asinh(sh);
        out->m0 = out->e * sinh(Hh) - Hh;
    }

    nisp_elements_derive(out);
    return true;
}

void nisp_elements_circular(int body, float r_km, double nu0, double t0, nisp_elements_t* out) {
    memset(out, 0, sizeof(*out));
    out->body = body;
    out->mu   = nisp_body(body)->mu;
    out->a    = r_km;
    out->e    = 0.0;
    out->inc  = 0.0;
    out->lan  = 0.0;
    out->aop  = 0.0;
    out->t0   = t0;
    out->m0   = nu0;
    nisp_elements_derive(out);
}

float nisp_conic_speed(nisp_elements_t const* el, float r_km) {
    if (r_km <= 0.0f) return 0.0f;
    double const v2 = el->mu * (2.0 / r_km - 1.0 / el->a);
    return (v2 > 0.0) ? (float)sqrt(v2) : 0.0f;
}

float nisp_circularisation_delta_v(nisp_elements_t const* el, float r_km) {
    if (r_km <= 0.0f) return 0.0f;
    return (float)(sqrt(el->mu / r_km) - nisp_conic_speed(el, r_km));
}

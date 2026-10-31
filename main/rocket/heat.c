// =====================================================================
//  Null Island Space Program  --  heating, one float per part (see heat.h)
// =====================================================================

#include "rocket/heat.h"

#include <math.h>
#include <stddef.h>

#include "sim/atmosphere.h"

float nisp_heat_load_rate(int body, float alt_km, float v_rel_kms) {
    float const rho = nisp_atm_density(body, alt_km);
    if (rho <= 0.0f) return 0.0f;
    float const v = v_rel_kms * 1000.0f;  // km/s -> m/s
    return rho * v * v * v;
}

float nisp_heat_expose(nisp_part_t const* p, float cos_flow_axis) {
    if (p->shield <= 0.0f) return 1.0f;
    float facing = cos_flow_axis;
    if (facing < 0.0f) facing = 0.0f;  // shield facing away protects nothing
    if (facing > 1.0f) facing = 1.0f;
    return 1.0f - p->shield * facing;
}

void nisp_heat_tick(nisp_stack_t* s, nisp_heat_input_t const* in, float dt_s) {
    if (s == NULL || dt_s <= 0.0f) return;

    float q3 = 0.0f;
    if (in != NULL) q3 = nisp_heat_load_rate(in->body, in->alt_km, in->v_rel_kms);

    for (int i = 0; i < s->count; i++) {
        nisp_part_t* p = &s->part[i];
        if (p->capacity <= 0.0f) continue;  // a drag plate, not a part

        float rate = 0.0f;
        if (q3 > 0.0f) {
            rate = (float)NISP_HEAT_K * q3 * p->area_m2 * nisp_heat_expose(p, in->cos_flow_axis) / p->capacity;
        }

        p->heat += (rate - p->heat / (float)NISP_HEAT_TAU) * dt_s;
        if (p->heat < 0.0f) p->heat = 0.0f;
    }
}

int nisp_heat_breach(nisp_stack_t const* s) {
    if (s == NULL) return -1;

    int best = -1;
    float best_over = 0.0f;
    for (int i = 0; i < s->count; i++) {
        nisp_part_t const* p = &s->part[i];
        if (p->capacity <= 0.0f) continue;
        float const limit = p->limit > 0.0f ? p->limit : 1.0f;
        if (p->heat < limit) continue;
        float const over = p->heat / limit;
        if (best < 0 || over > best_over) {
            best_over = over;
            best = i;
        }
    }
    return best;
}

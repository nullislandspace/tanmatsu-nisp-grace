// =====================================================================
//  Null Island Space Program  --  the random numbers (see rng.h)
// =====================================================================

#include "common/rng.h"

#define GOLDEN 0x9E3779B97F4A7C15ull

static uint64_t s_state;

void rng_init(uint64_t seed) {
    s_state = seed;
}

uint64_t rng_mix64(uint64_t x) {
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

uint32_t rng_u32(void) {
    s_state += GOLDEN;
    return (uint32_t)(rng_mix64(s_state) >> 32);
}

float rng_unit(void) {
    // 24 bits of the mixed word, over 2^24: exactly representable in a
    // float, and never 1.0.
    return (float)(rng_u32() >> 8) * (1.0f / 16777216.0f);
}

float rng_span(float lo, float hi) {
    return lo + (hi - lo) * rng_unit();
}

int rng_int(int lo, int hi) {
    if (hi <= lo) return lo;
    return lo + (int)(rng_unit() * (float)(hi - lo + 1));
}

uint32_t rng_hash(uint64_t seed, int64_t a, int64_t b) {
    return (uint32_t)(rng_mix64(seed + (uint64_t)a * GOLDEN + (uint64_t)b * 0xC2B2AE3D27D4EB4Full) >> 32);
}

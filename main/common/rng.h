#pragma once
// =====================================================================
//  Null Island Space Program  --  the random numbers
// ---------------------------------------------------------------------
//  splitmix64: six lines, no state beyond one 64-bit word, and good
//  enough statistics for a starfield or a survey grid. Everything the
//  world is made of comes out of a seed, so the same seed rebuilds the
//  same sky exactly -- which is also what makes a screenshot hash a
//  meaningful regression test (testkit/devtest.h).
//
//  rng_hash() is the stateless form: the same (seed, a, b) always gives
//  the same value, so per-cell noise needs no iteration order.
// =====================================================================

#include <stdint.h>

void rng_init(uint64_t seed);

// The stateless mixer: any 64 bits in, well-mixed 64 bits out.
uint64_t rng_mix64(uint64_t x);

uint32_t rng_u32(void);
float rng_unit(void);  // [0, 1)
float rng_span(float lo, float hi);
int rng_int(int lo, int hi);  // inclusive

// A value for one thing in one world: same seed and same coordinates,
// same value, whatever order the caller visits them in.
uint32_t rng_hash(uint64_t seed, int64_t a, int64_t b);

/*
 * ks_stats.h - small, allocation-free statistics used on the node.
 */
#ifndef KS_STATS_H
#define KS_STATS_H

#include "ks_types.h"

/* Welford's online mean/variance (numerically stable in float). */
typedef struct {
    uint32_t n;
    float mean;
    float m2;
} ks_welford_t;

void ks_welford_reset(ks_welford_t *w);
void ks_welford_add(ks_welford_t *w, float x);
float ks_welford_var(const ks_welford_t *w); /* unbiased; 0 if n < 2 */
/* Pool two accumulators (Chan et al. parallel update). */
ks_welford_t ks_welford_merge(const ks_welford_t *a, const ks_welford_t *b);

/* First-order low-pass with time constant tau, irregular sampling. */
typedef struct {
    float y;
    bool init;
} ks_ewma_t;

void ks_ewma_reset(ks_ewma_t *e);
float ks_ewma_update(ks_ewma_t *e, float x, float dt_s, float tau_s);

/* Circular low-pass on the unit circle (degrees in/out, [0,360)). */
typedef struct {
    float c, s;
    bool init;
} ks_circ_ewma_t;

void ks_circ_reset(ks_circ_ewma_t *e);
float ks_circ_update(ks_circ_ewma_t *e, float deg, float dt_s, float tau_s);
float ks_circ_value(const ks_circ_ewma_t *e);
/* Mean resultant length R in [0,1]; 1 means no spread. */
float ks_circ_resultant(const ks_circ_ewma_t *e);

/* Angle helpers (degrees). */
float ks_wrap360(float deg);          /* -> [0, 360)   */
float ks_wrap180(float deg);          /* -> [-180, 180) */
float ks_angdiff(float a, float b);   /* a - b wrapped to [-180,180) */

/* Median of n <= 64 floats (copies, does not reorder input). */
float ks_median(const float *x, int n);

static inline float ks_clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Serial-number arithmetic (RFC 1982) on uint32 milliseconds. */
static inline int32_t ks_ms_diff(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

#endif /* KS_STATS_H */

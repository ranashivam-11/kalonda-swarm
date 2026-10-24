/*
 * ks_stats.c
 */
#include "ks/ks_stats.h"

#include <math.h>

void ks_welford_reset(ks_welford_t *w)
{
    w->n = 0u;
    w->mean = 0.0f;
    w->m2 = 0.0f;
}

void ks_welford_add(ks_welford_t *w, float x)
{
    w->n += 1u;
    const float d = x - w->mean;
    w->mean += d / (float)w->n;
    w->m2 += d * (x - w->mean);
}

float ks_welford_var(const ks_welford_t *w)
{
    return (w->n < 2u) ? 0.0f : w->m2 / (float)(w->n - 1u);
}

ks_welford_t ks_welford_merge(const ks_welford_t *a, const ks_welford_t *b)
{
    ks_welford_t r;
    r.n = a->n + b->n;
    if (r.n == 0u) {
        r.mean = 0.0f;
        r.m2 = 0.0f;
        return r;
    }
    const float d = b->mean - a->mean;
    const float na = (float)a->n, nb = (float)b->n, n = (float)r.n;
    r.mean = a->mean + d * nb / n;
    r.m2 = a->m2 + b->m2 + d * d * na * nb / n;
    return r;
}

void ks_ewma_reset(ks_ewma_t *e)
{
    e->y = 0.0f;
    e->init = false;
}

float ks_ewma_update(ks_ewma_t *e, float x, float dt_s, float tau_s)
{
    if (!e->init || tau_s <= 0.0f) {
        e->y = x;
        e->init = true;
        return e->y;
    }
    const float a = 1.0f - expf(-dt_s / tau_s);
    e->y += a * (x - e->y);
    return e->y;
}

void ks_circ_reset(ks_circ_ewma_t *e)
{
    e->c = 0.0f;
    e->s = 0.0f;
    e->init = false;
}

float ks_circ_update(ks_circ_ewma_t *e, float deg, float dt_s, float tau_s)
{
    const float r = deg * KS_DEG2RAD;
    const float c = cosf(r), s = sinf(r);
    if (!e->init || tau_s <= 0.0f) {
        e->c = c;
        e->s = s;
        e->init = true;
    } else {
        const float a = 1.0f - expf(-dt_s / tau_s);
        e->c += a * (c - e->c);
        e->s += a * (s - e->s);
    }
    return ks_circ_value(e);
}

float ks_circ_value(const ks_circ_ewma_t *e)
{
    return ks_wrap360(atan2f(e->s, e->c) * KS_RAD2DEG);
}

float ks_circ_resultant(const ks_circ_ewma_t *e)
{
    return sqrtf(e->c * e->c + e->s * e->s);
}

float ks_wrap360(float deg)
{
    float r = fmodf(deg, 360.0f);
    if (r < 0.0f) {
        r += 360.0f;
    }
    if (r >= 360.0f) { /* fmodf rounding for tiny negatives */
        r -= 360.0f;
    }
    return r;
}

float ks_wrap180(float deg)
{
    float r = ks_wrap360(deg + 180.0f) - 180.0f;
    return r;
}

float ks_angdiff(float a, float b) { return ks_wrap180(a - b); }

float ks_median(const float *x, int n)
{
    float buf[64];
    if (n <= 0) {
        return 0.0f;
    }
    if (n > 64) {
        n = 64;
    }
    for (int i = 0; i < n; ++i) {
        float v = x[i];
        int j = i - 1;
        while (j >= 0 && buf[j] > v) {
            buf[j + 1] = buf[j];
            --j;
        }
        buf[j + 1] = v;
    }
    return (n & 1) ? buf[n / 2] : 0.5f * (buf[n / 2 - 1] + buf[n / 2]);
}

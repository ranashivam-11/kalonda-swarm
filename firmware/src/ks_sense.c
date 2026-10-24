/*
 * ks_sense.c
 */
#include "ks/ks_sense.h"

#include <math.h>

#include "ks/ks_stats.h"

void ks_rotor_init(ks_rotor_t *r)
{
    for (int i = 0; i < KS_ROTOR_EDGES; ++i) r->t_us[i] = 0u;
    r->head = 0u;
    r->count = 0u;
    r->n_glitch = 0u;
}

static uint32_t last_edge(const ks_rotor_t *r)
{
    return r->t_us[(r->head + KS_ROTOR_EDGES - 1u) % KS_ROTOR_EDGES];
}

void ks_rotor_edge(ks_rotor_t *r, uint32_t t_us)
{
    if (r->count > 0u) {
        /* Reject contact-bounce-like glitches: an edge closer than 1/5 of
         * the previous period to the last one. */
        const uint32_t prev = last_edge(r);
        const uint32_t dt = t_us - prev;
        if (r->count > 1u) {
            const uint32_t prev2 = r->t_us[(r->head + KS_ROTOR_EDGES - 2u) % KS_ROTOR_EDGES];
            const uint32_t per = prev - prev2;
            if (dt < per / 5u) {
                r->n_glitch += 1u;
                return;
            }
        }
        if (dt == 0u) {
            r->n_glitch += 1u;
            return;
        }
    }
    r->t_us[r->head] = t_us;
    r->head = (uint8_t)((r->head + 1u) % KS_ROTOR_EDGES);
    if (r->count < KS_ROTOR_EDGES) r->count += 1u;
}

float ks_rotor_rpm(const ks_rotor_t *r, uint32_t now_us, float edges_per_rev, uint32_t timeout_us)
{
    if (r->count < 2u || edges_per_rev <= 0.0f) return 0.0f;
    const uint32_t since = now_us - last_edge(r);
    if (since > timeout_us) return 0.0f;

    float per[KS_ROTOR_EDGES - 1];
    int n = 0;
    for (uint8_t k = 1; k < r->count; ++k) {
        const uint32_t a = r->t_us[(r->head + KS_ROTOR_EDGES - k) % KS_ROTOR_EDGES];
        const uint32_t b = r->t_us[(r->head + KS_ROTOR_EDGES - k - 1u) % KS_ROTOR_EDGES];
        per[n++] = (float)(uint32_t)(a - b);
    }
    float p = ks_median(per, n);
    if ((float)since > p) p = (float)since; /* decelerating: no edge yet */
    if (p <= 0.0f) return 0.0f;
    return 60.0e6f / (p * edges_per_rev);
}

float ks_anemo_speed(const ks_anemo_cal_t *c, float f_hz)
{
    if (!(f_hz > c->threshold_hz)) return 0.0f;
    const float u = c->slope * f_hz + c->offset;
    return u > 0.0f ? u : 0.0f;
}

bool ks_vane_dir(const ks_vane_cal_t *c, uint16_t code, float *dir_deg)
{
    if (c->code_max <= c->code_min) return false;
    if (code < c->code_min || code > c->code_max) return false;
    const float span = 360.0f - c->deadband_deg;
    const float frac = (float)(code - c->code_min) / (float)(c->code_max - c->code_min);
    *dir_deg = ks_wrap360(frac * span + 0.5f * c->deadband_deg + c->north_offset_deg);
    return true;
}

float ks_ntc_temp_c(const ks_ntc_t *n, float ratio)
{
    if (!(ratio > 0.001f && ratio < 0.999f)) return NAN;
    const float r = n->r_series * ratio / (1.0f - ratio);
    const float inv_t = 1.0f / (n->t0_c + 273.15f) + logf(r / n->r0_ohm) / n->beta_k;
    return 1.0f / inv_t - 273.15f;
}

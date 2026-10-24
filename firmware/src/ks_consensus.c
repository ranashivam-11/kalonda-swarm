/*
 * ks_consensus.c
 */
#include "ks/ks_consensus.h"

#include <math.h>

#define OUTLIER_DEG 45.0f

void ks_cons_init(ks_cons_t *k)
{
    k->init = false;
    k->c = 1.0f;
    k->s = 0.0f;
    ks_circ_reset(&k->own);
    k->spread_deg = 180.0f;
    k->n_used = 0;
    k->n_rejected = 0;
    k->ok = false;
    k->sector_valid = false;
    k->sector = 0u;
}

uint8_t ks_sector_of(float dir_deg)
{
    const float d = ks_wrap360(dir_deg + 0.5f * KS_SECTOR_DEG);
    uint32_t s = (uint32_t)(d / KS_SECTOR_DEG);
    return (uint8_t)(s % KS_SECTORS);
}

float ks_cons_dir(const ks_cons_t *k) { return ks_wrap360(atan2f(k->s, k->c) * KS_RAD2DEG); }

void ks_cons_update(ks_cons_t *k, const ks_params_t *p, float own_dir_deg, bool own_valid,
                    const float *nb_dir_deg, int n_nb, float dt_s)
{
    /* Robust centre of the neighbours' estimates (vector mean). */
    float mc = 0.0f, ms = 0.0f;
    for (int j = 0; j < n_nb; ++j) {
        mc += cosf(nb_dir_deg[j] * KS_DEG2RAD);
        ms += sinf(nb_dir_deg[j] * KS_DEG2RAD);
    }
    const float centre = atan2f(ms, mc) * KS_RAD2DEG;

    float zc = 0.0f, zs = 0.0f;
    if (own_valid) {
        const float z = ks_circ_update(&k->own, own_dir_deg, dt_s, p->cons_tau_s);
        /* A vane that disagrees with the neighbourhood by more than the
         * outlier bound distrusts itself; otherwise its bias would leak into
         * the whole array as bias / N. */
        if (n_nb >= 2 && fabsf(ks_angdiff(z, centre)) > OUTLIER_DEG) {
            own_valid = false;
        } else {
            zc = cosf(z * KS_DEG2RAD);
            zs = sinf(z * KS_DEG2RAD);
        }
    }

    if (!k->init) {
        if (own_valid) {
            k->c = zc;
            k->s = zs;
            k->init = true;
        } else if (n_nb > 0 && (mc * mc + ms * ms) > 1e-6f) {
            const float n = sqrtf(mc * mc + ms * ms);
            k->c = mc / n;
            k->s = ms / n;
            k->init = true;
        } else {
            k->ok = false;
            return;
        }
    }

    const float cur = ks_cons_dir(k);
    float dc = 0.0f, ds = 0.0f;
    int used = 0, rej = 0;
    float spread = 0.0f;
    for (int j = 0; j < n_nb; ++j) {
        if (fabsf(ks_angdiff(nb_dir_deg[j], centre)) > OUTLIER_DEG && n_nb >= 3) {
            ++rej;
            continue;
        }
        const float r = nb_dir_deg[j] * KS_DEG2RAD;
        dc += cosf(r) - k->c;
        ds += sinf(r) - k->s;
        ++used;
        const float d = fabsf(ks_angdiff(nb_dir_deg[j], cur));
        if (d > spread) spread = d;
    }
    if (used > 0) {
        k->c += p->cons_a * dc / (float)used;
        k->s += p->cons_a * ds / (float)used;
    }
    if (own_valid) {
        k->c += p->cons_b * (zc - k->c);
        k->s += p->cons_b * (zs - k->s);
    }
    const float n = sqrtf(k->c * k->c + k->s * k->s);
    if (n > 1e-6f) {
        k->c /= n;
        k->s /= n;
    }

    k->n_used = used;
    k->n_rejected = rej;
    k->spread_deg = (used > 0) ? spread : 0.0f;
    k->ok = (used > 0) ? (spread < p->cons_ok_deg) : own_valid;

    /* Sector with hysteresis around the boundaries. */
    const float d = ks_cons_dir(k);
    const uint8_t s_new = ks_sector_of(d);
    if (!k->sector_valid) {
        k->sector = s_new;
        k->sector_valid = true;
    } else if (s_new != k->sector) {
        const float off = fabsf(ks_angdiff(d, (float)k->sector * KS_SECTOR_DEG));
        if (off > 0.5f * KS_SECTOR_DEG + p->sector_hyst_deg) {
            k->sector = s_new;
        }
    }
}

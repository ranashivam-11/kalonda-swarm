/*
 * ks_params.c - defaults, name-based access and validation of parameters.
 *
 * Defaults describe the reference node: 2.0 m rotor (inferred from the
 * preprint's Table VII power curve, see docs/hardware.md), N42 rotor magnets
 * (Table V: 80 C maximum operating temperature) and an ESP32 module rated to
 * 85 C ambient.
 */
#include "ks/ks_params.h"

#include <string.h>

typedef enum { PT_F32, PT_U8, PT_U16, PT_U32 } ptype_t;

typedef struct {
    const char *name;
    ptype_t type;
    size_t off;
    double lo, hi;
} pdesc_t;

#define F(field, lo, hi) {#field, PT_F32, offsetof(ks_params_t, field), lo, hi}
#define U8(field, lo, hi) {#field, PT_U8, offsetof(ks_params_t, field), lo, hi}
#define U16(field, lo, hi) {#field, PT_U16, offsetof(ks_params_t, field), lo, hi}
#define U32(field, lo, hi) {#field, PT_U32, offsetof(ks_params_t, field), lo, hi}

static const pdesc_t k_desc[] = {
    U16(node_id, 0, 65534),
    F(x_m, -10000, 10000),
    F(y_m, -10000, 10000),
    F(rotor_d_m, 0.1, 20),
    F(hub_h_m, 0.5, 100),
    F(vane_north_deg, -360, 360),
    F(yaw_north_deg, -360, 360),
    U8(utility, 0, 1),
    F(beta, 0, 2),
    F(accept_z, -5, 10),
    F(reject_z, 0, 10),
    U8(pool_max, 1, 32),
    F(kappa_prior, 1, 10),
    U16(kappa_memory, 0, 10000),
    U32(epoch_ms, 1000, 3600000),
    U32(settle_ms, 0, 3600000),
    U32(block_ms, 100, 3600000),
    U8(epochs_per_round, 6, 64),
    F(step_init_deg, 0.1, 30),
    F(step_min_deg, 0.1, 30),
    F(step_max_deg, 0.1, 30),
    F(yaw_offset_max_deg, 0, 45),
    U8(backoff_rounds, 0, 255),
    F(u_min_ms, 0, 50),
    F(u_max_ms, 0, 50),
    F(u_safe_ms, 0, 60),
    F(u_cutout_ms, 0, 80),
    F(rpm_max, 1, 10000),
    F(t_stator_max_c, -40, 250),
    F(t_mcu_max_c, -40, 150),
    F(t_magnet_max_c, -40, 250),
    F(batt_min_v, 0, 100),
    F(derate_offset_deg, 0, 90),
    F(furl_offset_deg, 0, 90),
    F(brake_max_magnet_c, -40, 250),
    U32(isolation_timeout_ms, 1000, 3600000),
    U32(mode_dwell_ms, 0, 3600000),
    F(ke_20, 0, 100),
    F(alpha_br, -0.01, 0),
    F(tau_th_s, 1, 100000),
    F(r_stator_ohm, 0, 100),
    F(v_diode, 0, 5),
    F(pole_pairs, 1, 64),
    F(tsr_opt, 1, 15),
    U8(toggle_mode, 0, 2),
    U32(toggle_period_s, 60, 86400),
    U32(neighbor_timeout_ms, 500, 600000),
    U32(latency_ms, 0, 1000),
    F(cons_a, 0, 1),
    F(cons_b, 0, 1),
    F(cons_tau_s, 0, 3600),
    F(cons_ok_deg, 0.5, 90),
    F(sector_hyst_deg, 0, 5),
    F(ti_split, 0.01, 0.5),
    F(ti_hyst, 0, 0.1),
    F(ti_tau_s, 10, 7200),
    F(wake_k, 0, 0.5),
    F(wake_margin_d, 0, 5),
    F(wake_max_d, 1, 50),
};

#define N_DESC (sizeof(k_desc) / sizeof(k_desc[0]))

void ks_params_default(ks_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->node_id = 0;
    p->rotor_d_m = 2.0f;
    p->hub_h_m = 10.0f;

    p->utility = KS_UTIL_COOPERATIVE;
    p->beta = 1.0f;
    /* Pooled sequential test (ks_negotiate.c: decide). With the per-trial
     * signal-to-noise of a calm night (|delta| ~ 0.1-0.8 SE) a one-shot
     * z > 1 rule lets a node random-walk on a flat objective; requiring a
     * pooled z of 2 before moving and giving up after twelve inconclusive
     * repeats keeps the selfish array near greedy while a cooperative node
     * still climbs (docs/multi_agent.md, section 5). */
    p->accept_z = 2.0f;
    p->reject_z = 1.0f;
    p->pool_max = 12u;
    /* Null calibration (ks_negotiate.c): the SIL measures kappa ~ 1.8-2.0 on
     * calm nights and ~1.7 in convective afternoons; until a node has its
     * own estimate it assumes 2, worth NULLCAL_PRIOR_N idle rounds. */
    p->kappa_prior = 2.0f;
    p->kappa_memory = 64u;

    /* 30-s phases: most of a trial's noise is slow wind wander, which the
     * ABBA contrast rejects as (f T)^4, so short phases beat long ones; 9 s
     * of settling is three time constants of the reference rotor and wake.
     * A field rotor may need more: settle_ms >= 3 x (yaw slew + rotor time
     * constant + wake advection to the nearest neighbour). */
    p->epoch_ms = 30000u;
    p->settle_ms = 9000u;
    p->block_ms = 3000u;
    p->epochs_per_round = 6u;

    /* The cooperative gain of a step grows ~linearly with its size at night
     * while the noise does not: 10-deg first steps learn faster than 6. */
    p->step_init_deg = 10.0f;
    p->step_min_deg = 2.5f;
    p->step_max_deg = 12.5f;
    p->yaw_offset_max_deg = 25.0f;
    p->backoff_rounds = 4u;

    p->u_min_ms = 2.5f;
    p->u_max_ms = 9.0f;
    p->u_safe_ms = 11.0f;
    p->u_cutout_ms = 16.0f;
    p->rpm_max = 450.0f;
    p->t_stator_max_c = 110.0f;
    p->t_mcu_max_c = 80.0f;
    p->t_magnet_max_c = 75.0f;
    p->batt_min_v = 0.0f;
    p->derate_offset_deg = 20.0f;
    p->furl_offset_deg = 70.0f;
    /* N42 (Table V): with a 300 kA/m armature field from a shorted bus the
     * knee is reached near 65 C (python/kalonda/dynamo/magnets.py), so a hot
     * rotor is furled, not braked, unless it is overspeeding. */
    p->brake_max_magnet_c = 60.0f;
    p->isolation_timeout_ms = 30000u;
    p->mode_dwell_ms = 60000u;

    p->ke_20 = 1.30f;
    p->alpha_br = -0.0012f;
    p->tau_th_s = 1200.0f;
    p->r_stator_ohm = 0.8f;
    p->v_diode = 1.4f;
    p->pole_pairs = 8.0f;
    p->tsr_opt = 5.5f;

    p->toggle_mode = KS_TOGGLE_ALWAYS_ON;
    p->toggle_period_s = 3600u;

    p->neighbor_timeout_ms = 5000u;
    p->latency_ms = 3u;

    p->cons_a = 0.5f;
    p->cons_b = 0.05f;
    p->cons_tau_s = 30.0f;
    p->cons_ok_deg = 8.0f;
    p->sector_hyst_deg = 2.0f;
    p->ti_split = 0.10f;
    p->ti_hyst = 0.01f;
    p->ti_tau_s = 600.0f;

    p->wake_k = 0.10f;
    p->wake_margin_d = 0.5f;
    p->wake_max_d = 15.0f;
}

static const pdesc_t *find(const char *name)
{
    for (size_t i = 0; i < N_DESC; ++i) {
        if (strcmp(k_desc[i].name, name) == 0) {
            return &k_desc[i];
        }
    }
    return NULL;
}

int ks_params_set(ks_params_t *p, const char *name, double value)
{
    const pdesc_t *d = find(name);
    if (d == NULL) {
        return -1;
    }
    if (!(value >= d->lo && value <= d->hi)) { /* also rejects NaN */
        return -2;
    }
    uint8_t *base = (uint8_t *)p + d->off;
    switch (d->type) {
    case PT_F32: { float v = (float)value; memcpy(base, &v, sizeof v); break; }
    case PT_U8: { uint8_t v = (uint8_t)value; memcpy(base, &v, sizeof v); break; }
    case PT_U16: { uint16_t v = (uint16_t)value; memcpy(base, &v, sizeof v); break; }
    case PT_U32: { uint32_t v = (uint32_t)value; memcpy(base, &v, sizeof v); break; }
    }
    return 0;
}

int ks_params_get(const ks_params_t *p, const char *name, double *value)
{
    const pdesc_t *d = find(name);
    if (d == NULL) {
        return -1;
    }
    const uint8_t *base = (const uint8_t *)p + d->off;
    switch (d->type) {
    case PT_F32: { float v; memcpy(&v, base, sizeof v); *value = v; break; }
    case PT_U8: { uint8_t v; memcpy(&v, base, sizeof v); *value = v; break; }
    case PT_U16: { uint16_t v; memcpy(&v, base, sizeof v); *value = v; break; }
    case PT_U32: { uint32_t v; memcpy(&v, base, sizeof v); *value = v; break; }
    }
    return 0;
}

size_t ks_params_count(void) { return N_DESC; }

const char *ks_params_name(size_t i) { return i < N_DESC ? k_desc[i].name : NULL; }

int ks_params_validate(const ks_params_t *p)
{
    if (p->settle_ms >= p->epoch_ms) return 1;
    if (p->block_ms > p->epoch_ms - p->settle_ms) return 2;
    if (!(p->step_min_deg <= p->step_init_deg && p->step_init_deg <= p->step_max_deg)) return 3;
    if (p->step_max_deg > p->yaw_offset_max_deg && p->yaw_offset_max_deg > 0.0f) return 4;
    if (!(p->u_min_ms < p->u_max_ms)) return 5;
    if (!(p->u_max_ms <= p->u_safe_ms && p->u_safe_ms <= p->u_cutout_ms)) return 6;
    if (p->epochs_per_round < 2u + KS_PHASES) return 7;
    if (p->neighbor_timeout_ms >= p->isolation_timeout_ms) return 8;
    /* A trial must fit a whole number of blocks per phase, at least three. */
    if ((p->epoch_ms - p->settle_ms) / p->block_ms < 3u) return 9;
    if (p->pool_max < 1u) return 10;
    return 0;
}

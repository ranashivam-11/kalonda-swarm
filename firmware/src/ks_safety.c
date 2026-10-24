/*
 * ks_safety.c
 */
#include "ks/ks_safety.h"

#include <math.h>
#include <string.h>

#define OVERSPEED_HOLD_MS 2000u
#define ANEMO_ZERO_HOLD_MS 60000u
#define VANE_WINDOW_MS 600000u
#define VANE_MIN_STD_DEG 0.5f
#define ROTOR_TURNING_RPM 60.0f
#define ANEMO_DRIFT_RATIO 0.6f
#define DEMAG_RATIO 0.93f

void ks_safety_init(ks_safety_t *s, uint32_t now_ms)
{
    memset(s, 0, sizeof(*s));
    s->mode = KS_MODE_SAFE;  /* start conservative until consensus forms */
    s->mode_since_ms = now_ms;
    s->clean_since_ms = now_ms;
    s->vane_win_start_ms = now_ms;
    s->last_ms = now_ms;
    ks_welford_reset(&s->vane_win);
    s->init = true;
}

float ks_safety_magnet_temp(const ks_safety_t *s) { return s->t_mag.init ? s->t_mag.y : NAN; }

float ks_safety_ke_ratio(const ks_safety_t *s) { return s->ke_ratio.init ? s->ke_ratio.y : NAN; }

bool ks_safety_anemo_usable(const ks_safety_t *s)
{
    return (s->faults & (KS_F_ANEMO_STUCK | KS_F_ANEMO_DRIFT)) == 0u;
}

bool ks_safety_vane_usable(const ks_safety_t *s) { return (s->faults & KS_F_VANE_STUCK) == 0u; }

ks_mode_t ks_safety_update(ks_safety_t *s, const ks_params_t *p, const ks_safety_in_t *in,
                           uint32_t now_ms)
{
    uint32_t dt_ms = now_ms - s->last_ms;
    if (dt_ms == 0u || dt_ms > 10000u) dt_ms = 1000u;
    s->last_ms = now_ms;
    const float dt = (float)dt_ms * 1e-3f;

    /* Latched dust flags survive the recomputation below. */
    uint16_t f = (uint16_t)(in->driver_faults & (KS_F_HALL_TIMEOUT | KS_F_ADC_RANGE | KS_F_YAW_STALL));
    f |= (uint16_t)(s->faults & (KS_F_ANEMO_STUCK | KS_F_ANEMO_DRIFT | KS_F_VANE_STUCK | KS_F_DEMAG_SUSPECT));

    /* --- wind ------------------------------------------------------------ */
    if (in->anemo_valid) {
        ks_ewma_update(&s->u10, in->u_ms, dt, 600.0f);
        ks_ewma_update(&s->gust3, in->u_ms, dt, 3.0f);
    }
    if (s->u10.init && s->u10.y > p->u_safe_ms) f |= KS_F_HIGH_WIND;
    if (s->gust3.init && s->gust3.y > p->u_cutout_ms) f |= KS_F_CUTOUT;

    /* --- overspeed -------------------------------------------------------- */
    if (in->rpm > p->rpm_max) {
        s->overspeed_ms += dt_ms;
    } else {
        s->overspeed_ms = 0u;
    }
    if (s->overspeed_ms >= OVERSPEED_HOLD_MS) f |= KS_F_OVERSPEED;

    /* --- heat -------------------------------------------------------------- */
    if (in->t_stator_c == in->t_stator_c) { /* not NaN */
        ks_ewma_update(&s->t_mag, in->t_stator_c, dt, p->tau_th_s);
        if (in->t_stator_c > p->t_stator_max_c) f |= KS_F_OVERTEMP_STATOR;
        if (s->t_mag.y > p->t_magnet_max_c) f |= KS_F_MAGNET_HOT;
    }
    if (in->t_mcu_c == in->t_mcu_c && in->t_mcu_c > p->t_mcu_max_c) f |= KS_F_OVERTEMP_MCU;

    /* --- Br monitor: measured vs thermally expected EMF constant ---------- */
    const float omega = in->rpm * (2.0f * KS_PI / 60.0f);
    if (omega > 5.0f && p->ke_20 > 0.0f && s->t_mag.init) {
        const float ke_meas = (in->v_bus + in->i_bus * p->r_stator_ohm + p->v_diode) / omega;
        const float ke_exp = p->ke_20 * (1.0f + p->alpha_br * (s->t_mag.y - 20.0f));
        if (ke_exp > 0.0f) {
            if (!s->ke_ratio.init) { /* presume healthy: one transient sample must not latch a fault */
                s->ke_ratio.y = 1.0f;
                s->ke_ratio.init = true;
            }
            ks_ewma_update(&s->ke_ratio, ke_meas / ke_exp, dt, 600.0f);
            if (s->ke_ratio.y < DEMAG_RATIO) f |= KS_F_DEMAG_SUSPECT;
        }
    }

    /* --- dust: anemometer --------------------------------------------------- */
    if (in->anemo_valid && in->rpm > ROTOR_TURNING_RPM) {
        if (in->u_ms < 0.3f) {
            s->anemo_zero_ms += dt_ms;
        } else {
            s->anemo_zero_ms = 0u;
            f &= (uint16_t)~KS_F_ANEMO_STUCK;
        }
        const float u_rot = omega * 0.5f * p->rotor_d_m / p->tsr_opt;
        if (u_rot > 1.0f && in->u_ms > 0.5f) {
            if (!s->anemo_ratio.init) { /* presume healthy, see ke_ratio */
                s->anemo_ratio.y = 1.0f;
                s->anemo_ratio.init = true;
            }
            ks_ewma_update(&s->anemo_ratio, in->u_ms / u_rot, dt, 1800.0f);
            if (s->anemo_ratio.y < ANEMO_DRIFT_RATIO) {
                f |= KS_F_ANEMO_DRIFT;
            } else if (s->anemo_ratio.y > ANEMO_DRIFT_RATIO + 0.1f) {
                f &= (uint16_t)~KS_F_ANEMO_DRIFT;
            }
        }
    }
    if (s->anemo_zero_ms >= ANEMO_ZERO_HOLD_MS) f |= KS_F_ANEMO_STUCK;
    if (!in->anemo_valid) f |= KS_F_ANEMO_STUCK;

    /* --- dust: vane --------------------------------------------------------- */
    if (in->vane_valid) {
        if (s->vane_win.n == 0u) s->vane_ref = in->dir_deg;
        ks_welford_add(&s->vane_win, ks_angdiff(in->dir_deg, s->vane_ref));
    }
    if ((uint32_t)(now_ms - s->vane_win_start_ms) >= VANE_WINDOW_MS) {
        const bool windy = s->u10.init && s->u10.y > 2.0f;
        if (s->vane_win.n > 300u && windy) {
            if (sqrtf(ks_welford_var(&s->vane_win)) < VANE_MIN_STD_DEG) {
                f |= KS_F_VANE_STUCK;
            } else {
                f &= (uint16_t)~KS_F_VANE_STUCK;
            }
        }
        ks_welford_reset(&s->vane_win);
        s->vane_win_start_ms = now_ms;
    }
    if (!in->vane_valid) f |= KS_F_VANE_STUCK;

    /* --- network ------------------------------------------------------------ */
    if (in->n_alive == 0) {
        s->isolated_ms += dt_ms;
    } else {
        s->isolated_ms = 0u;
    }
    if (s->isolated_ms >= p->isolation_timeout_ms) f |= KS_F_RADIO_ISOLATED;

    if (p->batt_min_v > 0.0f && in->batt_v < p->batt_min_v) f |= KS_F_BATTERY_LOW;

    s->faults = f;

    /* --- mode ---------------------------------------------------------------- */
    ks_mode_t target = KS_MODE_NORMAL;
    if (f & (KS_F_OVERSPEED | KS_F_CUTOUT)) {
        target = KS_MODE_SHUTDOWN;
    } else if ((f & (KS_F_HIGH_WIND | KS_F_RADIO_ISOLATED | KS_F_OVERTEMP_MCU | KS_F_BATTERY_LOW)) ||
               !in->consensus_ok) {
        target = KS_MODE_SAFE;
    } else if (f & (KS_F_MAGNET_HOT | KS_F_OVERTEMP_STATOR)) {
        target = KS_MODE_DERATE;
    }

    if (target > s->mode) {
        s->mode = target;
        s->mode_since_ms = now_ms;
        s->relaxing = false;
    } else if (target < s->mode) {
        if (!s->relaxing) {
            s->relaxing = true;
            s->clean_since_ms = now_ms;
        }
        if ((uint32_t)(now_ms - s->clean_since_ms) >= p->mode_dwell_ms) {
            s->mode = target;
            s->mode_since_ms = now_ms;
            s->relaxing = false;
        }
    } else {
        s->relaxing = false;
    }
    return s->mode;
}

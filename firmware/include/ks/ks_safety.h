/*
 * ks_safety.h - supervisor: dust, heat, overspeed, isolation.
 *
 * The supervisor overrides negotiation. Its effect on yaw is one of:
 *   NORMAL   negotiated offset
 *   DERATE   fixed thermal offset (unload stator; protects N42 magnets,
 *            whose maximum operating temperature is 80 C)
 *   SAFE     zero offset, no trials (greedy operation is always safe)
 *   SHUTDOWN brake request + active furl
 * Modes escalate immediately and relax only after mode_dwell_ms of clean
 * conditions.
 */
#ifndef KS_SAFETY_H
#define KS_SAFETY_H

#include "ks_params.h"
#include "ks_stats.h"

typedef struct {
    float rpm;
    float u_ms;            /* own anemometer, 1-s mean        */
    bool anemo_valid;
    float dir_deg;         /* own vane, 1-s mean              */
    bool vane_valid;
    float v_bus, i_bus;    /* rectified bus                    */
    float t_stator_c, t_mcu_c;
    float batt_v;
    int n_alive;           /* neighbours heard recently        */
    bool consensus_ok;
    uint16_t driver_faults;/* raised by drivers (Hall timeout, ADC range, yaw stall) */
} ks_safety_in_t;

typedef struct {
    ks_mode_t mode;
    uint16_t faults;
    uint32_t mode_since_ms;
    uint32_t clean_since_ms;
    /* filters */
    ks_ewma_t u10;          /* 10-min mean wind                  */
    ks_ewma_t gust3;        /* 3-s mean wind                     */
    ks_ewma_t t_mag;        /* magnet temperature estimate       */
    ks_ewma_t anemo_ratio;  /* anemometer / rotor-derived wind    */
    ks_ewma_t ke_ratio;     /* measured / expected EMF constant  */
    ks_welford_t vane_win;  /* vane activity in the current window */
    uint32_t vane_win_start_ms;
    uint32_t overspeed_ms;  /* accumulated time above rpm_max     */
    uint32_t anemo_zero_ms; /* rotor turning, cups not            */
    uint32_t isolated_ms;
    float vane_ref;         /* first vane sample of the window    */
    bool relaxing;          /* target below mode, dwell running   */
    uint32_t last_ms;
    bool init;
} ks_safety_t;

void ks_safety_init(ks_safety_t *s, uint32_t now_ms);

/* Call at 1 Hz. Returns the mode to apply. */
ks_mode_t ks_safety_update(ks_safety_t *s, const ks_params_t *p, const ks_safety_in_t *in,
                           uint32_t now_ms);

/* Estimates exposed for beacons and logs. */
float ks_safety_magnet_temp(const ks_safety_t *s);
float ks_safety_ke_ratio(const ks_safety_t *s);
bool ks_safety_anemo_usable(const ks_safety_t *s);
bool ks_safety_vane_usable(const ks_safety_t *s);

#endif /* KS_SAFETY_H */

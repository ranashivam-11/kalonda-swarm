/*
 * ks_params.h - runtime parameters of one node.
 *
 * Parameters are set by name (ks_params_set) so that the ESP32 port can load
 * them from NVS key/value pairs and the Python software-in-the-loop harness
 * can set them through ctypes without mirroring the struct layout.
 */
#ifndef KS_PARAMS_H
#define KS_PARAMS_H

#include "ks_types.h"

typedef struct {
    /* --- identity and surveyed geometry ------------------------------- */
    uint16_t node_id;
    float x_m, y_m;        /* local east/north from the site datum [m]   */
    float rotor_d_m;       /* rotor diameter [m]                         */
    float hub_h_m;         /* hub height [m]                             */
    float vane_north_deg;  /* installation alignment of the vane (port)  */
    float yaw_north_deg;   /* installation alignment of the yaw encoder  */

    /* --- objective ----------------------------------------------------- */
    uint8_t utility;       /* ks_utility_t                               */
    float beta;            /* weight on downstream credit, 0..1          */
    float accept_z;        /* accept once the pooled z exceeds this      */
    float reject_z;        /* reject once the pooled z falls below -this */
    uint8_t pool_max;      /* trials pooled per decision; then reject    */
    float kappa_prior;     /* SE inflation assumed before calibration    */
    uint16_t kappa_memory; /* idle rounds remembered; 0 freezes kappa    */

    /* --- trial timing --------------------------------------------------- */
    uint32_t epoch_ms;     /* length of one phase                        */
    uint32_t settle_ms;    /* discarded at the start of each phase       */
    uint32_t block_ms;     /* averaging block for the standard error     */
    uint8_t epochs_per_round; /* PROPOSE + 4 phases + REPORT/DECIDE = 6  */

    /* --- yaw search (per wind sector) ---------------------------------- */
    float step_init_deg, step_min_deg, step_max_deg;
    float yaw_offset_max_deg;
    uint8_t backoff_rounds; /* eligible rounds skipped once converged    */

    /* --- operating window and protection ------------------------------- */
    float u_min_ms, u_max_ms;     /* trials only inside this window       */
    float u_safe_ms;              /* 10-min mean above which SAFE         */
    float u_cutout_ms;            /* 3-s gust above which SHUTDOWN        */
    float rpm_max;
    float t_stator_max_c, t_mcu_max_c, t_magnet_max_c;
    float batt_min_v;             /* 0 disables the battery check         */
    float derate_offset_deg;      /* thermal unloading offset             */
    float furl_offset_deg;        /* active furl in SHUTDOWN              */
    float brake_max_magnet_c;     /* no electrical brake above this magnet
                                     temperature unless overspeeding     */
    uint32_t isolation_timeout_ms;/* no neighbours -> SAFE                */
    uint32_t mode_dwell_ms;       /* minimum time before relaxing a mode  */

    /* --- magnet / generator model (Br monitor) ------------------------- */
    float ke_20;           /* EMF constant at 20 C [V s/rad]             */
    float alpha_br;        /* reversible Br coefficient [1/K]            */
    float tau_th_s;        /* stator -> magnet thermal lag [s]           */
    float r_stator_ohm;    /* DC-equivalent winding resistance [ohm]     */
    float v_diode;         /* rectifier drop [V]                         */
    float pole_pairs;      /* Hall edges per mech. revolution = pole_pairs */
    float tsr_opt;         /* tip-speed ratio used for the rotor wind check */

    /* --- validation toggle --------------------------------------------- */
    uint8_t toggle_mode;   /* ks_toggle_t                                */
    uint32_t toggle_period_s;

    /* --- network ------------------------------------------------------- */
    uint32_t neighbor_timeout_ms;
    uint32_t latency_ms;   /* one-way radio latency used by epoch sync   */
    uint8_t key[16];       /* SipHash-2-4 frame authentication key       */

    /* --- consensus ------------------------------------------------------ */
    float cons_a;          /* neighbour (disagreement) gain               */
    float cons_b;          /* innovation gain on own vane                 */
    float cons_tau_s;      /* low-pass of own vane before innovation      */
    float cons_ok_deg;     /* spread below which consensus is declared    */
    float sector_hyst_deg; /* hysteresis on sector changes                */
    float ti_split;        /* TI separating stable / convective classes   */
    float ti_hyst;         /* hysteresis half-width on the TI split       */
    float ti_tau_s;        /* averaging time of the TI estimate           */

    /* --- geometric wake cone used to find downstream participants ------ */
    float wake_k;          /* top-hat expansion rate                      */
    float wake_margin_d;   /* extra lateral margin (steered wakes) [D]    */
    float wake_max_d;      /* ignore pairs farther than this [D]          */
} ks_params_t;

void ks_params_default(ks_params_t *p);

/* 0 on success, -1 unknown name, -2 out of range. */
int ks_params_set(ks_params_t *p, const char *name, double value);
int ks_params_get(const ks_params_t *p, const char *name, double *value);

size_t ks_params_count(void);
const char *ks_params_name(size_t i);

/* Cross-field validation. Returns 0 if consistent, otherwise the 1-based
 * index of the first violated rule (see ks_params.c). */
int ks_params_validate(const ks_params_t *p);

#endif /* KS_PARAMS_H */

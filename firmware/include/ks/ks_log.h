/*
 * ks_log.h - 64-byte binary log records (decoded by kalonda.telemetry.binlog).
 *
 *   off size
 *     0   1  record type (KS_LOG_*)
 *     1   1  record version (state/event 1, trial 2)
 *     2   2  node id
 *     4   4  local time [ms]
 *     8   4  epoch
 *    12  50  payload (type specific, see docs/protocol.md)
 *    62   2  CRC-16/CCITT-FALSE over bytes 0..61
 */
#ifndef KS_LOG_H
#define KS_LOG_H

#include "ks_types.h"

enum { KS_LOG_STATE = 1, KS_LOG_TRIAL = 2, KS_LOG_EVENT = 3 };

enum {
    KS_EV_BOOT = 1,
    KS_EV_MODE = 2,
    KS_EV_EPOCH_ADJUST = 3,
    KS_EV_SECTOR = 4,
    KS_EV_AUTH_FAIL = 5,
    KS_EV_FAULTS = 6,
    KS_EV_TOGGLE = 7,
    KS_EV_CONFLICT = 8,
    KS_EV_LUT_PERSIST = 9,
    KS_EV_NULL_CAL = 10   /* a[0..1] = kappa per class, u[0..1] = idle rounds */
};

enum {
    KS_TEV_PROPOSED = 1,
    KS_TEV_JOINED = 2,
    KS_TEV_REPORT_SENT = 3,
    KS_TEV_REPORT_RCVD = 4,
    KS_TEV_DECIDED = 5,
    KS_TEV_ABORTED = 6
};

typedef struct {
    uint8_t b[KS_LOG_REC_BYTES];
} ks_logrec_t;

typedef struct {
    float u_ms, dir_deg, cons_dir_deg, uref_ms, power_w, rpm, yaw_meas_deg;
    float offset_deg, t_stator_c, t_mag_c, ke_ratio;
    uint16_t epoch_ms, status, faults;
    uint8_t sector, mode, n_alive, flags;
    float energy_wh;
} ks_log_state_t;

/* Trial record, version 2: v1 carried the phase means as floats and had no
 * pool fields; v2 stores the phase means as int16 (0.02 W) to make room. */
typedef struct {
    uint16_t trial_id, initiator;
    uint8_t event, role, sector, outcome;
    float base_deg, trial_deg;
    float delta_w, se_w;         /* own ABBA contrast of this trial       */
    float score_w, score_se_w;   /* this trial's score (own or cooperative) */
    float coop_score_w;          /* own + beta * downstream, in every mode */
    float phase_mean[KS_PHASES];
    float pool_mean_w, pool_z;   /* pooled evidence after this trial      */
    uint8_t pool_n;
    uint8_t n_reports, quality;
} ks_log_trial_t;

typedef struct {
    uint16_t code, arg;
    float a[4];
    uint32_t u[2];
} ks_log_event_t;

void ks_log_state(ks_logrec_t *r, uint16_t node, uint32_t t_ms, uint32_t epoch,
                  const ks_log_state_t *s);
void ks_log_trial(ks_logrec_t *r, uint16_t node, uint32_t t_ms, uint32_t epoch,
                  const ks_log_trial_t *t);
void ks_log_event(ks_logrec_t *r, uint16_t node, uint32_t t_ms, uint32_t epoch,
                  const ks_log_event_t *e);

/* 1 if the CRC matches. */
int ks_log_check(const ks_logrec_t *r);

#endif /* KS_LOG_H */

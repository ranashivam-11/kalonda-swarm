/*
 * ks_node.h - one edge node: the functional core of the firmware.
 *
 * The port (ESP32, or the Python software-in-the-loop harness) owns all I/O:
 *
 *   ks_node_rx()    enqueue an authenticated-or-not frame from the radio
 *   ks_node_step()  every KS_STEP_MS: sensors in -> yaw command, frames, logs out
 *
 * Nothing in here touches hardware, allocates memory or blocks, which is
 * what lets the exact same object code run on the turbine and inside the
 * simulation that produced the figures in the README.
 */
#ifndef KS_NODE_H
#define KS_NODE_H

#include "ks_consensus.h"
#include "ks_epoch.h"
#include "ks_geom.h"
#include "ks_log.h"
#include "ks_nbtab.h"
#include "ks_params.h"
#include "ks_safety.h"
#include "ks_stats.h"
#include "ks_wire.h"

typedef struct {
    uint32_t t_ms;            /* monotonic local time                       */
    float rpm;                /* rotor speed from Hall edge timing          */
    float v_bus, i_bus;       /* rectified bus voltage [V] and current [A]  */
    float u_ms;               /* own anemometer [m/s]                       */
    bool anemo_valid;
    float dir_deg;            /* own vane, wind FROM, degrees from north     */
    bool vane_valid;
    float yaw_deg;            /* measured nacelle heading                    */
    float t_stator_c, t_mcu_c;/* NAN if not fitted                           */
    float batt_v;
    uint16_t driver_faults;   /* KS_F_HALL_TIMEOUT | KS_F_ADC_RANGE | KS_F_YAW_STALL */
} ks_sensors_t;

typedef struct {
    uint8_t len;
    int8_t rssi;
    uint8_t data[KS_FRAME_MAX];
} ks_frame_t;

typedef struct {
    float yaw_cmd_deg;        /* absolute heading command                    */
    float offset_deg;         /* offset relative to the consensus wind       */
    bool yaw_enable;          /* false until a wind direction exists         */
    bool brake;
    ks_mode_t mode;
    uint16_t status, faults;
    bool persist_lut;         /* LUT changed: port should write it to NVS    */
    uint8_t n_tx;
    ks_frame_t tx[KS_TX_MAX];
    uint8_t n_log;
    ks_logrec_t log[KS_LOG_MAX];
    uint16_t n_log_dropped;
} ks_outputs_t;

/* One bin (wind sector x stability class) of the learned table. The node
 * negotiates only the steering *magnitude*; the sign follows geometry: the
 * wake is always deflected away from the side on which the nearest
 * downstream rotor currently sits (ks_node.c: update_side). */
typedef struct {
    float off_deg;            /* steering magnitude learned for this bin, >= 0 */
    float step_deg;           /* current search step                         */
    int8_t dir;               /* next direction to try (+1 / -1)             */
    uint8_t rejects;          /* consecutive rejections at this step         */
    uint8_t backoff;          /* eligible rounds still to skip               */
    uint16_t n_trials, n_accept;
    float led_self_w;         /* measured own change, summed along the path  */
    float led_down_w;         /* measured downstream change, same path       */
    float recv_w;             /* gains this node received from upstream      */
    /* Evidence pooled over repeats of one contrast (pool_base -> pool_trial).
     * One 4-minute ABBA trial rarely resolves a 1 W change on a calm night,
     * so an inconclusive trial is repeated and the repeats are combined
     * (inverse-variance) until the pooled z leaves [-reject_z, accept_z] or
     * pool_max trials are spent; then the status quo stays. Not persisted. */
    float pool_base, pool_trial;
    float pool_sw, pool_w;    /* sum of score/se^2 and of 1/se^2             */
    float pool_self, pool_down; /* summed own and downstream deltas [W]      */
    uint8_t pool_n;
} ks_lut_e_t;

/* ABBA measurement of normalised power over one trial. */
typedef struct {
    bool active;
    uint16_t trial_id, initiator;
    uint8_t sector;
    uint32_t start_epoch;     /* epoch of phase A1                           */
    ks_welford_t ph[KS_PHASES];
    int8_t blk_phase;
    uint32_t blk_idx;
    float blk_p, blk_u3;
    uint16_t blk_n;
    float sum_u3;
    uint32_t n_u3;
    int16_t bin0;             /* this node's own bin when measuring began    */
    bool participant;         /* measuring someone else's trial            */
    bool off0_set;
    float off0;               /* participant: own offset when measuring began */
    uint8_t quality;
} ks_meas_t;

typedef enum {
    KS_TR_IDLE = 0,
    KS_TR_PROPOSED = 1,       /* announced, waiting for A1                   */
    KS_TR_RUNNING = 2,        /* inside A1 B1 B2 A2                          */
    KS_TR_COLLECT = 3         /* waiting for reports                         */
} ks_trstate_t;

typedef struct {
    ks_trstate_t st;
    uint16_t id;
    uint8_t sector;
    float base_deg, trial_deg;
    uint32_t start_epoch;
    uint8_t n_part;
    uint16_t part[KS_MAX_PART];
    bool got[KS_MAX_PART];
    ks_report_t rep[KS_MAX_PART];
    ks_meas_t own;
    uint8_t n_propose_tx;
    uint32_t next_tx_ms;
    uint32_t decide_due_ms;
    bool decided;             /* a DECIDE repeat is still pending            */
    ks_decide_t decision;
    uint8_t n_decide_tx;
    uint32_t dec_next_ms;
} ks_initiator_t;

typedef struct {
    bool used;
    uint16_t initiator;
    ks_propose_t prop;
    ks_meas_t m;
    bool report_ready;
    ks_report_t rep;
    uint8_t n_report_tx;
    uint32_t next_tx_ms;
} ks_observer_t;

/* Null calibration of the trial standard error (ks_negotiate.c). In a round
 * in which a node neither initiates nor participates it measures the same
 * ABBA contrast of its own power anyway. Nothing it controls changed, so the
 * contrast is pure noise and the running mean of its squared z is the
 * factor kappa^2 by which the block-based standard error understates the
 * real phase-to-phase noise (blocks are autocorrelated; the wind wanders on
 * time scales longer than a phase). Every reported and own SE is multiplied
 * by kappa of the trial's stability class before it is used. */
typedef struct {
    ks_meas_t m;
    bool on;
    float off0;               /* applied offset when the window began        */
    float kap2[KS_TI_CLASSES];
    uint32_t n[KS_TI_CLASSES];
} ks_nullcal_t;

typedef struct {
    ks_params_t p;
    uint32_t now_ms, boot_ms;
    uint16_t boot, seq;

    ks_epoch_t ep;
    uint32_t last_epoch;

    ks_nbtab_t nb;
    ks_cons_t cons;
    ks_safety_t sf;
    ks_geom_t geo;
    uint32_t geo_sig;          /* signature of the site set last built      */
    uint32_t geo_built_ms;

    ks_lut_e_t lut[KS_BINS];
    ks_initiator_t tr;
    ks_observer_t obs[KS_MAX_OBSERVE];
    ks_nullcal_t nc;
    uint16_t next_trial_id;
    uint8_t skip_rounds;

    /* receive queue */
    ks_frame_t rxq[KS_RX_QUEUE];
    uint32_t rxq_t[KS_RX_QUEUE];
    uint8_t rxq_n;
    uint32_t rx_dropped;

    /* 1-s aggregation */
    uint32_t sec_start_ms;
    float acc_p, acc_u, acc_rpm, acc_v, acc_i, acc_dc, acc_ds;
    uint16_t acc_n, acc_nu, acc_nd;
    float p_1s, u_1s, rpm_1s, dir_1s, v_1s, i_1s;
    bool u_1s_ok, dir_1s_ok;
    float t_stator, t_mcu, yaw_meas;

    float uref;
    bool uref_ok, uref_fallback;

    /* turbulence intensity of the free-stream reference and the LUT bin */
    ks_ewma_t ti_m, ti_m2;
    uint32_t ti_samples;
    float ti;
    bool ti_valid;
    uint8_t ti_class;
    uint8_t bin;               /* sector + KS_SECTORS * ti_class            */
    bool bin_valid;

    /* steering side: -1 = turn anticlockwise (wake to the right of the wind),
     * +1 = clockwise (wake to the left), chosen to clear the nearest
     * downstream rotor; dy is that rotor's lateral offset (left positive) */
    int8_t side;
    bool side_valid;
    float side_dy;
    uint32_t side_flips;

    ks_mode_t mode;
    uint16_t faults;
    bool toggle_on, toggle_known;
    uint8_t bin_logged;
    float offset_cmd;

    double energy_wh;
    double ledger_sacrifice_wh, ledger_credit_wh, ledger_received_wh;

    uint32_t n_auth_fail, n_conflicts, last_auth_log_ms;
    uint32_t n_trials, n_accept, n_reject, n_abort, n_hold;

    ks_outputs_t *out;         /* valid only inside ks_node_step()          */
} ks_node_t;

void ks_node_init(ks_node_t *n, const ks_params_t *p, uint16_t boot_count, uint32_t now_ms);

/* Enqueue a received frame; decoding happens in the next step. */
int ks_node_rx(ks_node_t *n, const uint8_t *data, size_t len, int8_t rssi, uint32_t t_ms);

void ks_node_step(ks_node_t *n, const ks_sensors_t *s, ks_outputs_t *out);

/* LUT access (bin = sector + KS_SECTORS * stability class) for NVS
 * persistence and for simulation initial conditions; values are steering
 * magnitudes in degrees (clamped to [0, yaw_offset_max_deg]). */
float ks_node_lut_offset(const ks_node_t *n, uint8_t bin);
void ks_node_lut_set(ks_node_t *n, uint8_t bin, float off_deg);

static inline uint8_t ks_bin_sector(uint8_t bin) { return (uint8_t)(bin % KS_SECTORS); }
static inline uint8_t ks_bin_class(uint8_t bin) { return (uint8_t)(bin / KS_SECTORS); }

/* Toggle state for a given epoch (pure function of params and epoch). */
bool ks_toggle_state(const ks_params_t *p, uint32_t epoch);

#endif /* KS_NODE_H */

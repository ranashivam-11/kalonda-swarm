/*
 * test_negotiate.c - end-to-end negotiation on an in-process radio network.
 *
 * Three nodes in a row (5D spacing) under a steady west wind, with a toy
 * plant in which yawing node j by g costs cos^2(g) of its own power and
 * removes a Gaussian share of the deficit it imposes on node j+1:
 *
 *   P_j = P0 * cos^2(g_j) * (1 - 0.4 * exp(-(g_{j-1} / 15 deg)^2))
 *
 * Measurements are noise-free, so the tests check protocol logic exactly:
 * the selfish objective must settle at zero offset (and unravel any
 * cooperative starting point), the sacrifice-aware objective must climb to
 * the argmax of total power found by brute force.
 */
#include <math.h>
#include <string.h>

#include "ks/ks_bytes.h"
#include "ks/ks_log.h"
#include "ks/ks_node.h"
#include "ktest.h"

enum { N = 3 };

static const uint8_t KEY[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

typedef struct {
    ks_node_t node[N];
    ks_outputs_t out[N];
    float offset[N];
    uint32_t t;
    double drop_report; /* probability-like deterministic drop pattern */
    uint32_t n_dropped;
    float u_noise;      /* std of the free-stream fluctuation [m/s]       */
    float u_now;        /* current free-stream speed seen by all anemometers */
    float dir_amp;      /* slow wind-direction swing amplitude [deg]      */
    float dir_period_s;
    uint32_t rng;
    int yaw_blind;      /* plant power independent of yaw: every trial is a null */
    uint32_t z_after_ms; /* collect node 0's decision z-scores after this time   */
    uint32_t n_z;
    float z[512];
} net_t;

/* Shared free-stream fluctuation: every anemometer sees the same gust. */
static float gust(net_t *w)
{
    float acc = 0.0f;
    for (int k = 0; k < 12; ++k) {
        w->rng = w->rng * 1664525u + 1013904223u;
        acc += (float)(w->rng >> 8) / 16777216.0f;
    }
    return (acc - 6.0f) * w->u_noise; /* Irwin-Hall approximation of N(0,1) */
}

static int yaw_blind_plant = 0;

static float plant_power(const float *g, int j)
{
    static const float zero[N] = {0};
    if (yaw_blind_plant) g = zero;
    const float P0 = 50.0f;
    const float c = cosf(g[j] * KS_DEG2RAD);
    float deficit = 0.0f;
    if (j > 0) {
        const float x = g[j - 1] / 15.0f;
        deficit = 0.4f * expf(-x * x);
    }
    return P0 * c * c * (1.0f - deficit);
}

static void net_init(net_t *w, uint8_t utility)
{
    memset(w, 0, sizeof *w);
    for (int i = 0; i < N; ++i) {
        ks_params_t p;
        ks_params_default(&p);
        p.node_id = (uint16_t)(i + 1);
        p.x_m = 10.0f * (float)i;
        p.y_m = 0.0f;
        p.utility = utility;
        p.epoch_ms = 3000u;
        p.settle_ms = 500u;
        p.block_ms = 500u;
        p.mode_dwell_ms = 2000u;
        p.accept_z = 0.0f;
        memcpy(p.key, KEY, 16);
        KT_CHECK(ks_params_validate(&p) == 0);
        ks_node_init(&w->node[i], &p, 1u, (uint32_t)(i * 137u));
    }
    w->u_now = 6.0f;
}

/* Deterministic pseudo-random drop decision for REPORT frames. */
static int drop(net_t *w, const ks_frame_t *f)
{
    if (w->drop_report <= 0.0) return 0;
    if ((f->data[0] & 0x0Fu) != KS_MSG_REPORT) return 0;
    static uint32_t lcg = 12345u;
    lcg = lcg * 1103515245u + 12345u;
    if ((double)((lcg >> 8) & 0xFFFFu) / 65536.0 < w->drop_report) {
        w->n_dropped++;
        return 1;
    }
    return 0;
}

static void net_run(net_t *w, uint32_t seconds)
{
    const uint32_t steps = seconds * 1000u / KS_STEP_MS;
    for (uint32_t k = 0; k < steps; ++k) {
        w->t += KS_STEP_MS;
        if ((w->t % 1000u) == 0u) w->u_now = 6.0f + gust(w); /* one gust value per second */
        for (int i = 0; i < N; ++i) {
            ks_sensors_t s;
            memset(&s, 0, sizeof s);
            s.t_ms = w->t + (uint32_t)(i * 137u);
            const float p = plant_power(w->offset, i);
            s.rpm = 200.0f;
            s.v_bus = 48.0f;
            s.i_bus = p / 48.0f;
            s.u_ms = w->u_noise > 0.0f ? w->u_now : 6.0f;
            s.anemo_valid = true;
            s.dir_deg = 270.0f + 2.0f * sinf((float)w->t * 1e-3f / 7.0f);
            if (w->dir_amp > 0.0f) {
                s.dir_deg += w->dir_amp * sinf(2.0f * KS_PI * (float)w->t * 1e-3f / w->dir_period_s);
            }
            s.vane_valid = true;
            s.yaw_deg = 270.0f + w->offset[i];
            s.t_stator_c = 40.0f;
            s.t_mcu_c = 45.0f;
            yaw_blind_plant = w->yaw_blind;
            ks_node_step(&w->node[i], &s, &w->out[i]);
            w->offset[i] = w->out[i].offset_deg;
            if (i == 0 && w->t >= w->z_after_ms) {
                for (int r = 0; r < w->out[0].n_log; ++r) {
                    const uint8_t *b = w->out[0].log[r].b;
                    if (b[0] != KS_LOG_TRIAL || b[16] != KS_TEV_DECIDED || b[17] != 0u) continue;
                    if (b[19] == KS_OUT_ABORT || w->n_z >= 512u) continue;
                    const float sc = ks_get_f32(b + 32), se = ks_get_f32(b + 36);
                    if (se > 0.0f) w->z[w->n_z++] = sc / se;
                }
            }
        }
        /* broadcast medium: everything sent this step arrives next step */
        for (int i = 0; i < N; ++i) {
            for (int f = 0; f < w->out[i].n_tx; ++f) {
                const ks_frame_t *fr = &w->out[i].tx[f];
                for (int j = 0; j < N; ++j) {
                    if (j == i || drop(w, fr)) continue;
                    ks_node_rx(&w->node[j], fr->data, fr->len, -60, w->t + (uint32_t)(j * 137u));
                }
            }
        }
    }
}

static float brute_force_optimum(float *g_best)
{
    float best = -1.0f;
    float g[N] = {0, 0, 0};
    for (float a = 0.0f; a <= 25.0f; a += 0.25f) {
        for (float b = 0.0f; b <= 25.0f; b += 0.25f) {
            g[0] = a;
            g[1] = b;
            g[2] = 0.0f;
            const float tot = plant_power(g, 0) + plant_power(g, 1) + plant_power(g, 2);
            if (tot > best) {
                best = tot;
                g_best[0] = a;
                g_best[1] = b;
                g_best[2] = 0.0f;
            }
        }
    }
    return best;
}

static float total(const float *g) { return plant_power(g, 0) + plant_power(g, 1) + plant_power(g, 2); }

static void test_geometry_and_sync_form(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_COOPERATIVE);
    net_run(&w, 60u);
    for (int i = 0; i < N; ++i) {
        const ks_node_t *n = &w.node[i];
        KT_CHECK(n->ep.synced);
        KT_CHECK(n->cons.ok && n->cons.sector == 27);
        KT_CHECK(n->geo.valid && n->geo.n_colors == 3u);
        KT_CHECK(n->mode == KS_MODE_NORMAL);
        KT_CHECK(n->ep.epoch == w.node[0].ep.epoch);
    }
    KT_CHECK(w.node[0].geo.n_up == 0 && w.node[0].geo.n_down == 2);
    KT_CHECK(w.node[2].geo.n_down == 0);
    KT_CHECK(w.node[0].uref_ok && fabsf(w.node[2].uref - 6.0f) < 0.01f);
}

static void test_selfish_objective_stays_greedy(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_SELFISH);
    net_run(&w, 3600u);
    for (int i = 0; i < N; ++i) {
        KT_NEAR(ks_node_lut_offset(&w.node[i], 27), 0.0, 1e-6);
    }
    KT_CHECK(w.node[0].n_trials > 10u);
    KT_CHECK(w.node[0].n_accept == 0u);
    /* the forgone cooperative gain is visible in the logs: some rejected
     * trials had a positive cooperative score */
    KT_CHECK(w.node[0].lut[27].n_trials == w.node[0].n_trials);
}

static void test_selfish_objective_unravels_cooperation(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_SELFISH);
    ks_node_lut_set(&w.node[0], 27, 20.0f);
    ks_node_lut_set(&w.node[1], 27, 20.0f);
    net_run(&w, 4u * 3600u);
    KT_CHECK(fabsf(ks_node_lut_offset(&w.node[0], 27)) <= 3.0f);
    KT_CHECK(fabsf(ks_node_lut_offset(&w.node[1], 27)) <= 3.0f);
}

static void test_cooperative_objective_finds_optimum(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_COOPERATIVE);
    net_run(&w, 4u * 3600u);
    float g_opt[N];
    const float p_opt = brute_force_optimum(g_opt);
    float g[N];
    for (int i = 0; i < N; ++i) g[i] = ks_node_lut_offset(&w.node[i], 27);
    const float p0[N] = {0, 0, 0};
    const float gain_opt = p_opt / total(p0) - 1.0f;
    const float gain = total(g) / total(p0) - 1.0f;
    printf("    optimum %.1f/%.1f deg (+%.2f %%), negotiated %.1f/%.1f deg (+%.2f %%)\n", (double)g_opt[0],
           (double)g_opt[1], 100.0 * (double)gain_opt, (double)g[0], (double)g[1], 100.0 * (double)gain);
    KT_NEAR(g[0], g_opt[0], 3.0);
    KT_NEAR(g[1], g_opt[1], 3.0);
    KT_NEAR(g[2], 0.0, 1e-6);           /* last row never sacrifices */
    KT_CHECK(gain > 0.95f * gain_opt);  /* within 5 % of the attainable gain */
    /* ledger: the front unit pays, the back units are credited */
    KT_CHECK(w.node[0].lut[27].led_self_w < 0.0f);
    KT_CHECK(w.node[0].lut[27].led_down_w > 0.0f);
    KT_CHECK(w.node[1].lut[27].recv_w > 0.0f);
    KT_CHECK(w.node[0].ledger_sacrifice_wh > 0.0);
}

static void test_report_loss_aborts_but_still_converges(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_COOPERATIVE);
    w.drop_report = 0.3;
    net_run(&w, 6u * 3600u);
    KT_CHECK(w.n_dropped > 0u);
    KT_CHECK(w.node[0].n_abort > 0u);
    float g_opt[N];
    brute_force_optimum(g_opt);
    KT_NEAR(ks_node_lut_offset(&w.node[0], 27), g_opt[0], 4.0);
}

/* Gusts make single trials inconclusive. With the default pooled test
 * (accept at z > 2, reject at z < -1 or after 6 repeats) a selfish node
 * must not random-walk away from greedy, and a cooperative one must still
 * climb: HOLD outcomes appear and the pool, not one lucky trial, decides. */
static void noisy_defaults(net_t *w, float u_noise)
{
    ks_params_t d;
    ks_params_default(&d);
    for (int i = 0; i < N; ++i) {
        w->node[i].p.accept_z = d.accept_z;
        w->node[i].p.reject_z = d.reject_z;
        w->node[i].p.pool_max = d.pool_max;
    }
    w->u_noise = u_noise;
    w->rng = 2024u;
}

/* Null calibration end to end: on a plant whose power ignores yaw every
 * trial is a null, so once the idle rounds have calibrated kappa the
 * decision z-scores must have unit spread. The block-based SE alone is too
 * small (1-s gusts are shared by consecutive 0.5-s blocks): kappa > 1.3. */
static void test_null_calibration_gives_unit_z(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_SELFISH);
    noisy_defaults(&w, 0.25f);
    w.yaw_blind = 1;
    w.z_after_ms = 3u * 3600u * 1000u;
    KT_NEAR(sqrt((double)w.node[0].nc.kap2[0]), (double)w.node[0].p.kappa_prior, 1e-6);
    net_run(&w, 12u * 3600u);
    double m = 0.0, v = 0.0;
    for (uint32_t k = 0; k < w.n_z; ++k) m += w.z[k];
    m /= (double)w.n_z;
    for (uint32_t k = 0; k < w.n_z; ++k) v += (w.z[k] - m) * (w.z[k] - m);
    const double sd = sqrt(v / (double)(w.n_z - 1u));
    const double kap = sqrt((double)w.node[0].nc.kap2[0]);
    printf("    kappa %.2f after %u idle rounds; %u null decisions, z mean %+.2f sd %.2f\n", kap,
           w.node[0].nc.n[0], w.n_z, m, sd);
    KT_CHECK(w.node[0].nc.n[0] > 100u && w.n_z > 40u);
    KT_CHECK(kap > 1.3);
    KT_CHECK(sd > 0.75 && sd < 1.35);
    KT_CHECK(fabs(m) < 0.4);
}

/* Mean final offsets of nodes 0 and 1 over three gust seeds, 6 h each. */
static void noisy_runs(uint8_t utility, float az, float rz, uint8_t pool_max, double *mean, double *max,
                       uint32_t *n_hold)
{
    static net_t w;
    *mean = 0.0;
    *max = 0.0;
    *n_hold = 0u;
    for (uint32_t seed = 1u; seed <= 3u; ++seed) {
        net_init(&w, utility);
        noisy_defaults(&w, 0.25f);
        w.rng = seed * 7919u;
        for (int i = 0; i < N; ++i) {
            w.node[i].p.accept_z = az;
            w.node[i].p.reject_z = rz;
            w.node[i].p.pool_max = pool_max;
        }
        net_run(&w, 6u * 3600u);
        for (int i = 0; i < 2; ++i) {
            const double g = ks_node_lut_offset(&w.node[i], 27);
            *mean += g / 6.0;
            if (g > *max) *max = g;
        }
        *n_hold += w.node[0].n_hold;
    }
}

static void test_noisy_trials_are_pooled_before_deciding(void)
{
    ks_params_t d;
    ks_params_default(&d);
    double s_new, s_new_max, c_new, c_new_max;
    uint32_t h_new, h_coop;
    noisy_runs(KS_UTIL_SELFISH, d.accept_z, d.reject_z, d.pool_max, &s_new, &s_new_max, &h_new);
    noisy_runs(KS_UTIL_COOPERATIVE, d.accept_z, d.reject_z, d.pool_max, &c_new, &c_new_max, &h_coop);
    float g_opt[N];
    brute_force_optimum(g_opt);
    printf("    selfish %.1f deg (max %.1f), cooperative %.1f deg (optimum %.1f/%.1f)\n", s_new, s_new_max, c_new,
           (double)g_opt[0], (double)g_opt[1]);
    KT_CHECK(h_new > 0u && h_coop > 0u);    /* the pool, not one trial, decides */
    /* a selfish node may wander only where its own loss is below the noise
     * of a full pool (cos^2 loss < 3 %, |g| < 10 deg): its indifference zone */
    KT_CHECK(s_new_max <= 10.0);
    KT_CHECK(c_new >= 0.8 * 0.5 * (double)(g_opt[0] + g_opt[1]));
}

static void test_conflicting_proposal_lower_id_wins(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_COOPERATIVE);
    /* run until node 2 (id 2) is inside a trial */
    int guard = 0;
    while (w.node[1].tr.st != KS_TR_PROPOSED && guard++ < 200000) net_run(&w, 1u);
    KT_CHECK(w.node[1].tr.st == KS_TR_PROPOSED);
    /* forge an overlapping proposal from node id 1 involving node 3 */
    ks_node_t *n = &w.node[1];
    ks_hdr_t h;
    memset(&h, 0, sizeof h);
    h.version = KS_PROTO_VERSION;
    h.type = KS_MSG_PROPOSE;
    h.src = 1;
    h.dst = KS_NODE_BROADCAST;
    h.boot = 2;
    h.seq = 1;
    h.epoch = n->ep.epoch;
    ks_payload_t pl;
    memset(&pl, 0, sizeof pl);
    pl.propose.trial_id = 4242;
    pl.propose.sector = 27;
    pl.propose.n_part = 2;
    pl.propose.part[0] = 2;
    pl.propose.part[1] = 3;
    pl.propose.base_deg = 0.0f;
    pl.propose.trial_deg = 6.0f;
    pl.propose.start_epoch = n->tr.start_epoch;
    uint8_t buf[KS_FRAME_MAX];
    const size_t len = ks_wire_encode(buf, sizeof buf, &h, &pl, KEY);
    const uint32_t aborts = n->n_abort;
    ks_node_rx(n, buf, len, -50, w.t);
    net_run(&w, 1u);
    KT_CHECK(n->n_conflicts == 1u);
    KT_CHECK(n->n_abort == aborts + 1u);
    KT_CHECK(n->tr.st == KS_TR_IDLE);
}

static void test_stability_class_from_reference_turbulence(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_COOPERATIVE);
    w.rng = 7u;
    w.u_noise = 0.9f; /* TI = 0.15: convective */
    net_run(&w, 900u);
    for (int i = 0; i < N; ++i) {
        KT_CHECK(w.node[i].ti_valid);
        KT_NEAR(w.node[i].ti, 0.15, 0.03);
        KT_CHECK(w.node[i].ti_class == 1u && w.node[i].bin == 27u + KS_SECTORS);
    }
    w.u_noise = 0.3f; /* TI = 0.05: stable */
    net_run(&w, 3000u);
    for (int i = 0; i < N; ++i) {
        KT_CHECK(w.node[i].ti_class == 0u && w.node[i].bin == 27u);
    }
}

static void test_steering_side_follows_geometry(void)
{
    /* The wind swings +-8 deg about the row: the downstream neighbour moves
     * +-1.4 m across the centre line, so the steering side must flip, always
     * pointing the wake away from it, and never chatter. */
    static net_t w;
    net_init(&w, KS_UTIL_COOPERATIVE);
    w.dir_amp = 8.0f;
    w.dir_period_s = 1200.0f;
    ks_node_lut_set(&w.node[0], 27, 15.0f);
    net_run(&w, 2400u);
    const ks_node_t *n = &w.node[0];
    KT_CHECK(n->side_valid);
    KT_CHECK(n->side_flips >= 3u && n->side_flips <= 8u); /* ~2 per swing period */
    /* the sign of the applied offset is opposite to the neighbour's side */
    KT_CHECK((n->side_dy > 0.2f && n->side == -1) || (n->side_dy < -0.2f && n->side == 1) ||
             fabsf(n->side_dy) <= 0.2f);
    KT_CHECK(fabsf(w.offset[0]) > 1.0f || n->tr.st != KS_TR_IDLE);
    KT_CHECK(w.node[2].side_valid == false); /* last row never steers */
    KT_NEAR(w.offset[2], 0.0, 1e-6);
}

static void test_wrong_key_frames_are_ignored(void)
{
    static net_t w;
    net_init(&w, KS_UTIL_COOPERATIVE);
    net_run(&w, 30u);
    uint8_t bad[16];
    memcpy(bad, KEY, 16);
    bad[3] ^= 0xFFu;
    ks_hdr_t h;
    memset(&h, 0, sizeof h);
    h.type = KS_MSG_DECIDE;
    h.src = 1;
    h.dst = KS_NODE_BROADCAST;
    h.boot = 9;
    h.seq = 9;
    ks_payload_t pl;
    memset(&pl, 0, sizeof pl);
    pl.decide.outcome = KS_OUT_ACCEPT;
    uint8_t buf[KS_FRAME_MAX];
    const size_t len = ks_wire_encode(buf, sizeof buf, &h, &pl, bad);
    ks_node_rx(&w.node[2], buf, len, -50, w.t);
    net_run(&w, 1u);
    KT_CHECK(w.node[2].n_auth_fail == 1u);
}

int main(void)
{
    printf("test_negotiate\n");
    KT_RUN(test_geometry_and_sync_form);
    KT_RUN(test_selfish_objective_stays_greedy);
    KT_RUN(test_selfish_objective_unravels_cooperation);
    KT_RUN(test_cooperative_objective_finds_optimum);
    KT_RUN(test_report_loss_aborts_but_still_converges);
    KT_RUN(test_null_calibration_gives_unit_z);
    KT_RUN(test_noisy_trials_are_pooled_before_deciding);
    KT_RUN(test_conflicting_proposal_lower_id_wins);
    KT_RUN(test_stability_class_from_reference_turbulence);
    KT_RUN(test_steering_side_follows_geometry);
    KT_RUN(test_wrong_key_frames_are_ignored);
    KT_DONE();
}

/*
 * ks_negotiate.c - peer-to-peer yaw negotiation.
 *
 * A round is `epochs_per_round` synchronised epochs:
 *
 *   k = 0      PROPOSE   initiator announces base and trial offsets and the
 *                        downstream participants whose power will count
 *   k = 1..4   A1 B1 B2 A2   initiator applies base, trial, trial, base;
 *                        everyone involved measures normalised power
 *                        P / u_ref^3 in blocks (ABBA cancels linear drift)
 *   k = 5      REPORT    participants send delta = mean(B) - mean(A) and
 *                        its standard error; initiator scores and DECIDEs
 *
 * Only one colour class of the conflict graph initiates per round, so no two
 * trials ever share a participant. The score is where the objective lives:
 *
 *   selfish      score = dP_self
 *   cooperative  score = dP_self + beta * sum_j dP_j     (j downstream)
 *
 * dP_self is the unit's measured sacrifice (negative when it yaws away from
 * the wind); the sum is the measured gain it buys for its neighbours.
 * With beta = 1 this is the marginal-contribution ("wonderful life") utility,
 * which turns the array into a potential game whose potential is the total
 * measured power (docs/multi_agent.md).
 */
#include "ks_node_internal.h"

#include <math.h>
#include <string.h>

/* A1 vs A2 disagreement is flagged (KS_Q_DRIFT) but not fatal: a waked
 * rotor's power swings by half as its neighbour's wake meanders, which is
 * exactly where steering matters. The A phases are merged before the
 * variance is taken, so a drifting trial carries a large SE and little
 * weight in the pool; the null calibration prices the rest. Aborting on
 * drift discarded 18 % of downstream reports in the SIL. */
#define DRIFT_Z 6.0f
#define DRIFT_REL 0.5f
#define Q_FATAL (KS_Q_FEW_BLOCKS | KS_Q_SECTOR | KS_Q_MODE | KS_Q_MISSED | KS_Q_SELF)

#define NULLCAL_PRIOR_N 10u  /* weight of kappa_prior, in idle rounds        */
#define NULLCAL_Z2_CAP 36.0f /* one gust front must not swamp the estimate  */
#define NULLCAL_LOG_EVERY 16u

void ks__neg_init(ks_node_t *n)
{
    memset(&n->tr, 0, sizeof n->tr);
    memset(n->obs, 0, sizeof n->obs);
    memset(&n->nc, 0, sizeof n->nc);
    for (int c = 0; c < KS_TI_CLASSES; ++c) n->nc.kap2[c] = n->p.kappa_prior * n->p.kappa_prior;
}

/* SE inflation for a measurement that began in `bin0` (-1: unknown). */
static float kappa(const ks_node_t *n, int16_t bin0)
{
    const int c = (bin0 >= 0) ? (int)bin0 / KS_SECTORS : (int)n->ti_class;
    const float k2 = n->nc.kap2[c < KS_TI_CLASSES ? c : 0];
    return k2 > 1.0f ? sqrtf(k2) : 1.0f;
}

/* ------------------------------------------------------------ measurement */

static void meas_start(ks_meas_t *m, uint16_t trial_id, uint16_t initiator, uint8_t sector,
                       uint32_t start_epoch)
{
    memset(m, 0, sizeof(*m));
    m->active = true;
    m->trial_id = trial_id;
    m->initiator = initiator;
    m->sector = sector;
    m->start_epoch = start_epoch;
    m->blk_phase = -1;
    m->bin0 = -1;
    for (int i = 0; i < KS_PHASES; ++i) ks_welford_reset(&m->ph[i]);
}

static void meas_close_block(const ks_node_t *n, ks_meas_t *m)
{
    if (m->blk_phase >= 0 && m->blk_phase < KS_PHASES) {
        uint16_t need = (uint16_t)((n->p.block_ms / KS_STEP_MS) * 4u / 5u);
        if (need == 0u) need = 1u;
        if (m->blk_n >= need && m->blk_u3 > 0.0f) {
            ks_welford_add(&m->ph[m->blk_phase], m->blk_p / m->blk_u3);
        }
    }
    m->blk_phase = -1;
    m->blk_n = 0u;
    m->blk_p = 0.0f;
    m->blk_u3 = 0.0f;
}

static void meas_sample(ks_node_t *n, ks_meas_t *m, float p_w)
{
    if (!m->active) return;
    const int32_t ph = (int32_t)(n->ep.epoch - m->start_epoch);
    if (ph < 0 || ph >= KS_PHASES) {
        meas_close_block(n, m);
        return;
    }
    const uint32_t into = ks_epoch_ms_into(&n->ep, n->now_ms);
    if (into < n->p.settle_ms) {
        meas_close_block(n, m);
        return;
    }
    const uint32_t blk = (into - n->p.settle_ms) / n->p.block_ms;
    if (m->blk_phase != ph || m->blk_idx != blk) {
        meas_close_block(n, m);
        m->blk_phase = (int8_t)ph;
        m->blk_idx = blk;
    }
    if (n->mode != KS_MODE_NORMAL) m->quality |= KS_Q_MODE;
    /* A regime change *during* the trial invalidates the comparison; a
     * neighbour whose consensus sits one bin over at a sector boundary does
     * not, so each node checks only against its own bin at the start. */
    if (m->bin0 < 0 && n->bin_valid) m->bin0 = (int16_t)n->bin;
    if (m->participant) {
        /* a participant's bin may differ from the initiator's at a sector
         * edge; what would spoil its contrast is its own yaw moving */
        const float off = ks__neg_offset(n);
        if (!m->off0_set) {
            m->off0 = off;
            m->off0_set = true;
        } else if (fabsf(off - m->off0) > 0.5f) {
            m->quality |= KS_Q_SELF;
        }
    } else if (!n->bin_valid || (m->bin0 >= 0 && n->bin != (uint8_t)m->bin0)) {
        m->quality |= KS_Q_SECTOR;
    }
    if (!n->uref_ok || n->uref <= 0.1f) {
        m->quality |= KS_Q_NOREF;
        return;
    }
    const float u3 = n->uref * n->uref * n->uref;
    m->blk_p += p_w;
    m->blk_u3 += u3;
    m->blk_n += 1u;
    m->sum_u3 += u3;
    m->n_u3 += 1u;
}

static void meas_result(const ks_meas_t *m, ks_report_t *r)
{
    const ks_welford_t A = ks_welford_merge(&m->ph[0], &m->ph[3]);
    const ks_welford_t B = ks_welford_merge(&m->ph[1], &m->ph[2]);
    uint8_t q = m->quality;
    if (A.n < 3u || B.n < 3u || m->ph[0].n == 0u || m->ph[1].n == 0u || m->ph[2].n == 0u ||
        m->ph[3].n == 0u) {
        q |= KS_Q_FEW_BLOCKS;
    }
    const float u3 = m->n_u3 ? m->sum_u3 / (float)m->n_u3 : 0.0f;
    float var = 0.0f;
    if (A.n > 1u) var += ks_welford_var(&A) / (float)A.n;
    if (B.n > 1u) var += ks_welford_var(&B) / (float)B.n;
    if (m->ph[0].n >= 2u && m->ph[3].n >= 2u) {
        const float d = fabsf(m->ph[0].mean - m->ph[3].mean);
        const float s = sqrtf(ks_welford_var(&m->ph[0]) / (float)m->ph[0].n +
                              ks_welford_var(&m->ph[3]) / (float)m->ph[3].n) + 1e-12f;
        const float ref = fabsf(A.mean) + 1e-12f;
        if (d / s > DRIFT_Z && d / ref > DRIFT_REL) q |= KS_Q_DRIFT;
    }
    r->trial_id = m->trial_id;
    r->initiator = m->initiator;
    r->delta_w = (B.mean - A.mean) * u3;
    r->se_w = sqrtf(var) * u3;
    r->base_w = A.mean * u3;
    r->n_blocks = (uint16_t)(A.n + B.n);
    r->quality = q;
}

/* --------------------------------------------------------------- initiator */

static void send_propose(ks_node_t *n)
{
    ks_payload_t pl;
    memset(&pl, 0, sizeof pl);
    ks_propose_t *m = &pl.propose;
    m->trial_id = n->tr.id;
    m->sector = n->tr.sector;
    m->n_part = n->tr.n_part;
    m->base_deg = n->tr.base_deg;
    m->trial_deg = n->tr.trial_deg;
    m->start_epoch = n->tr.start_epoch;
    memcpy(m->part, n->tr.part, sizeof m->part);
    ks__send(n, KS_MSG_PROPOSE, KS_NODE_BROADCAST, &pl);
}

static void send_decide(ks_node_t *n)
{
    ks_payload_t pl;
    memset(&pl, 0, sizeof pl);
    pl.decide = n->tr.decision;
    ks__send(n, KS_MSG_DECIDE, KS_NODE_BROADCAST, &pl);
}

/* What the initiator's DECIDED record adds to the DECIDE frame: the frame
 * carries the pooled score the decision used, the log also keeps this
 * trial's own score. */
typedef struct {
    float score_w, score_se_w;
    float pool_mean_w, pool_z;
    uint8_t pool_n;
} ks_pool_note_t;

static void log_tr(ks_node_t *n, uint8_t ev, uint8_t role, const ks_initiator_t *t,
                   const ks_report_t *own, const ks_decide_t *d, const ks_pool_note_t *note)
{
    ks_log_trial_t L;
    memset(&L, 0, sizeof L);
    L.trial_id = t->id;
    L.initiator = n->p.node_id;
    L.event = ev;
    L.role = role;
    L.sector = t->sector;
    L.base_deg = t->base_deg;
    L.trial_deg = t->trial_deg;
    if (own) {
        L.delta_w = own->delta_w;
        L.se_w = own->se_w;
        L.quality = own->quality;
    }
    if (d) {
        L.outcome = d->outcome;
        L.score_w = d->score_w;
        L.score_se_w = d->score_se_w;
        L.coop_score_w = d->coop_score_w;
        L.n_reports = d->n_reports;
    }
    if (note) {
        L.score_w = note->score_w;
        L.score_se_w = note->score_se_w;
        L.pool_mean_w = note->pool_mean_w;
        L.pool_z = note->pool_z;
        L.pool_n = note->pool_n;
    }
    for (int i = 0; i < KS_PHASES; ++i) {
        L.phase_mean[i] = t->own.ph[i].n ? t->own.ph[i].mean : NAN;
    }
    ks__log_trial(n, &L);
}

static bool eligible(ks_node_t *n, uint32_t round)
{
    if (n->mode != KS_MODE_NORMAL || !n->toggle_on) return false;
    if (!n->cons.ok || !n->cons.sector_valid || !n->bin_valid) return false;
    if (!n->geo.valid || n->geo.sector != n->cons.sector || n->geo.n_colors == 0u) return false;
    if (!n->uref_ok || n->uref < n->p.u_min_ms || n->uref > n->p.u_max_ms) return false;
    if ((round % n->geo.n_colors) != n->geo.color[n->geo.self]) return false;
    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        if (n->obs[i].used) return false;
    }
    ks_lut_e_t *e = &n->lut[n->bin];
    if (e->backoff > 0u) {
        e->backoff -= 1u;
        return false;
    }
    if (n->geo.n_down == 0u || !n->side_valid) {
        return false; /* last row: there is nobody to steer for */
    }
    return true;
}

static void propose(ks_node_t *n, uint32_t epoch)
{
    const uint8_t s = n->bin;
    ks_lut_e_t *e = &n->lut[s];
    const float mx = n->p.yaw_offset_max_deg;
    const float base = e->off_deg;
    float trial;
    if (e->pool_n > 0u && fabsf(e->pool_base - base) < 0.01f && e->pool_trial <= mx) {
        trial = e->pool_trial; /* the contrast still being pooled */
    } else {
        e->pool_n = 0u;
        trial = ks_clampf(base + (float)e->dir * e->step_deg, 0.0f, mx);
        if (fabsf(trial - base) < 0.01f) {
            e->dir = (int8_t)-e->dir;
            trial = ks_clampf(base + (float)e->dir * e->step_deg, 0.0f, mx);
        }
    }
    if (fabsf(trial - base) < 0.01f) return;

    /* keep a pending DECIDE repeat alive across the reset */
    const bool dec_pending = n->tr.decided;
    const ks_decide_t dec = n->tr.decision;
    const uint8_t dec_tx = n->tr.n_decide_tx;
    const uint32_t dec_next = n->tr.dec_next_ms;
    memset(&n->tr, 0, sizeof n->tr);
    n->tr.decided = dec_pending;
    n->tr.decision = dec;
    n->tr.n_decide_tx = dec_tx;
    n->tr.dec_next_ms = dec_next;

    n->tr.st = KS_TR_PROPOSED;
    n->tr.id = n->next_trial_id++;
    n->tr.sector = s;
    n->tr.base_deg = base;
    n->tr.trial_deg = trial;
    n->tr.start_epoch = epoch + 1u;
    for (uint8_t k = 0; k < n->geo.n_down && n->tr.n_part < KS_MAX_PART; ++k) {
        if (ks__alive(n, n->geo.down[k])) n->tr.part[n->tr.n_part++] = n->geo.down[k];
    }
    meas_start(&n->tr.own, n->tr.id, n->p.node_id, s, n->tr.start_epoch);
    send_propose(n);
    n->tr.n_propose_tx = 1u;
    n->tr.next_tx_ms = n->now_ms + 300u + (uint32_t)((n->p.node_id * 37u) % 400u);
    log_tr(n, KS_TEV_PROPOSED, 0u, &n->tr, NULL, NULL, NULL);
}

/* Fold one trial into the bin's evidence pool and return the pooled z.
 * Trials of a different contrast (or after the base moved) start a new
 * pool. The SE floor keeps a noiseless bench rig finite. */
#define SE_FLOOR_W 1e-3f
static float pool_add(ks_lut_e_t *e, const ks_initiator_t *t, float score, float se, float d_self,
                      float d_down)
{
    if (e->pool_n == 0u || fabsf(e->pool_base - t->base_deg) > 0.01f ||
        fabsf(e->pool_trial - t->trial_deg) > 0.01f) {
        e->pool_base = t->base_deg;
        e->pool_trial = t->trial_deg;
        e->pool_sw = e->pool_w = e->pool_self = e->pool_down = 0.0f;
        e->pool_n = 0u;
    }
    const float s = fmaxf(se, SE_FLOOR_W);
    const float w = 1.0f / (s * s);
    e->pool_sw += score * w;
    e->pool_w += w;
    e->pool_self += d_self;
    e->pool_down += d_down;
    if (e->pool_n < 255u) e->pool_n += 1u;
    return e->pool_sw / sqrtf(e->pool_w);
}

static void lut_update(ks_node_t *n, ks_lut_e_t *e, uint8_t outcome, float trial)
{
    e->n_trials += 1u;
    if (outcome == KS_OUT_ACCEPT) {
        /* the ledger books the pooled mean of the contrast, not the last trial */
        const float k = e->pool_n ? 1.0f / (float)e->pool_n : 0.0f;
        e->off_deg = trial;
        e->led_self_w += e->pool_self * k;
        e->led_down_w += e->pool_down * k;
        e->pool_n = 0u;
        e->step_deg = ks_clampf(e->step_deg * 1.5f, n->p.step_min_deg, n->p.step_max_deg);
        e->rejects = 0u;
        e->n_accept += 1u;
    } else if (outcome == KS_OUT_REJECT) {
        e->pool_n = 0u;
        e->dir = (int8_t)-e->dir;
        e->rejects += 1u;
        if (e->rejects >= 2u) {
            e->rejects = 0u;
            if (e->step_deg <= n->p.step_min_deg + 1e-3f) {
                e->backoff = n->p.backoff_rounds; /* converged: test rarely */
            }
            e->step_deg = ks_clampf(e->step_deg * 0.5f, n->p.step_min_deg, n->p.step_max_deg);
        }
    }
}

static void decide(ks_node_t *n)
{
    ks_initiator_t *t = &n->tr;
    meas_close_block(n, &t->own);
    ks_report_t own;
    meas_result(&t->own, &own);
    own.se_w *= kappa(n, t->own.bin0);

    float down = 0.0f, var_down = 0.0f;
    uint8_t n_rep = 0u, q_down = 0u;
    for (uint8_t i = 0; i < t->n_part; ++i) {
        if (!t->got[i]) continue;
        down += t->rep[i].delta_w;
        var_down += t->rep[i].se_w * t->rep[i].se_w;
        q_down |= t->rep[i].quality;
        ++n_rep;
    }
    const float beta = n->p.beta;
    const float coop = own.delta_w + beta * down;
    const float coop_se = sqrtf(own.se_w * own.se_w + beta * beta * var_down);
    const bool selfish = (n->p.utility == KS_UTIL_SELFISH);
    const float score = selfish ? own.delta_w : coop;
    const float se = selfish ? own.se_w : coop_se;
    const uint8_t q = (uint8_t)(own.quality | (selfish ? 0u : q_down));

    /* Sequential decision on the pooled evidence for this contrast. An
     * aborted trial adds nothing to the pool; a HOLD repeats the contrast
     * at the node's next turn in this bin. */
    ks_lut_e_t *e = &n->lut[t->sector];
    uint8_t outcome;
    float pz = 0.0f;
    if (q & Q_FATAL) {
        outcome = KS_OUT_ABORT;
    } else if (!selfish && n_rep < t->n_part) {
        outcome = KS_OUT_ABORT;
    } else {
        pz = pool_add(e, t, score, se, own.delta_w, down);
        if (pz > n->p.accept_z) {
            outcome = KS_OUT_ACCEPT;
        } else if (pz < -n->p.reject_z || e->pool_n >= n->p.pool_max) {
            outcome = KS_OUT_REJECT;
        } else {
            outcome = KS_OUT_HOLD;
        }
    }
    const uint8_t pool_n = (outcome == KS_OUT_ABORT) ? 0u : e->pool_n;
    const float pool_mean = pool_n ? e->pool_sw / e->pool_w : 0.0f;
    const float pool_se = pool_n ? 1.0f / sqrtf(e->pool_w) : 0.0f;

    lut_update(n, e, outcome, t->trial_deg);
    if (outcome == KS_OUT_ACCEPT && n->out) n->out->persist_lut = true;
    n->n_trials += 1u;
    if (outcome == KS_OUT_ACCEPT) n->n_accept += 1u;
    else if (outcome == KS_OUT_REJECT) n->n_reject += 1u;
    else if (outcome == KS_OUT_HOLD) n->n_hold += 1u;
    else n->n_abort += 1u;

    ks_decide_t *d = &t->decision;
    memset(d, 0, sizeof *d);
    d->trial_id = t->id;
    d->sector = t->sector;
    d->outcome = outcome;
    d->new_deg = (outcome == KS_OUT_ACCEPT) ? t->trial_deg : t->base_deg;
    d->score_w = pool_n ? pool_mean : score;
    d->score_se_w = pool_n ? pool_se : se;
    d->coop_score_w = coop;
    d->n_reports = n_rep;
    d->n_part = t->n_part;
    send_decide(n);
    t->decided = true;
    t->n_decide_tx = 1u;
    t->dec_next_ms = n->now_ms + 400u;
    own.quality = q; /* log the quality the decision actually used */
    const ks_pool_note_t note = {score, se, pool_mean, pool_n ? pz : 0.0f, pool_n};
    log_tr(n, KS_TEV_DECIDED, 0u, t, &own, d, &note);
    t->st = KS_TR_IDLE;
    t->own.active = false;
}

static void abort_own(ks_node_t *n, uint8_t q)
{
    ks_initiator_t *t = &n->tr;
    if (t->st == KS_TR_IDLE) return;
    ks_decide_t *d = &t->decision;
    memset(d, 0, sizeof *d);
    d->trial_id = t->id;
    d->sector = t->sector;
    d->outcome = KS_OUT_ABORT;
    d->new_deg = t->base_deg;
    d->n_part = t->n_part;
    send_decide(n);
    t->decided = true;
    t->n_decide_tx = 1u;
    t->dec_next_ms = n->now_ms + 400u;
    ks_report_t own;
    memset(&own, 0, sizeof own);
    own.quality = q;
    log_tr(n, KS_TEV_ABORTED, 0u, t, &own, d, NULL);
    n->n_trials += 1u;
    n->n_abort += 1u;
    n->lut[t->sector].n_trials += 1u;
    t->st = KS_TR_IDLE;
    t->own.active = false;
}

void ks__neg_abort_all(ks_node_t *n, uint8_t reason_q)
{
    abort_own(n, reason_q);
    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        if (n->obs[i].used) n->obs[i].m.quality |= reason_q;
    }
}

/* ------------------------------------------------------- null calibration */

static void nullcal_start(ks_node_t *n, uint32_t epoch)
{
    ks_nullcal_t *c = &n->nc;
    c->on = false;
    if (n->p.kappa_memory == 0u) return; /* calibration frozen at kappa_prior */
    if (n->tr.st != KS_TR_IDLE || n->mode != KS_MODE_NORMAL || !n->bin_valid || !n->uref_ok) return;
    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        if (n->obs[i].used && !n->obs[i].report_ready) return;
    }
    meas_start(&c->m, 0u, n->p.node_id, n->bin, epoch + 1u);
    c->off0 = ks__neg_offset(n);
    c->on = true;
}

static void nullcal_finish(ks_node_t *n)
{
    ks_nullcal_t *c = &n->nc;
    c->on = false;
    meas_close_block(n, &c->m);
    ks_report_t r;
    meas_result(&c->m, &r);
    if ((r.quality & Q_FATAL) || !(r.se_w > 0.0f) || c->m.bin0 < 0) return;
    const int k = c->m.bin0 / KS_SECTORS;
    if (k >= KS_TI_CLASSES) return;
    const float z = r.delta_w / r.se_w;
    const float z2 = fminf(z * z, NULLCAL_Z2_CAP);
    c->n[k] += 1u;
    uint32_t m = c->n[k] + NULLCAL_PRIOR_N;
    if (m > n->p.kappa_memory) m = n->p.kappa_memory;
    c->kap2[k] += (z2 - c->kap2[k]) / (float)m;
    if ((c->n[k] % NULLCAL_LOG_EVERY) == 0u) {
        ks__log_event(n, KS_EV_NULL_CAL, (uint16_t)k, sqrtf(c->kap2[0]), sqrtf(c->kap2[1]), c->n[0], c->n[1]);
    }
}

/* --------------------------------------------------------- epoch / ticks */

void ks__neg_epoch(ks_node_t *n, uint32_t epoch)
{
    const uint32_t R = n->p.epochs_per_round;
    ks_initiator_t *t = &n->tr;

    if (t->st == KS_TR_PROPOSED && epoch >= t->start_epoch) t->st = KS_TR_RUNNING;
    if (t->st == KS_TR_RUNNING && epoch >= t->start_epoch + KS_PHASES) {
        meas_close_block(n, &t->own);
        t->st = KS_TR_COLLECT;
        t->decide_due_ms = n->now_ms + n->p.epoch_ms * 6u / 10u;
    }
    if (t->st == KS_TR_COLLECT && epoch >= t->start_epoch + KS_PHASES + 1u) decide(n);

    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        ks_observer_t *o = &n->obs[i];
        if (!o->used) continue;
        if (!o->report_ready && epoch >= o->m.start_epoch + KS_PHASES) {
            meas_close_block(n, &o->m);
            meas_result(&o->m, &o->rep);
            o->rep.se_w *= kappa(n, o->m.bin0);
            o->report_ready = true;
            const uint32_t span = n->p.epoch_ms * 3u / 10u;
            o->next_tx_ms = n->now_ms + 100u +
                            (uint32_t)((n->p.node_id * 131u + o->prop.trial_id * 17u) % (span ? span : 1u));
        }
        if (epoch >= o->m.start_epoch + KS_PHASES + 2u) o->used = false; /* expired */
    }

    if (n->nc.on && epoch >= n->nc.m.start_epoch + KS_PHASES) nullcal_finish(n);
    if ((epoch % R) == 0u && t->st == KS_TR_IDLE && eligible(n, epoch / R)) propose(n, epoch);
    if ((epoch % R) == 0u) nullcal_start(n, epoch);
}

void ks__neg_tick(ks_node_t *n)
{
    ks_initiator_t *t = &n->tr;
    if (t->st == KS_TR_PROPOSED && t->n_propose_tx < 2u &&
        (int32_t)(n->now_ms - t->next_tx_ms) >= 0) {
        send_propose(n);
        t->n_propose_tx += 1u;
    }
    if (t->st == KS_TR_COLLECT) {
        uint8_t got = 0u;
        for (uint8_t i = 0; i < t->n_part; ++i) got += t->got[i] ? 1u : 0u;
        if (got == t->n_part || (int32_t)(n->now_ms - t->decide_due_ms) >= 0) decide(n);
    }
    if (t->decided && t->n_decide_tx < 2u && (int32_t)(n->now_ms - t->dec_next_ms) >= 0) {
        send_decide(n);
        t->n_decide_tx += 1u;
    }
    if (t->decided && t->n_decide_tx >= 2u) t->decided = false;

    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        ks_observer_t *o = &n->obs[i];
        if (!o->used || !o->report_ready || o->n_report_tx >= 2u) continue;
        if ((int32_t)(n->now_ms - o->next_tx_ms) < 0) continue;
        ks_payload_t pl;
        memset(&pl, 0, sizeof pl);
        pl.report = o->rep;
        if (ks__send(n, KS_MSG_REPORT, o->initiator, &pl)) {
            if (o->n_report_tx == 0u) {
                ks_log_trial_t L;
                memset(&L, 0, sizeof L);
                L.trial_id = o->prop.trial_id;
                L.initiator = o->initiator;
                L.event = KS_TEV_REPORT_SENT;
                L.role = 1u;
                L.sector = o->prop.sector;
                L.base_deg = o->prop.base_deg;
                L.trial_deg = o->prop.trial_deg;
                L.delta_w = o->rep.delta_w;
                L.se_w = o->rep.se_w;
                L.quality = o->rep.quality;
                for (int k = 0; k < KS_PHASES; ++k) L.phase_mean[k] = o->m.ph[k].n ? o->m.ph[k].mean : NAN;
                ks__log_trial(n, &L);
            }
            o->n_report_tx += 1u;
            o->next_tx_ms = n->now_ms + 700u;
        }
    }
}

void ks__neg_sample(ks_node_t *n, float p_w)
{
    if (n->nc.on) {
        /* a side flip or a learned step would be a real change, not noise */
        if (fabsf(ks__neg_offset(n) - n->nc.off0) > 0.5f || n->tr.st != KS_TR_IDLE) {
            n->nc.on = false;
        } else {
            meas_sample(n, &n->nc.m, p_w);
        }
    }
    if (n->tr.st != KS_TR_IDLE) meas_sample(n, &n->tr.own, p_w);
    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        if (n->obs[i].used && !n->obs[i].report_ready) meas_sample(n, &n->obs[i].m, p_w);
    }
}

float ks__neg_offset(const ks_node_t *n)
{
    if (!n->side_valid) return 0.0f; /* no downstream neighbour: never sacrifice */
    const ks_initiator_t *t = &n->tr;
    float mag;
    if (t->st == KS_TR_RUNNING) {
        const uint32_t ph = n->ep.epoch - t->start_epoch;
        mag = (ph == 1u || ph == 2u) ? t->trial_deg : t->base_deg;
    } else if (t->st == KS_TR_PROPOSED || t->st == KS_TR_COLLECT) {
        mag = t->base_deg;
    } else if (n->bin_valid) {
        mag = n->lut[n->bin].off_deg;
    } else {
        mag = 0.0f;
    }
    return (float)n->side * mag;
}

/* --------------------------------------------------------------- messages */

static bool contains(const uint16_t *v, uint8_t n, uint16_t id)
{
    for (uint8_t i = 0; i < n; ++i) {
        if (v[i] == id) return true;
    }
    return false;
}

void ks__on_propose(ks_node_t *n, const ks_hdr_t *h, const ks_propose_t *m)
{
    const uint16_t me = n->p.node_id;
    ks_initiator_t *t = &n->tr;

    /* Two initiators whose trials overlap in time and share a node: the
     * lower id keeps its trial (both sides apply the same rule). */
    if (t->st == KS_TR_PROPOSED || t->st == KS_TR_RUNNING) {
        const int32_t de = (int32_t)(m->start_epoch - t->start_epoch);
        if (de > -(int32_t)(KS_PHASES + 1) && de < (int32_t)(KS_PHASES + 1)) {
            bool related = contains(t->part, t->n_part, h->src) || contains(m->part, m->n_part, me);
            for (uint8_t i = 0; i < m->n_part && !related; ++i) {
                related = contains(t->part, t->n_part, m->part[i]);
            }
            if (related) {
                n->n_conflicts += 1u;
                ks__log_event(n, KS_EV_CONFLICT, h->src, (float)t->id, (float)m->trial_id, 0u, 0u);
                if (h->src < me) {
                    abort_own(n, KS_Q_MISSED);
                } else {
                    return;
                }
            }
        }
    }

    if (!contains(m->part, m->n_part, me)) return;
    if (m->start_epoch <= n->ep.epoch) return; /* too late to see A1 whole */
    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        const ks_observer_t *o = &n->obs[i];
        if (o->used && o->initiator == h->src && o->prop.trial_id == m->trial_id) return;
    }
    ks_observer_t *slot = NULL;
    for (int i = 0; i < KS_MAX_OBSERVE && !slot; ++i) {
        if (!n->obs[i].used) slot = &n->obs[i];
    }
    if (!slot) {
        for (int i = 0; i < KS_MAX_OBSERVE && !slot; ++i) {
            if (n->obs[i].report_ready && n->obs[i].n_report_tx >= 2u) slot = &n->obs[i];
        }
    }
    if (!slot) return;
    n->nc.on = false; /* this round is not idle after all */
    memset(slot, 0, sizeof *slot);
    slot->used = true;
    slot->initiator = h->src;
    slot->prop = *m;
    meas_start(&slot->m, m->trial_id, h->src, m->sector, m->start_epoch);
    slot->m.participant = true;

    ks_log_trial_t L;
    memset(&L, 0, sizeof L);
    L.trial_id = m->trial_id;
    L.initiator = h->src;
    L.event = KS_TEV_JOINED;
    L.role = 1u;
    L.sector = m->sector;
    L.base_deg = m->base_deg;
    L.trial_deg = m->trial_deg;
    for (int k = 0; k < KS_PHASES; ++k) L.phase_mean[k] = NAN;
    ks__log_trial(n, &L);
}

void ks__on_report(ks_node_t *n, const ks_hdr_t *h, const ks_report_t *m)
{
    ks_initiator_t *t = &n->tr;
    if (m->initiator != n->p.node_id || m->trial_id != t->id) return;
    if (t->st != KS_TR_RUNNING && t->st != KS_TR_COLLECT) return;
    for (uint8_t i = 0; i < t->n_part; ++i) {
        if (t->part[i] != h->src || t->got[i]) continue;
        t->got[i] = true;
        t->rep[i] = *m;
        ks_log_trial_t L;
        memset(&L, 0, sizeof L);
        L.trial_id = t->id;
        L.initiator = h->src; /* reporter */
        L.event = KS_TEV_REPORT_RCVD;
        L.role = 0u;
        L.sector = t->sector;
        L.base_deg = t->base_deg;
        L.trial_deg = t->trial_deg;
        L.delta_w = m->delta_w;
        L.se_w = m->se_w;
        L.quality = m->quality;
        for (int k = 0; k < KS_PHASES; ++k) L.phase_mean[k] = NAN;
        ks__log_trial(n, &L);
        return;
    }
}

void ks__on_decide(ks_node_t *n, const ks_hdr_t *h, const ks_decide_t *m)
{
    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        ks_observer_t *o = &n->obs[i];
        if (!o->used || o->initiator != h->src || o->prop.trial_id != m->trial_id) continue;
        if (m->outcome == KS_OUT_ACCEPT && o->report_ready && m->sector < KS_BINS) {
            n->lut[m->sector].recv_w += o->rep.delta_w;
        }
        ks_log_trial_t L;
        memset(&L, 0, sizeof L);
        L.trial_id = m->trial_id;
        L.initiator = h->src;
        L.event = KS_TEV_DECIDED;
        L.role = 1u;
        L.sector = m->sector;
        L.outcome = m->outcome;
        L.base_deg = o->prop.base_deg;
        L.trial_deg = o->prop.trial_deg;
        L.delta_w = o->rep.delta_w;
        L.se_w = o->rep.se_w;
        L.score_w = m->score_w;
        L.score_se_w = m->score_se_w;
        L.coop_score_w = m->coop_score_w;
        L.n_reports = m->n_reports;
        L.quality = o->rep.quality;
        for (int k = 0; k < KS_PHASES; ++k) L.phase_mean[k] = o->m.ph[k].n ? o->m.ph[k].mean : NAN;
        ks__log_trial(n, &L);
        o->used = false;
        return;
    }
}

/*
 * ks_node.c - step loop, radio intake, 1 Hz housekeeping, yaw command.
 */
#include "ks_node_internal.h"

#include <math.h>
#include <string.h>

#include "ks/ks_crypto.h"

#define BEACON_FRESH_MS 2500u
#define GEO_REFRESH_MS 30000u
#define AUTH_LOG_INTERVAL_MS 10000u

/* ------------------------------------------------------------------ utils */

bool ks__alive(const ks_node_t *n, uint16_t id)
{
    const ks_nb_t *e = ks_nbtab_cfind(&n->nb, id);
    return e != NULL && ks_nb_alive(e, n->now_ms, n->p.neighbor_timeout_ms);
}

bool ks__send(ks_node_t *n, uint8_t type, uint16_t dst, const ks_payload_t *pl)
{
    ks_outputs_t *o = n->out;
    if (o == NULL || o->n_tx >= KS_TX_MAX) {
        return false;
    }
    ks_hdr_t h;
    memset(&h, 0, sizeof h);
    h.version = KS_PROTO_VERSION;
    h.type = type;
    h.src = n->p.node_id;
    h.dst = dst;
    h.boot = n->boot;
    h.seq = ++n->seq;
    h.epoch = n->ep.epoch;
    const uint32_t into = ks_epoch_ms_into(&n->ep, n->now_ms);
    h.epoch_ms = (uint16_t)(into > 65535u ? 65535u : into);
    const size_t len = ks_wire_encode(o->tx[o->n_tx].data, KS_FRAME_MAX, &h, pl, n->p.key);
    if (len == 0u) {
        return false;
    }
    o->tx[o->n_tx].len = (uint8_t)len;
    o->tx[o->n_tx].rssi = 0;
    o->n_tx += 1u;
    return true;
}

static ks_logrec_t *log_slot(ks_node_t *n)
{
    ks_outputs_t *o = n->out;
    if (o == NULL) return NULL;
    if (o->n_log >= KS_LOG_MAX) {
        o->n_log_dropped += 1u;
        return NULL;
    }
    return &o->log[o->n_log++];
}

void ks__log_trial(ks_node_t *n, const ks_log_trial_t *t)
{
    ks_logrec_t *r = log_slot(n);
    if (r) ks_log_trial(r, n->p.node_id, n->now_ms, n->ep.epoch, t);
}

void ks__log_event(ks_node_t *n, uint16_t code, uint16_t arg, float a0, float a1, uint32_t u0,
                   uint32_t u1)
{
    ks_logrec_t *r = log_slot(n);
    if (!r) return;
    ks_log_event_t e;
    memset(&e, 0, sizeof e);
    e.code = code;
    e.arg = arg;
    e.a[0] = a0;
    e.a[1] = a1;
    e.u[0] = u0;
    e.u[1] = u1;
    ks_log_event(r, n->p.node_id, n->now_ms, n->ep.epoch, &e);
}

bool ks_toggle_state(const ks_params_t *p, uint32_t epoch)
{
    if (p->toggle_mode == KS_TOGGLE_ALWAYS_ON) return true;
    const uint64_t t_s = (uint64_t)epoch * p->epoch_ms / 1000u;
    const uint32_t window = (uint32_t)(t_s / (p->toggle_period_s ? p->toggle_period_s : 1u));
    if (p->toggle_mode == KS_TOGGLE_ALTERNATE) return (window & 1u) == 0u;
    /* Keyed pseudo-random schedule: every node derives the same bit from the
     * shared key, so the array toggles together with no coordinator, and the
     * on/off sequence cannot alias with the diurnal cycle. */
    uint8_t msg[8] = {'T', 'O', 'G', 'G', 0, 0, 0, 0};
    msg[4] = (uint8_t)window;
    msg[5] = (uint8_t)(window >> 8);
    msg[6] = (uint8_t)(window >> 16);
    msg[7] = (uint8_t)(window >> 24);
    return (ks_siphash24(p->key, msg, sizeof msg) & 1u) != 0u;
}

/* ------------------------------------------------------------------- init */

void ks_node_init(ks_node_t *n, const ks_params_t *p, uint16_t boot_count, uint32_t now_ms)
{
    memset(n, 0, sizeof(*n));
    n->p = *p;
    n->now_ms = now_ms;
    n->boot_ms = now_ms;
    n->boot = boot_count;
    ks_epoch_init(&n->ep, p->epoch_ms, now_ms);
    n->last_epoch = 0u;
    ks_nbtab_init(&n->nb);
    ks_cons_init(&n->cons);
    ks_safety_init(&n->sf, now_ms);
    for (int s = 0; s < KS_BINS; ++s) {
        n->lut[s].step_deg = p->step_init_deg;
        n->lut[s].dir = 1;
    }
    n->sec_start_ms = now_ms;
    n->mode = KS_MODE_SAFE;
    n->t_stator = NAN;
    n->t_mcu = NAN;
    n->bin_logged = 0xFFu;
    n->next_trial_id = (uint16_t)(boot_count << 8);
    ks__neg_init(n);
}

int ks_node_rx(ks_node_t *n, const uint8_t *data, size_t len, int8_t rssi, uint32_t t_ms)
{
    if (len > KS_FRAME_MAX) return -2;
    if (n->rxq_n >= KS_RX_QUEUE) {
        n->rx_dropped += 1u;
        return -1;
    }
    ks_frame_t *f = &n->rxq[n->rxq_n];
    memcpy(f->data, data, len);
    f->len = (uint8_t)len;
    f->rssi = rssi;
    n->rxq_t[n->rxq_n] = t_ms;
    n->rxq_n += 1u;
    return 0;
}

float ks_node_lut_offset(const ks_node_t *n, uint8_t bin)
{
    return bin < KS_BINS ? n->lut[bin].off_deg : 0.0f;
}

void ks_node_lut_set(ks_node_t *n, uint8_t bin, float off_deg)
{
    if (bin >= KS_BINS) return;
    n->lut[bin].off_deg = ks_clampf(off_deg, 0.0f, n->p.yaw_offset_max_deg);
}

/* ------------------------------------------------------------------ radio */

static void process_rx(ks_node_t *n)
{
    for (uint8_t q = 0; q < n->rxq_n; ++q) {
        const ks_frame_t *f = &n->rxq[q];
        const uint32_t t_rx = n->rxq_t[q];
        ks_hdr_t h;
        ks_payload_t pl;
        const int rc = ks_wire_decode(f->data, f->len, n->p.key, &h, &pl);
        if (rc != 0) {
            if (rc == -3) {
                n->n_auth_fail += 1u;
                if ((uint32_t)(n->now_ms - n->last_auth_log_ms) >= AUTH_LOG_INTERVAL_MS) {
                    n->last_auth_log_ms = n->now_ms;
                    ks__log_event(n, KS_EV_AUTH_FAIL, 0, 0.0f, 0.0f, n->n_auth_fail, 0u);
                }
            }
            continue;
        }
        if (h.src == n->p.node_id) continue;
        ks_nb_t *e = ks_nbtab_accept(&n->nb, &h, t_rx, f->rssi);
        if (e == NULL) continue;

        switch (h.type) {
        case KS_MSG_BEACON: {
            e->b = pl.beacon;
            e->has_beacon = true;
            e->last_beacon_ms = t_rx;
            const uint32_t before = n->ep.epoch;
            if (ks_epoch_observe(&n->ep, h.src, h.epoch, h.epoch_ms, t_rx, n->p.latency_ms)) {
                if (n->ep.epoch != before) {
                    ks__log_event(n, KS_EV_EPOCH_ADJUST, h.src, 0.0f, 0.0f, before, n->ep.epoch);
                }
            }
            break;
        }
        case KS_MSG_PROPOSE: ks__on_propose(n, &h, &pl.propose); break;
        case KS_MSG_REPORT:
            if (h.dst == n->p.node_id || h.dst == KS_NODE_BROADCAST) ks__on_report(n, &h, &pl.report);
            break;
        case KS_MSG_DECIDE: ks__on_decide(n, &h, &pl.decide); break;
        default: break;
        }
    }
    n->rxq_n = 0u;
}

/* ------------------------------------------------------------- 1 Hz tasks */

static uint32_t site_signature(const ks_site_t *s, int n, uint8_t sector)
{
    uint32_t h = 2166136261u ^ sector; /* FNV-1a over ids (order-independent sum) */
    uint32_t sum = 0u;
    for (int i = 0; i < n; ++i) {
        uint32_t x = s[i].id;
        x ^= (uint32_t)lroundf(s[i].x * 10.0f) * 2654435761u;
        x ^= (uint32_t)lroundf(s[i].y * 10.0f) * 40503u;
        sum += x * 16777619u;
    }
    return h ^ sum ^ (uint32_t)n;
}

static void update_geometry(ks_node_t *n)
{
    if (!n->cons.sector_valid) return;
    static ks_site_t sites[KS_MAX_NODES]; /* scratch, not reentrant */
    int m = 0;
    sites[m].id = n->p.node_id;
    sites[m].x = n->p.x_m;
    sites[m].y = n->p.y_m;
    ++m;
    for (int i = 0; i < KS_MAX_NODES && m < KS_MAX_NODES; ++i) {
        const ks_nb_t *e = &n->nb.e[i];
        if (!ks_nb_alive(e, n->now_ms, n->p.neighbor_timeout_ms)) continue;
        sites[m].id = e->id;
        sites[m].x = e->b.x_m;
        sites[m].y = e->b.y_m;
        ++m;
    }
    const uint32_t sig = site_signature(sites, m, n->cons.sector);
    if (n->geo.valid && sig == n->geo_sig &&
        (uint32_t)(n->now_ms - n->geo_built_ms) < GEO_REFRESH_MS) {
        return;
    }
    const ks_cone_t cone = {n->p.rotor_d_m, n->p.wake_k, n->p.wake_margin_d, n->p.wake_max_d};
    ks_geom_build(&n->geo, &cone, sites, m, n->p.node_id, n->cons.sector);
    n->geo_sig = sig;
    n->geo_built_ms = n->now_ms;
}

static void update_reference(ks_node_t *n)
{
    float u[KS_MAX_NODES];
    int k = 0;
    const bool me_front = n->geo.valid && n->geo.n_up == 0u;
    const bool me_ok = n->u_1s_ok && ks_safety_anemo_usable(&n->sf);
    if (me_front && me_ok) u[k++] = n->u_1s;
    for (int i = 0; i < KS_MAX_NODES && k < KS_MAX_NODES; ++i) {
        const ks_nb_t *e = &n->nb.e[i];
        if (!e->used || !e->has_beacon) continue;
        if ((uint32_t)(n->now_ms - e->last_beacon_ms) > BEACON_FRESH_MS) continue;
        const uint16_t st = e->b.status;
        if ((st & KS_ST_FRONT_ROW) && (st & KS_ST_ANEMO_OK) && ks_bin_sector(e->b.sector) == n->cons.sector) {
            u[k++] = e->b.u_ms;
        }
    }
    if (k > 0) {
        n->uref = ks_median(u, k);
        n->uref_ok = true;
        n->uref_fallback = false;
    } else if (me_ok) {
        n->uref = n->u_1s;
        n->uref_ok = true;
        n->uref_fallback = true;
    } else {
        n->uref_ok = false;
    }
}

/* Which way to steer. The optimal wake-steering direction flips sign as the
 * wind swings across the line joining a turbine to its downstream
 * neighbour; with 5D spacing a 4 degree swing moves the neighbour 0.35 D
 * sideways, as far as steering moves the wake. A fixed signed offset per
 * 10-degree sector therefore averages to almost nothing under ordinary
 * wind wander, while "always deflect away from the neighbour" keeps the
 * gain (docs/multi_agent.md, model sweep). Hysteresis of 0.1 D stops the
 * side from chattering when the neighbour sits on the centre line. */
static void update_side(ks_node_t *n)
{
    if (!n->geo.valid || n->geo.n_down == 0u || !n->cons.init) {
        n->side_valid = false;
        return;
    }
    const ks_nb_t *e = ks_nbtab_cfind(&n->nb, n->geo.down[0]);
    if (e == NULL || !e->has_beacon) return;
    const float phi = ks_cons_dir(&n->cons) * KS_DEG2RAD;
    const float cx = cosf(phi), cy = -sinf(phi); /* left of the wind's travel */
    const float dy = (e->b.x_m - n->p.x_m) * cx + (e->b.y_m - n->p.y_m) * cy;
    const float h = 0.1f * n->p.rotor_d_m;
    const int8_t before = n->side;
    if (!n->side_valid) {
        n->side = (dy >= 0.0f) ? (int8_t)-1 : (int8_t)1;
        n->side_valid = true;
    } else if (n->side < 0 && dy < -h) {
        n->side = 1;
    } else if (n->side > 0 && dy > h) {
        n->side = -1;
    }
    if (n->side != before && before != 0) n->side_flips += 1u;
    n->side_dy = dy;
}

/* Turbulence intensity of the shared free-stream reference. Every node
 * filters the same reference series, so all nodes land in the same stability
 * class without exchanging it; hysteresis keeps the class from chattering. */
static void update_turbulence(ks_node_t *n, float dt_s)
{
    if (n->uref_ok) {
        ks_ewma_update(&n->ti_m, n->uref, dt_s, n->p.ti_tau_s);
        ks_ewma_update(&n->ti_m2, n->uref * n->uref, dt_s, n->p.ti_tau_s);
        n->ti_samples += 1u;
        const float var = n->ti_m2.y - n->ti_m.y * n->ti_m.y;
        n->ti = (n->ti_m.y > 0.5f) ? sqrtf(var > 0.0f ? var : 0.0f) / n->ti_m.y : 0.0f;
        const bool was = n->ti_valid;
        n->ti_valid = n->ti_samples >= (uint32_t)n->p.ti_tau_s;
        if (n->ti_valid && !was) {
            n->ti_class = (n->ti > n->p.ti_split) ? 1u : 0u;
        } else if (n->ti_valid) {
            if (n->ti > n->p.ti_split + n->p.ti_hyst) n->ti_class = 1u;
            else if (n->ti < n->p.ti_split - n->p.ti_hyst) n->ti_class = 0u;
        }
    }
    n->bin_valid = n->cons.sector_valid && n->ti_valid;
    n->bin = (uint8_t)(n->cons.sector + KS_SECTORS * n->ti_class);
}

static uint16_t status_bits(const ks_node_t *n)
{
    uint16_t st = 0u;
    if (n->cons.ok) st |= KS_ST_CONSENSUS_OK;
    if (n->ep.synced) st |= KS_ST_SYNCED;
    if (n->toggle_on) st |= KS_ST_TOGGLE_ON;
    if (n->geo.valid && n->geo.n_up == 0u) st |= KS_ST_FRONT_ROW;
    if (n->tr.st != KS_TR_IDLE) st |= KS_ST_TRIAL_INIT;
    for (int i = 0; i < KS_MAX_OBSERVE; ++i) {
        if (n->obs[i].used) st |= KS_ST_TRIAL_PART;
    }
    if (n->uref_ok) st |= KS_ST_REF_VALID;
    if (n->u_1s_ok && ks_safety_anemo_usable(&n->sf)) st |= KS_ST_ANEMO_OK;
    if (n->dir_1s_ok && ks_safety_vane_usable(&n->sf)) st |= KS_ST_VANE_OK;
    st |= (uint16_t)((uint16_t)n->mode << KS_ST_MODE_SHIFT);
    return st;
}

static void send_beacon(ks_node_t *n, int n_alive)
{
    ks_payload_t pl;
    memset(&pl, 0, sizeof pl);
    ks_beacon_t *b = &pl.beacon;
    b->x_m = n->p.x_m;
    b->y_m = n->p.y_m;
    b->u_ms = n->u_1s_ok ? n->u_1s : 0.0f;
    b->dir_deg = n->dir_1s;
    b->cons_dir_deg = n->cons.init ? ks_cons_dir(&n->cons) : n->dir_1s;
    b->uref_ms = n->uref_ok ? n->uref : 0.0f;
    b->power_w = n->p_1s;
    b->rpm = n->rpm_1s;
    b->yaw_off_deg = n->offset_cmd;
    b->status = status_bits(n);
    b->faults = n->faults;
    const float t = n->t_stator;
    b->temp_c = (t == t) ? (int8_t)ks_clampf(t, -127.0f, 127.0f) : (int8_t)-128;
    b->sector = n->bin_valid ? n->bin : (n->cons.sector_valid ? n->cons.sector : 0u);
    b->beta_u8 = (n->p.utility == KS_UTIL_SELFISH) ? 0u
                 : (uint8_t)lroundf(ks_clampf(n->p.beta, 0.0f, 1.0f) * 255.0f);
    b->n_nb = (uint8_t)(n_alive > 255 ? 255 : n_alive);
    b->uptime_s = (uint32_t)(n->now_ms - n->boot_ms) / 1000u;
    ks__send(n, KS_MSG_BEACON, KS_NODE_BROADCAST, &pl);
}

static void log_state(ks_node_t *n, int n_alive)
{
    ks_logrec_t *r;
    if (n->out == NULL || n->out->n_log >= KS_LOG_MAX) return;
    r = &n->out->log[n->out->n_log++];
    ks_log_state_t s;
    memset(&s, 0, sizeof s);
    s.u_ms = n->u_1s;
    s.dir_deg = n->dir_1s;
    s.cons_dir_deg = n->cons.init ? ks_cons_dir(&n->cons) : NAN;
    s.uref_ms = n->uref_ok ? n->uref : NAN;
    s.power_w = n->p_1s;
    s.rpm = n->rpm_1s;
    s.yaw_meas_deg = n->yaw_meas;
    s.offset_deg = n->offset_cmd;
    s.t_stator_c = n->t_stator;
    s.t_mag_c = ks_safety_magnet_temp(&n->sf);
    s.ke_ratio = ks_safety_ke_ratio(&n->sf);
    const uint32_t into = ks_epoch_ms_into(&n->ep, n->now_ms);
    s.epoch_ms = (uint16_t)(into > 65535u ? 65535u : into);
    s.status = status_bits(n);
    s.faults = n->faults;
    s.sector = n->bin_valid ? n->bin : 0xFFu;
    s.mode = (uint8_t)n->mode;
    s.n_alive = (uint8_t)(n_alive > 255 ? 255 : n_alive);
    s.flags = (uint8_t)((n->uref_fallback ? 1u : 0u) | (n->p.utility == KS_UTIL_SELFISH ? 2u : 0u));
    s.energy_wh = (float)n->energy_wh;
    ks_log_state(r, n->p.node_id, n->now_ms, n->ep.epoch, &s);
}

static void second_tick(ks_node_t *n, const ks_sensors_t *s, float dt_s)
{
    /* 1-s means */
    n->p_1s = n->acc_n ? n->acc_p / n->acc_n : 0.0f;
    n->rpm_1s = n->acc_n ? n->acc_rpm / n->acc_n : 0.0f;
    n->v_1s = n->acc_n ? n->acc_v / n->acc_n : 0.0f;
    n->i_1s = n->acc_n ? n->acc_i / n->acc_n : 0.0f;
    n->u_1s_ok = n->acc_nu > 0u;
    n->u_1s = n->u_1s_ok ? n->acc_u / n->acc_nu : 0.0f;
    n->dir_1s_ok = n->acc_nd > 0u;
    if (n->dir_1s_ok) n->dir_1s = ks_wrap360(atan2f(n->acc_ds, n->acc_dc) * KS_RAD2DEG);
    n->acc_p = n->acc_u = n->acc_rpm = n->acc_v = n->acc_i = n->acc_dc = n->acc_ds = 0.0f;
    n->acc_n = n->acc_nu = n->acc_nd = 0u;

    const int n_alive = ks_nbtab_count_alive(&n->nb, n->now_ms, n->p.neighbor_timeout_ms);

    /* supervisor */
    ks_safety_in_t si;
    memset(&si, 0, sizeof si);
    si.rpm = n->rpm_1s;
    si.u_ms = n->u_1s;
    si.anemo_valid = n->u_1s_ok;
    si.dir_deg = n->dir_1s;
    si.vane_valid = n->dir_1s_ok;
    si.v_bus = n->v_1s;
    si.i_bus = n->i_1s;
    si.t_stator_c = n->t_stator;
    si.t_mcu_c = n->t_mcu;
    si.batt_v = s->batt_v;
    si.n_alive = n_alive;
    si.consensus_ok = n->cons.ok;
    si.driver_faults = s->driver_faults;
    const ks_mode_t old_mode = n->mode;
    n->mode = ks_safety_update(&n->sf, &n->p, &si, n->now_ms);
    n->faults = n->sf.faults;
    if (n->mode != old_mode) {
        ks__log_event(n, KS_EV_MODE, (uint16_t)n->mode, (float)old_mode, (float)n->mode, n->faults, 0u);
        if (n->mode != KS_MODE_NORMAL) ks__neg_abort_all(n, KS_Q_MODE);
    }

    /* consensus over alive neighbours' broadcast estimates */
    float nb_dir[KS_MAX_NODES];
    int k = 0;
    for (int i = 0; i < KS_MAX_NODES; ++i) {
        const ks_nb_t *e = &n->nb.e[i];
        if (!ks_nb_alive(e, n->now_ms, n->p.neighbor_timeout_ms)) continue;
        if (!(e->b.status & (KS_ST_VANE_OK | KS_ST_CONSENSUS_OK))) continue;
        nb_dir[k++] = e->b.cons_dir_deg;
    }
    const bool vane_ok = n->dir_1s_ok && ks_safety_vane_usable(&n->sf);
    ks_cons_update(&n->cons, &n->p, n->dir_1s, vane_ok, nb_dir, k, dt_s);
    update_geometry(n);
    update_side(n);
    update_reference(n);
    update_turbulence(n, dt_s);

    if (n->bin_valid && n->bin != n->bin_logged) {
        if (n->bin_logged != 0xFFu) {
            ks__log_event(n, KS_EV_SECTOR, n->bin, (float)n->bin_logged, (float)n->bin, 0u, 0u);
        }
        n->bin_logged = n->bin;
    }

    /* toggle */
    const bool tog = ks_toggle_state(&n->p, n->ep.epoch);
    if (!n->toggle_known || tog != n->toggle_on) {
        if (n->toggle_known) {
            ks__log_event(n, KS_EV_TOGGLE, tog ? 1u : 0u, tog ? 1.0f : 0.0f, 0.0f, n->ep.epoch, 0u);
            ks__neg_abort_all(n, KS_Q_MODE);
        }
        n->toggle_on = tog;
        n->toggle_known = true;
    }

    /* sacrifice / credit ledger: estimated from measured trial deltas */
    if (n->mode == KS_MODE_NORMAL && n->toggle_on && n->bin_valid) {
        const ks_lut_e_t *e = &n->lut[n->bin];
        const double h = (double)dt_s / 3600.0;
        if (e->led_self_w < 0.0f) n->ledger_sacrifice_wh += -(double)e->led_self_w * h;
        if (e->led_down_w > 0.0f) n->ledger_credit_wh += (double)e->led_down_w * h;
        if (e->recv_w > 0.0f) n->ledger_received_wh += (double)e->recv_w * h;
    }

    send_beacon(n, n_alive);
    log_state(n, n_alive);
}

/* ------------------------------------------------------------------- step */

void ks_node_step(ks_node_t *n, const ks_sensors_t *s, ks_outputs_t *out)
{
    memset(out, 0, sizeof(*out));
    n->out = out;
    const uint32_t prev_ms = n->now_ms;
    n->now_ms = s->t_ms;

    ks_epoch_advance(&n->ep, n->now_ms);
    process_rx(n);
    ks_epoch_advance(&n->ep, n->now_ms);

    if (n->ep.epoch != n->last_epoch) {
        const uint32_t jump = n->ep.epoch - n->last_epoch;
        if (jump > 1u) ks__neg_abort_all(n, KS_Q_MISSED);
        ks__neg_epoch(n, n->ep.epoch);
        n->last_epoch = n->ep.epoch;
    }

    /* raw sample bookkeeping */
    const float p_w = s->v_bus * s->i_bus;
    n->acc_p += p_w;
    n->acc_rpm += s->rpm;
    n->acc_v += s->v_bus;
    n->acc_i += s->i_bus;
    n->acc_n += 1u;
    if (s->anemo_valid) {
        n->acc_u += s->u_ms;
        n->acc_nu += 1u;
    }
    if (s->vane_valid) {
        n->acc_dc += cosf(s->dir_deg * KS_DEG2RAD);
        n->acc_ds += sinf(s->dir_deg * KS_DEG2RAD);
        n->acc_nd += 1u;
    }
    n->t_stator = s->t_stator_c;
    n->t_mcu = s->t_mcu_c;
    n->yaw_meas = s->yaw_deg;
    const uint32_t dt_ms = n->now_ms - prev_ms;
    if (dt_ms > 0u && dt_ms < 5000u) n->energy_wh += (double)p_w * (double)dt_ms / 3.6e6;

    ks__neg_sample(n, p_w);

    if ((uint32_t)(n->now_ms - n->sec_start_ms) >= 1000u) {
        const uint32_t el = n->now_ms - n->sec_start_ms;
        n->sec_start_ms = (el < 2000u) ? n->sec_start_ms + 1000u : n->now_ms;
        second_tick(n, s, (float)el * 1e-3f);
    }

    ks__neg_tick(n);

    /* yaw command */
    float off = 0.0f;
    switch (n->mode) {
    case KS_MODE_SHUTDOWN: off = n->p.furl_offset_deg; break;
    case KS_MODE_DERATE: off = n->p.derate_offset_deg; break;
    case KS_MODE_SAFE: off = 0.0f; break;
    case KS_MODE_NORMAL: off = n->toggle_on ? ks__neg_offset(n) : 0.0f; break;
    }
    n->offset_cmd = off;
    const float wind = n->cons.init ? ks_cons_dir(&n->cons) : s->dir_deg;
    out->offset_deg = off;
    out->yaw_cmd_deg = ks_wrap360(wind + off);
    out->yaw_enable = n->cons.init;
    if (n->mode == KS_MODE_SHUTDOWN) {
        const float t_mag = ks_safety_magnet_temp(&n->sf);
        const bool hot = (t_mag == t_mag) && t_mag > n->p.brake_max_magnet_c;
        out->brake = (n->faults & KS_F_OVERSPEED) || !hot;
    } else {
        out->brake = false;
    }
    out->mode = n->mode;
    out->status = status_bits(n);
    out->faults = n->faults;
    n->out = NULL;
}

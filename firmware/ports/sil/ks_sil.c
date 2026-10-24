/*
 * ks_sil.c - software-in-the-loop shim (host only).
 */
#include "ks_sil.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ks/ks_consensus.h"
#include "ks/ks_crypto.h"
#include "ks/ks_geom.h"
#include "ks/ks_node.h"

typedef struct {
    ks_params_t p;
    ks_node_t node;
    ks_outputs_t out;
    int initialised;
} sil_t;

int ks_sil_abi_version(void) { return 1; }

void *ks_sil_new(void)
{
    sil_t *s = (sil_t *)calloc(1, sizeof(sil_t));
    if (s) ks_params_default(&s->p);
    return s;
}

void ks_sil_free(void *h) { free(h); }

int ks_sil_set(void *h, const char *name, double value)
{
    sil_t *s = (sil_t *)h;
    int rc = ks_params_set(&s->p, name, value);
    if (rc == 0 && s->initialised) s->node.p = s->p; /* live retune */
    return rc;
}

int ks_sil_get_param(void *h, const char *name, double *value)
{
    return ks_params_get(&((sil_t *)h)->p, name, value);
}

void ks_sil_set_key(void *h, const uint8_t key[16])
{
    sil_t *s = (sil_t *)h;
    memcpy(s->p.key, key, 16);
    if (s->initialised) memcpy(s->node.p.key, key, 16);
}

int ks_sil_init(void *h, uint16_t boot, uint32_t now_ms)
{
    sil_t *s = (sil_t *)h;
    const int v = ks_params_validate(&s->p);
    if (v != 0) return v;
    ks_node_init(&s->node, &s->p, boot, now_ms);
    s->initialised = 1;
    return 0;
}

int ks_sil_rx(void *h, const uint8_t *data, int len, int rssi, uint32_t t_ms)
{
    if (len < 0) return -2;
    return ks_node_rx(&((sil_t *)h)->node, data, (size_t)len, (int8_t)rssi, t_ms);
}

void ks_sil_step(void *h, const double *v, double *o)
{
    sil_t *s = (sil_t *)h;
    ks_sensors_t in;
    memset(&in, 0, sizeof in);
    in.t_ms = (uint32_t)v[KS_SIL_T_MS];
    in.rpm = (float)v[KS_SIL_RPM];
    in.v_bus = (float)v[KS_SIL_VBUS];
    in.i_bus = (float)v[KS_SIL_IBUS];
    in.u_ms = (float)v[KS_SIL_U];
    in.anemo_valid = v[KS_SIL_ANEMO_VALID] != 0.0;
    in.dir_deg = (float)v[KS_SIL_DIR];
    in.vane_valid = v[KS_SIL_VANE_VALID] != 0.0;
    in.yaw_deg = (float)v[KS_SIL_YAW];
    in.t_stator_c = (float)v[KS_SIL_TSTATOR];
    in.t_mcu_c = (float)v[KS_SIL_TMCU];
    in.batt_v = (float)v[KS_SIL_BATT];
    in.driver_faults = (uint16_t)v[KS_SIL_DRVFAULTS];
    ks_node_step(&s->node, &in, &s->out);
    o[KS_SIL_O_YAWCMD] = s->out.yaw_cmd_deg;
    o[KS_SIL_O_OFFSET] = s->out.offset_deg;
    o[KS_SIL_O_YAWEN] = s->out.yaw_enable;
    o[KS_SIL_O_BRAKE] = s->out.brake;
    o[KS_SIL_O_MODE] = s->out.mode;
    o[KS_SIL_O_STATUS] = s->out.status;
    o[KS_SIL_O_FAULTS] = s->out.faults;
    o[KS_SIL_O_NTX] = s->out.n_tx;
    o[KS_SIL_O_NLOG] = s->out.n_log;
    o[KS_SIL_O_PERSIST] = s->out.persist_lut;
}

void ks_sil_step_batch(void **h, int n, const double *sens, double *outv)
{
    for (int k = 0; k < n; ++k) {
        ks_sil_step(h[k], sens + (size_t)k * KS_SIL_NSENS, outv + (size_t)k * KS_SIL_NOUT);
    }
}

int ks_sil_tx(void *h, int i, uint8_t *buf, int cap)
{
    sil_t *s = (sil_t *)h;
    if (i < 0 || i >= s->out.n_tx) return -1;
    const ks_frame_t *f = &s->out.tx[i];
    if (f->len > cap) return -2;
    memcpy(buf, f->data, f->len);
    return f->len;
}

int ks_sil_log(void *h, int i, uint8_t *buf64)
{
    sil_t *s = (sil_t *)h;
    if (i < 0 || i >= s->out.n_log) return -1;
    memcpy(buf64, s->out.log[i].b, KS_LOG_REC_BYTES);
    return KS_LOG_REC_BYTES;
}

double ks_sil_query(void *h, const char *w)
{
    const ks_node_t *n = &((sil_t *)h)->node;
#define Q(name, expr) if (strcmp(w, name) == 0) return (double)(expr)
    Q("offset", n->offset_cmd);
    Q("sector", n->cons.sector_valid ? n->cons.sector : -1);
    Q("bin", n->bin_valid ? n->bin : -1);
    Q("ti", n->ti_valid ? n->ti : NAN);
    Q("ti_class", n->ti_valid ? n->ti_class : -1);
    Q("mode", n->mode);
    Q("faults", n->faults);
    Q("epoch", n->ep.epoch);
    Q("epoch_ms", ks_epoch_ms_into(&n->ep, n->now_ms));
    Q("synced", n->ep.synced);
    Q("epoch_adjust", n->ep.n_adjust);
    Q("cons_dir", n->cons.init ? ks_cons_dir(&n->cons) : NAN);
    Q("cons_ok", n->cons.ok);
    Q("cons_spread", n->cons.spread_deg);
    Q("uref", n->uref_ok ? n->uref : NAN);
    Q("uref_fallback", n->uref_fallback);
    Q("n_alive", ks_nbtab_count_alive(&n->nb, n->now_ms, n->p.neighbor_timeout_ms));
    Q("n_trials", n->n_trials);
    Q("n_accept", n->n_accept);
    Q("n_reject", n->n_reject);
    Q("n_abort", n->n_abort);
    Q("n_hold", n->n_hold);
    Q("kappa0", sqrt((double)n->nc.kap2[0]));
    Q("kappa1", sqrt((double)n->nc.kap2[1]));
    Q("nullcal_n0", n->nc.n[0]);
    Q("nullcal_n1", n->nc.n[1]);
    Q("n_conflicts", n->n_conflicts);
    Q("n_auth_fail", n->n_auth_fail);
    Q("rx_dropped", n->rx_dropped);
    Q("ledger_sacrifice_wh", n->ledger_sacrifice_wh);
    Q("ledger_credit_wh", n->ledger_credit_wh);
    Q("ledger_received_wh", n->ledger_received_wh);
    Q("energy_wh", n->energy_wh);
    Q("geo_valid", n->geo.valid);
    Q("geo_color", n->geo.valid && n->geo.self >= 0 ? n->geo.color[n->geo.self] : -1);
    Q("geo_ncolors", n->geo.n_colors);
    Q("geo_ndown", n->geo.n_down);
    Q("geo_nup", n->geo.n_up);
    Q("tr_state", n->tr.st);
    Q("p_1s", n->p_1s);
    Q("u_1s", n->u_1s);
    Q("toggle_on", n->toggle_on);
    Q("t_mag", ks_safety_magnet_temp(&n->sf));
    Q("side", n->side_valid ? n->side : 0);
    Q("side_dy", n->side_dy);
    Q("side_flips", n->side_flips);
    Q("ke_ratio", ks_safety_ke_ratio(&n->sf));
#undef Q
    return NAN;
}

void ks_sil_lut_set(void *h, int sector, double off_deg)
{
    if (sector < 0 || sector >= KS_BINS) return;
    ks_node_lut_set(&((sil_t *)h)->node, (uint8_t)sector, (float)off_deg);
}

double ks_sil_lut_get(void *h, int sector, const char *f)
{
    if (sector < 0 || sector >= KS_BINS) return NAN;
    const ks_lut_e_t *e = &((sil_t *)h)->node.lut[sector];
    if (strcmp(f, "off") == 0) return e->off_deg;
    if (strcmp(f, "step") == 0) return e->step_deg;
    if (strcmp(f, "dir") == 0) return e->dir;
    if (strcmp(f, "n_trials") == 0) return e->n_trials;
    if (strcmp(f, "n_accept") == 0) return e->n_accept;
    if (strcmp(f, "led_self") == 0) return e->led_self_w;
    if (strcmp(f, "led_down") == 0) return e->led_down_w;
    if (strcmp(f, "recv") == 0) return e->recv_w;
    if (strcmp(f, "backoff") == 0) return e->backoff;
    if (strcmp(f, "pool_n") == 0) return e->pool_n;
    if (strcmp(f, "pool_z") == 0) return e->pool_n ? (double)(e->pool_sw / sqrtf(e->pool_w)) : 0.0;
    return NAN;
}

uint64_t ks_sil_siphash(const uint8_t key[16], const uint8_t *msg, int len)
{
    return ks_siphash24(key, msg, (size_t)(len < 0 ? 0 : len));
}

uint16_t ks_sil_crc16(const uint8_t *msg, int len)
{
    return ks_crc16_ccitt(msg, (size_t)(len < 0 ? 0 : len));
}

int ks_sil_sector_of(double dir_deg) { return ks_sector_of((float)dir_deg); }

int ks_sil_geom(const uint16_t *ids, const double *x, const double *y, int n, int self_idx,
                int sector, double rotor_d, double k, double margin_d, double max_d,
                uint8_t *colors, uint16_t *down, int *n_up)
{
    if (n <= 0 || n > KS_MAX_NODES || self_idx < 0 || self_idx >= n || sector < 0 ||
        sector >= KS_SECTORS) {
        return -1;
    }
    static ks_site_t sites[KS_MAX_NODES];
    static ks_geom_t g;
    for (int i = 0; i < n; ++i) {
        sites[i].id = ids[i];
        sites[i].x = (float)x[i];
        sites[i].y = (float)y[i];
    }
    const ks_cone_t c = {(float)rotor_d, (float)k, (float)margin_d, (float)max_d};
    ks_geom_build(&g, &c, sites, n, ids[self_idx], (uint8_t)sector);
    for (int i = 0; i < n; ++i) colors[i] = ks_geom_color_of(&g, ids[i]);
    for (int i = 0; i < g.n_down; ++i) down[i] = g.down[i];
    if (n_up) *n_up = g.n_up;
    return g.n_down;
}

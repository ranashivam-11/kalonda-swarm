/*
 * ks_log.c
 */
#include "ks/ks_log.h"

#include <math.h>
#include <string.h>

#include "ks/ks_bytes.h"
#include "ks/ks_crypto.h"

static void head(ks_logrec_t *r, uint8_t type, uint16_t node, uint32_t t_ms, uint32_t epoch)
{
    memset(r->b, 0, sizeof r->b);
    r->b[0] = type;
    r->b[1] = 1u;
    ks_put_u16(r->b + 2, node);
    ks_put_u32(r->b + 4, t_ms);
    ks_put_u32(r->b + 8, epoch);
}

static void seal(ks_logrec_t *r)
{
    ks_put_u16(r->b + 62, ks_crc16_ccitt(r->b, 62));
}

static int16_t i16_scaled(float v, float scale)
{
    if (v != v) return INT16_MIN; /* NaN sentinel */
    float x = v * scale;
    if (x > 32767.0f) x = 32767.0f;
    if (x < -32767.0f) x = -32767.0f;
    return (int16_t)lroundf(x);
}

void ks_log_state(ks_logrec_t *r, uint16_t node, uint32_t t_ms, uint32_t epoch,
                  const ks_log_state_t *s)
{
    head(r, KS_LOG_STATE, node, t_ms, epoch);
    uint8_t *p = r->b + 12;
    ks_put_f32(p + 0, s->u_ms);
    ks_put_f32(p + 4, s->dir_deg);
    ks_put_f32(p + 8, s->cons_dir_deg);
    ks_put_f32(p + 12, s->uref_ms);
    ks_put_f32(p + 16, s->power_w);
    ks_put_f32(p + 20, s->rpm);
    ks_put_f32(p + 24, s->yaw_meas_deg);
    ks_put_u16(p + 28, (uint16_t)i16_scaled(s->offset_deg, 100.0f));
    ks_put_u16(p + 30, (uint16_t)i16_scaled(s->t_stator_c, 10.0f));
    ks_put_u16(p + 32, (uint16_t)i16_scaled(s->t_mag_c, 10.0f));
    ks_put_u16(p + 34, (uint16_t)i16_scaled(s->ke_ratio, 10000.0f));
    ks_put_u16(p + 36, s->epoch_ms);
    ks_put_u16(p + 38, s->status);
    ks_put_u16(p + 40, s->faults);
    p[42] = s->sector;
    p[43] = s->mode;
    p[44] = s->n_alive;
    p[45] = s->flags;
    ks_put_f32(p + 46, s->energy_wh);
    seal(r);
}

void ks_log_trial(ks_logrec_t *r, uint16_t node, uint32_t t_ms, uint32_t epoch,
                  const ks_log_trial_t *t)
{
    head(r, KS_LOG_TRIAL, node, t_ms, epoch);
    r->b[1] = 2u;
    uint8_t *p = r->b + 12;
    ks_put_u16(p + 0, t->trial_id);
    ks_put_u16(p + 2, t->initiator);
    p[4] = t->event;
    p[5] = t->role;
    p[6] = t->sector;
    p[7] = t->outcome;
    ks_put_u16(p + 8, (uint16_t)i16_scaled(t->base_deg, 100.0f));
    ks_put_u16(p + 10, (uint16_t)i16_scaled(t->trial_deg, 100.0f));
    ks_put_f32(p + 12, t->delta_w);
    ks_put_f32(p + 16, t->se_w);
    ks_put_f32(p + 20, t->score_w);
    ks_put_f32(p + 24, t->score_se_w);
    ks_put_f32(p + 28, t->coop_score_w);
    for (int i = 0; i < KS_PHASES; ++i) {
        ks_put_u16(p + 32 + 2 * i, (uint16_t)i16_scaled(t->phase_mean[i], 50.0f));
    }
    ks_put_f32(p + 40, t->pool_mean_w);
    ks_put_u16(p + 44, (uint16_t)i16_scaled(t->pool_z, 1000.0f));
    p[46] = t->pool_n;
    p[48] = t->n_reports;
    p[49] = t->quality;
    seal(r);
}

void ks_log_event(ks_logrec_t *r, uint16_t node, uint32_t t_ms, uint32_t epoch,
                  const ks_log_event_t *e)
{
    head(r, KS_LOG_EVENT, node, t_ms, epoch);
    uint8_t *p = r->b + 12;
    ks_put_u16(p + 0, e->code);
    ks_put_u16(p + 2, e->arg);
    for (int i = 0; i < 4; ++i) {
        ks_put_f32(p + 4 + 4 * i, e->a[i]);
    }
    ks_put_u32(p + 20, e->u[0]);
    ks_put_u32(p + 24, e->u[1]);
    seal(r);
}

int ks_log_check(const ks_logrec_t *r)
{
    return ks_get_u16(r->b + 62) == ks_crc16_ccitt(r->b, 62);
}

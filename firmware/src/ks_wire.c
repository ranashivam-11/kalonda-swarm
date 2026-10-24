/*
 * ks_wire.c - encode/decode of authenticated protocol frames.
 */
#include "ks/ks_wire.h"

#include <math.h>
#include <string.h>

#include "ks/ks_bytes.h"
#include "ks/ks_crypto.h"
#include "ks/ks_stats.h"

#define BEACON_LEN 30u
#define PROPOSE_FIXED 12u
#define REPORT_LEN 19u
#define DECIDE_LEN 20u

/* --- fixed-point helpers ------------------------------------------------ */

static uint16_t q_u16(float v, float scale)
{
    float x = v * scale;
    if (!(x > 0.0f)) return 0u;       /* also maps NaN to 0 */
    if (x > 65535.0f) return 65535u;
    return (uint16_t)lroundf(x);
}

static int16_t q_i16(float v, float scale)
{
    float x = v * scale;
    if (x != x) return 0;             /* NaN */
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lroundf(x);
}

static uint16_t q_angle(float deg)
{
    uint32_t c = (uint32_t)lroundf(ks_wrap360(deg) * 100.0f);
    return (uint16_t)(c % 36000u);
}

size_t ks_wire_payload_len(uint8_t type, const ks_payload_t *pl)
{
    switch (type) {
    case KS_MSG_BEACON: return BEACON_LEN;
    case KS_MSG_PROPOSE:
        return PROPOSE_FIXED + 2u * (size_t)(pl ? pl->propose.n_part : 0u);
    case KS_MSG_REPORT: return REPORT_LEN;
    case KS_MSG_DECIDE: return DECIDE_LEN;
    default: return 0u;
    }
}

static void put_hdr(uint8_t *b, const ks_hdr_t *h)
{
    b[0] = (uint8_t)((KS_PROTO_VERSION << 4) | (h->type & 0x0Fu));
    b[1] = h->flags;
    ks_put_u16(b + 2, h->src);
    ks_put_u16(b + 4, h->dst);
    ks_put_u16(b + 6, h->boot);
    ks_put_u16(b + 8, h->seq);
    ks_put_u32(b + 10, h->epoch);
    ks_put_u16(b + 14, h->epoch_ms);
}

static void put_payload(uint8_t *b, uint8_t type, const ks_payload_t *pl)
{
    switch (type) {
    case KS_MSG_BEACON: {
        const ks_beacon_t *m = &pl->beacon;
        ks_put_u16(b + 0, (uint16_t)q_i16(m->x_m, 10.0f));
        ks_put_u16(b + 2, (uint16_t)q_i16(m->y_m, 10.0f));
        ks_put_u16(b + 4, q_u16(m->u_ms, 100.0f));
        ks_put_u16(b + 6, q_angle(m->dir_deg));
        ks_put_u16(b + 8, q_angle(m->cons_dir_deg));
        ks_put_u16(b + 10, q_u16(m->uref_ms, 100.0f));
        ks_put_u16(b + 12, q_u16(m->power_w, 10.0f));
        ks_put_u16(b + 14, q_u16(m->rpm, 10.0f));
        ks_put_u16(b + 16, (uint16_t)q_i16(m->yaw_off_deg, 100.0f));
        ks_put_u16(b + 18, m->status);
        ks_put_u16(b + 20, m->faults);
        b[22] = (uint8_t)m->temp_c;
        b[23] = m->sector;
        b[24] = m->beta_u8;
        b[25] = m->n_nb;
        ks_put_u32(b + 26, m->uptime_s);
        break;
    }
    case KS_MSG_PROPOSE: {
        const ks_propose_t *m = &pl->propose;
        ks_put_u16(b + 0, m->trial_id);
        b[2] = m->sector;
        b[3] = m->n_part;
        ks_put_u16(b + 4, (uint16_t)q_i16(m->base_deg, 100.0f));
        ks_put_u16(b + 6, (uint16_t)q_i16(m->trial_deg, 100.0f));
        ks_put_u32(b + 8, m->start_epoch);
        for (uint8_t i = 0; i < m->n_part; ++i) {
            ks_put_u16(b + 12 + 2u * i, m->part[i]);
        }
        break;
    }
    case KS_MSG_REPORT: {
        const ks_report_t *m = &pl->report;
        ks_put_u16(b + 0, m->trial_id);
        ks_put_u16(b + 2, m->initiator);
        ks_put_f32(b + 4, m->delta_w);
        ks_put_f32(b + 8, m->se_w);
        ks_put_f32(b + 12, m->base_w);
        ks_put_u16(b + 16, m->n_blocks);
        b[18] = m->quality;
        break;
    }
    case KS_MSG_DECIDE: {
        const ks_decide_t *m = &pl->decide;
        ks_put_u16(b + 0, m->trial_id);
        b[2] = m->sector;
        b[3] = m->outcome;
        ks_put_u16(b + 4, (uint16_t)q_i16(m->new_deg, 100.0f));
        ks_put_f32(b + 6, m->score_w);
        ks_put_f32(b + 10, m->score_se_w);
        ks_put_f32(b + 14, m->coop_score_w);
        b[18] = m->n_reports;
        b[19] = m->n_part;
        break;
    }
    default: break;
    }
}

size_t ks_wire_encode(uint8_t *buf, size_t cap, const ks_hdr_t *h,
                      const ks_payload_t *pl, const uint8_t key[16])
{
    if (h->type == KS_MSG_PROPOSE && pl->propose.n_part > KS_MAX_PART) {
        return 0u;
    }
    const size_t plen = ks_wire_payload_len(h->type, pl);
    if (plen == 0u) {
        return 0u;
    }
    const size_t total = KS_HDR_BYTES + plen + KS_TAG_BYTES;
    if (total > cap || total > KS_FRAME_MAX) {
        return 0u;
    }
    put_hdr(buf, h);
    put_payload(buf + KS_HDR_BYTES, h->type, pl);
    const uint64_t tag = ks_siphash24(key, buf, KS_HDR_BYTES + plen);
    for (unsigned i = 0; i < KS_TAG_BYTES; ++i) {
        buf[KS_HDR_BYTES + plen + i] = (uint8_t)(tag >> (8u * i));
    }
    return total;
}

static int finite_f(float x) { return x == x && x < 3.0e38f && x > -3.0e38f; }

int ks_wire_decode(const uint8_t *buf, size_t len, const uint8_t key[16],
                   ks_hdr_t *h, ks_payload_t *pl)
{
    if (len < KS_HDR_BYTES + KS_TAG_BYTES || len > KS_FRAME_MAX) {
        return -1;
    }
    if ((buf[0] >> 4) != KS_PROTO_VERSION) {
        return -2;
    }
    const size_t body = len - KS_TAG_BYTES;
    const uint64_t tag = ks_siphash24(key, buf, body);
    uint8_t expect[KS_TAG_BYTES];
    for (unsigned i = 0; i < KS_TAG_BYTES; ++i) {
        expect[i] = (uint8_t)(tag >> (8u * i));
    }
    if (!ks_ct_equal(expect, buf + body, KS_TAG_BYTES)) {
        return -3;
    }

    h->version = (uint8_t)(buf[0] >> 4);
    h->type = (uint8_t)(buf[0] & 0x0Fu);
    h->flags = buf[1];
    h->src = ks_get_u16(buf + 2);
    h->dst = ks_get_u16(buf + 4);
    h->boot = ks_get_u16(buf + 6);
    h->seq = ks_get_u16(buf + 8);
    h->epoch = ks_get_u32(buf + 10);
    h->epoch_ms = ks_get_u16(buf + 14);

    const uint8_t *b = buf + KS_HDR_BYTES;
    const size_t plen = body - KS_HDR_BYTES;
    memset(pl, 0, sizeof(*pl));

    switch (h->type) {
    case KS_MSG_BEACON: {
        if (plen != BEACON_LEN) return -5;
        ks_beacon_t *m = &pl->beacon;
        const uint16_t dir = ks_get_u16(b + 6), cdir = ks_get_u16(b + 8);
        if (dir >= 36000u || cdir >= 36000u || b[23] >= KS_BINS) return -6;
        m->x_m = (float)(int16_t)ks_get_u16(b + 0) / 10.0f;
        m->y_m = (float)(int16_t)ks_get_u16(b + 2) / 10.0f;
        m->u_ms = (float)ks_get_u16(b + 4) / 100.0f;
        m->dir_deg = (float)dir / 100.0f;
        m->cons_dir_deg = (float)cdir / 100.0f;
        m->uref_ms = (float)ks_get_u16(b + 10) / 100.0f;
        m->power_w = (float)ks_get_u16(b + 12) / 10.0f;
        m->rpm = (float)ks_get_u16(b + 14) / 10.0f;
        m->yaw_off_deg = (float)(int16_t)ks_get_u16(b + 16) / 100.0f;
        m->status = ks_get_u16(b + 18);
        m->faults = ks_get_u16(b + 20);
        m->temp_c = (int8_t)b[22];
        m->sector = b[23];
        m->beta_u8 = b[24];
        m->n_nb = b[25];
        m->uptime_s = ks_get_u32(b + 26);
        return 0;
    }
    case KS_MSG_PROPOSE: {
        if (plen < PROPOSE_FIXED) return -5;
        ks_propose_t *m = &pl->propose;
        m->trial_id = ks_get_u16(b + 0);
        m->sector = b[2];
        m->n_part = b[3];
        if (m->n_part > KS_MAX_PART || m->sector >= KS_BINS) return -6;
        if (plen != PROPOSE_FIXED + 2u * (size_t)m->n_part) return -5;
        m->base_deg = (float)(int16_t)ks_get_u16(b + 4) / 100.0f;
        m->trial_deg = (float)(int16_t)ks_get_u16(b + 6) / 100.0f;
        if (fabsf(m->base_deg) > 90.0f || fabsf(m->trial_deg) > 90.0f) return -6;
        m->start_epoch = ks_get_u32(b + 8);
        for (uint8_t i = 0; i < m->n_part; ++i) {
            m->part[i] = ks_get_u16(b + 12 + 2u * i);
        }
        return 0;
    }
    case KS_MSG_REPORT: {
        if (plen != REPORT_LEN) return -5;
        ks_report_t *m = &pl->report;
        m->trial_id = ks_get_u16(b + 0);
        m->initiator = ks_get_u16(b + 2);
        m->delta_w = ks_get_f32(b + 4);
        m->se_w = ks_get_f32(b + 8);
        m->base_w = ks_get_f32(b + 12);
        m->n_blocks = ks_get_u16(b + 16);
        m->quality = b[18];
        if (!finite_f(m->delta_w) || !finite_f(m->se_w) || !finite_f(m->base_w) ||
            m->se_w < 0.0f) {
            return -6;
        }
        return 0;
    }
    case KS_MSG_DECIDE: {
        if (plen != DECIDE_LEN) return -5;
        ks_decide_t *m = &pl->decide;
        m->trial_id = ks_get_u16(b + 0);
        m->sector = b[2];
        m->outcome = b[3];
        if (m->sector >= KS_BINS || m->outcome > KS_OUT_HOLD) return -6;
        m->new_deg = (float)(int16_t)ks_get_u16(b + 4) / 100.0f;
        m->score_w = ks_get_f32(b + 6);
        m->score_se_w = ks_get_f32(b + 10);
        m->coop_score_w = ks_get_f32(b + 14);
        m->n_reports = b[18];
        m->n_part = b[19];
        if (!finite_f(m->score_w) || !finite_f(m->score_se_w) || !finite_f(m->coop_score_w)) {
            return -6;
        }
        return 0;
    }
    default:
        return -4;
    }
}

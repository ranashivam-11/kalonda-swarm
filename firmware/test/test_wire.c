/*
 * test_wire.c - frame round trips, authentication and malformed input.
 */
#include <string.h>

#include "ks/ks_wire.h"
#include "ktest.h"

static const uint8_t KEY[16] = {0x4b, 0x61, 0x6c, 0x6f, 0x6e, 0x64, 0x61, 0x2d,
                                0x73, 0x77, 0x61, 0x72, 0x6d, 0x2d, 0x76, 0x31};

static ks_hdr_t hdr(uint8_t type)
{
    ks_hdr_t h;
    memset(&h, 0, sizeof h);
    h.version = KS_PROTO_VERSION;
    h.type = type;
    h.src = 7;
    h.dst = KS_NODE_BROADCAST;
    h.boot = 3;
    h.seq = 41;
    h.epoch = 123456;
    h.epoch_ms = 4321;
    return h;
}

static void test_beacon_roundtrip(void)
{
    ks_payload_t a, b;
    memset(&a, 0, sizeof a);
    a.beacon.x_m = -12.3f;
    a.beacon.y_m = 45.6f;
    a.beacon.u_ms = 4.87f;
    a.beacon.dir_deg = 271.25f;
    a.beacon.cons_dir_deg = 359.99f;
    a.beacon.uref_ms = 5.01f;
    a.beacon.power_w = 72.1f;
    a.beacon.rpm = 280.4f;
    a.beacon.yaw_off_deg = -12.34f;
    a.beacon.status = 0xA5A5u;
    a.beacon.faults = 0x0102u;
    a.beacon.temp_c = 47;
    a.beacon.sector = 27;
    a.beacon.beta_u8 = 255;
    a.beacon.n_nb = 11;
    a.beacon.uptime_s = 86400u * 30u;
    ks_hdr_t h = hdr(KS_MSG_BEACON), h2;
    uint8_t buf[KS_FRAME_MAX];
    const size_t len = ks_wire_encode(buf, sizeof buf, &h, &a, KEY);
    KT_CHECK(len == KS_HDR_BYTES + 30u + KS_TAG_BYTES);
    KT_CHECK(ks_wire_decode(buf, len, KEY, &h2, &b) == 0);
    KT_CHECK(h2.src == 7 && h2.boot == 3 && h2.seq == 41 && h2.epoch == 123456u && h2.epoch_ms == 4321);
    KT_NEAR(b.beacon.x_m, -12.3, 0.051);
    KT_NEAR(b.beacon.y_m, 45.6, 0.051);
    KT_NEAR(b.beacon.u_ms, 4.87, 0.006);
    KT_NEAR(b.beacon.dir_deg, 271.25, 0.006);
    KT_NEAR(b.beacon.cons_dir_deg, 359.99, 0.006);
    KT_NEAR(b.beacon.power_w, 72.1, 0.051);
    KT_NEAR(b.beacon.rpm, 280.4, 0.051);
    KT_NEAR(b.beacon.yaw_off_deg, -12.34, 0.006);
    KT_CHECK(b.beacon.status == 0xA5A5u && b.beacon.faults == 0x0102u);
    KT_CHECK(b.beacon.temp_c == 47 && b.beacon.sector == 27 && b.beacon.beta_u8 == 255);
    KT_CHECK(b.beacon.uptime_s == 86400u * 30u);
}

static void test_propose_roundtrip_variable_length(void)
{
    for (uint8_t np = 0; np <= KS_MAX_PART; ++np) {
        ks_payload_t a, b;
        memset(&a, 0, sizeof a);
        a.propose.trial_id = 0xBEEF;
        a.propose.sector = 27;
        a.propose.n_part = np;
        a.propose.base_deg = 6.0f;
        a.propose.trial_deg = 12.5f;
        a.propose.start_epoch = 99;
        for (uint8_t i = 0; i < np; ++i) a.propose.part[i] = (uint16_t)(100 + i);
        ks_hdr_t h = hdr(KS_MSG_PROPOSE), h2;
        uint8_t buf[KS_FRAME_MAX];
        const size_t len = ks_wire_encode(buf, sizeof buf, &h, &a, KEY);
        KT_CHECK(len == KS_HDR_BYTES + 12u + 2u * np + KS_TAG_BYTES);
        KT_CHECK(ks_wire_decode(buf, len, KEY, &h2, &b) == 0);
        KT_CHECK(b.propose.n_part == np && b.propose.trial_id == 0xBEEF);
        KT_NEAR(b.propose.trial_deg, 12.5, 1e-6);
        for (uint8_t i = 0; i < np; ++i) KT_CHECK(b.propose.part[i] == 100 + i);
    }
}

static void test_report_decide_roundtrip(void)
{
    ks_payload_t a, b;
    memset(&a, 0, sizeof a);
    a.report.trial_id = 9;
    a.report.initiator = 2;
    a.report.delta_w = 3.25f;
    a.report.se_w = 0.75f;
    a.report.base_w = 40.0f;
    a.report.n_blocks = 32;
    a.report.quality = KS_Q_NOREF;
    ks_hdr_t h = hdr(KS_MSG_REPORT), h2;
    uint8_t buf[KS_FRAME_MAX];
    size_t len = ks_wire_encode(buf, sizeof buf, &h, &a, KEY);
    KT_CHECK(ks_wire_decode(buf, len, KEY, &h2, &b) == 0);
    KT_CHECK(b.report.delta_w == 3.25f && b.report.se_w == 0.75f && b.report.n_blocks == 32);

    memset(&a, 0, sizeof a);
    a.decide.trial_id = 9;
    a.decide.sector = 3;
    a.decide.outcome = KS_OUT_ACCEPT;
    a.decide.new_deg = -8.0f;
    a.decide.score_w = 1.5f;
    a.decide.score_se_w = 0.5f;
    a.decide.coop_score_w = 1.5f;
    a.decide.n_reports = 2;
    a.decide.n_part = 2;
    h = hdr(KS_MSG_DECIDE);
    len = ks_wire_encode(buf, sizeof buf, &h, &a, KEY);
    KT_CHECK(len == KS_HDR_BYTES + 20u + KS_TAG_BYTES);
    KT_CHECK(ks_wire_decode(buf, len, KEY, &h2, &b) == 0);
    KT_CHECK(b.decide.outcome == KS_OUT_ACCEPT && b.decide.n_reports == 2);
    KT_NEAR(b.decide.new_deg, -8.0, 1e-6);
}

static void test_tamper_and_wrong_key_rejected(void)
{
    ks_payload_t a, b;
    memset(&a, 0, sizeof a);
    a.report.delta_w = 1.0f;
    ks_hdr_t h = hdr(KS_MSG_REPORT), h2;
    uint8_t buf[KS_FRAME_MAX];
    const size_t len = ks_wire_encode(buf, sizeof buf, &h, &a, KEY);
    for (size_t i = 0; i < len; ++i) {
        uint8_t t[KS_FRAME_MAX];
        memcpy(t, buf, len);
        t[i] ^= 0x01u;
        const int rc = ks_wire_decode(t, len, KEY, &h2, &b);
        KT_CHECK(rc == -3 || (i == 0 && rc == -2)); /* version nibble or tag */
    }
    uint8_t other[16];
    memcpy(other, KEY, 16);
    other[0] ^= 1u;
    KT_CHECK(ks_wire_decode(buf, len, other, &h2, &b) == -3);
}

static void test_malformed_rejected(void)
{
    ks_hdr_t h2;
    ks_payload_t b;
    uint8_t buf[KS_FRAME_MAX];
    memset(buf, 0, sizeof buf);
    KT_CHECK(ks_wire_decode(buf, 5, KEY, &h2, &b) == -1);
    KT_CHECK(ks_wire_decode(buf, KS_FRAME_MAX + 1u, KEY, &h2, &b) == -1);

    /* A forged-but-authenticated report with NaN must still be refused. */
    ks_payload_t a;
    memset(&a, 0, sizeof a);
    a.report.se_w = -1.0f;
    ks_hdr_t h = hdr(KS_MSG_REPORT);
    const size_t len = ks_wire_encode(buf, sizeof buf, &h, &a, KEY);
    KT_CHECK(ks_wire_decode(buf, len, KEY, &h2, &b) == -6);

    /* too many participants cannot be encoded */
    memset(&a, 0, sizeof a);
    a.propose.n_part = KS_MAX_PART + 1;
    h = hdr(KS_MSG_PROPOSE);
    KT_CHECK(ks_wire_encode(buf, sizeof buf, &h, &a, KEY) == 0u);

    /* unknown type */
    memset(&a, 0, sizeof a);
    h = hdr(9);
    KT_CHECK(ks_wire_encode(buf, sizeof buf, &h, &a, KEY) == 0u);
}

static void test_all_frames_fit_espnow_v1(void)
{
    ks_payload_t a;
    memset(&a, 0, sizeof a);
    a.propose.n_part = KS_MAX_PART;
    KT_CHECK(KS_HDR_BYTES + ks_wire_payload_len(KS_MSG_PROPOSE, &a) + KS_TAG_BYTES <= 250u);
    KT_CHECK(KS_HDR_BYTES + ks_wire_payload_len(KS_MSG_BEACON, &a) + KS_TAG_BYTES <= 250u);
}

int main(void)
{
    printf("test_wire\n");
    KT_RUN(test_beacon_roundtrip);
    KT_RUN(test_propose_roundtrip_variable_length);
    KT_RUN(test_report_decide_roundtrip);
    KT_RUN(test_tamper_and_wrong_key_rejected);
    KT_RUN(test_malformed_rejected);
    KT_RUN(test_all_frames_fit_espnow_v1);
    KT_DONE();
}

/*
 * test_log.c - log record layout and integrity.
 */
#include <math.h>
#include <string.h>

#include "ks/ks_bytes.h"
#include "ks/ks_log.h"
#include "ktest.h"

static void test_state_layout(void)
{
    ks_log_state_t s;
    memset(&s, 0, sizeof s);
    s.u_ms = 4.5f;
    s.power_w = 59.1f;
    s.offset_deg = -12.5f;
    s.t_stator_c = 61.3f;
    s.t_mag_c = NAN;
    s.ke_ratio = 0.9876f;
    s.status = 0x1234u;
    s.faults = 0x8001u;
    s.sector = 27;
    s.mode = 1;
    s.energy_wh = 678.0f;
    ks_logrec_t r;
    ks_log_state(&r, 14, 123456u, 789u, &s);
    KT_CHECK(r.b[0] == KS_LOG_STATE && r.b[1] == 1u);
    KT_CHECK(ks_get_u16(r.b + 2) == 14 && ks_get_u32(r.b + 4) == 123456u && ks_get_u32(r.b + 8) == 789u);
    KT_CHECK(ks_get_f32(r.b + 12) == 4.5f);
    KT_CHECK(ks_get_f32(r.b + 28) == 59.1f);
    KT_CHECK((int16_t)ks_get_u16(r.b + 40) == -1250);
    KT_CHECK((int16_t)ks_get_u16(r.b + 42) == 613);
    KT_CHECK((int16_t)ks_get_u16(r.b + 44) == INT16_MIN); /* NaN sentinel */
    KT_CHECK(ks_get_u16(r.b + 46) == 9876u);
    KT_CHECK(ks_get_u16(r.b + 50) == 0x1234u && ks_get_u16(r.b + 52) == 0x8001u);
    KT_CHECK(r.b[54] == 27 && r.b[55] == 1);
    KT_CHECK(ks_get_f32(r.b + 58) == 678.0f);
    KT_CHECK(ks_log_check(&r));
}

static void test_crc_detects_corruption(void)
{
    ks_log_event_t e;
    memset(&e, 0, sizeof e);
    e.code = KS_EV_MODE;
    e.a[0] = 2.0f;
    ks_logrec_t r;
    ks_log_event(&r, 1, 2u, 3u, &e);
    KT_CHECK(ks_log_check(&r));
    for (int i = 0; i < 62; ++i) {
        ks_logrec_t c = r;
        c.b[i] ^= 0x10u;
        KT_CHECK(!ks_log_check(&c));
    }
}

static void test_trial_layout(void)
{
    ks_log_trial_t t;
    memset(&t, 0, sizeof t);
    t.trial_id = 77;
    t.initiator = 3;
    t.event = KS_TEV_DECIDED;
    t.outcome = 1;
    t.base_deg = 6.0f;
    t.trial_deg = 12.0f;
    t.score_w = 1.25f;
    t.phase_mean[0] = NAN;
    t.phase_mean[3] = 0.5f;
    t.pool_mean_w = 0.75f;
    t.pool_z = -1.234f;
    t.pool_n = 3;
    t.n_reports = 2;
    ks_logrec_t r;
    ks_log_trial(&r, 3, 10u, 20u, &t);
    KT_CHECK(r.b[1] == 2);
    KT_CHECK(ks_get_u16(r.b + 12) == 77 && r.b[16] == KS_TEV_DECIDED && r.b[19] == 1);
    KT_CHECK((int16_t)ks_get_u16(r.b + 22) == 1200);
    KT_CHECK(ks_get_f32(r.b + 32) == 1.25f);
    KT_CHECK((int16_t)ks_get_u16(r.b + 44) == INT16_MIN);   /* NaN phase */
    KT_CHECK((int16_t)ks_get_u16(r.b + 50) == 25);          /* 0.5 W at 0.02 W */
    KT_CHECK(ks_get_f32(r.b + 52) == 0.75f);
    KT_CHECK((int16_t)ks_get_u16(r.b + 56) == -1234);
    KT_CHECK(r.b[58] == 3);
    KT_CHECK(r.b[60] == 2);
    KT_CHECK(ks_log_check(&r));
}

int main(void)
{
    printf("test_log\n");
    KT_RUN(test_state_layout);
    KT_RUN(test_crc_detects_corruption);
    KT_RUN(test_trial_layout);
    KT_DONE();
}

/*
 * test_sense.c - Hall edge timing, anemometer transfer function, vane, NTC.
 */
#include "ks/ks_sense.h"
#include "ktest.h"

static void test_rotor_constant_speed(void)
{
    ks_rotor_t r;
    ks_rotor_init(&r);
    /* 100 rpm with 8 edges per revolution: 75 ms between edges */
    uint32_t t = 1000u;
    for (int i = 0; i < 20; ++i) {
        ks_rotor_edge(&r, t);
        t += 75000u;
    }
    KT_NEAR(ks_rotor_rpm(&r, t - 75000u + 10u, 8.0f, 2000000u), 100.0, 0.05);
}

static void test_rotor_median_rejects_jitter_and_glitches(void)
{
    ks_rotor_t r;
    ks_rotor_init(&r);
    uint32_t t = 0u;
    const int jitter[] = {0, 900, -1200, 400, -300, 1500, -700, 200, 0, -900};
    for (int i = 0; i < 10; ++i) {
        ks_rotor_edge(&r, (uint32_t)((int32_t)t + jitter[i]));
        if (i == 5) ks_rotor_edge(&r, t + jitter[i] + 1000u); /* bounce 1 ms later */
        t += 75000u;
    }
    KT_CHECK(r.n_glitch == 1u);
    KT_NEAR(ks_rotor_rpm(&r, t - 75000u, 8.0f, 2000000u), 100.0, 2.5);
}

static void test_rotor_stop_and_wrap(void)
{
    ks_rotor_t r;
    ks_rotor_init(&r);
    uint32_t t = 0xFFFF0000u; /* edges straddle the 32-bit wrap */
    for (int i = 0; i < 6; ++i) {
        ks_rotor_edge(&r, t);
        t += 37500u; /* 200 rpm */
    }
    const uint32_t last = t - 37500u;
    KT_NEAR(ks_rotor_rpm(&r, last + 1000u, 8.0f, 1000000u), 200.0, 0.1);
    /* no edge for 150 ms: bounded by elapsed time -> 50 rpm */
    KT_NEAR(ks_rotor_rpm(&r, last + 150000u, 8.0f, 1000000u), 50.0, 0.1);
    KT_NEAR(ks_rotor_rpm(&r, last + 1000001u, 8.0f, 1000000u), 0.0, 0.0);
}

static void test_anemometer_transfer(void)
{
    const ks_anemo_cal_t c = {0.75f, 0.25f, 0.3f};
    KT_NEAR(ks_anemo_speed(&c, 4.0f), 3.25, 1e-6);
    KT_NEAR(ks_anemo_speed(&c, 0.2f), 0.0, 0.0);
}

static void test_vane(void)
{
    const ks_vane_cal_t c = {100u, 3900u, 10.0f, 0.0f};
    float d;
    KT_CHECK(ks_vane_dir(&c, 100u, &d));
    KT_NEAR(d, 5.0, 1e-4);
    KT_CHECK(ks_vane_dir(&c, 3900u, &d));
    KT_NEAR(d, 355.0, 1e-3);
    KT_CHECK(!ks_vane_dir(&c, 50u, &d)); /* dead band */
    const ks_vane_cal_t c2 = {100u, 3900u, 10.0f, 30.0f};
    KT_CHECK(ks_vane_dir(&c2, 3900u, &d));
    KT_NEAR(d, 25.0, 1e-3); /* wraps through north */
}

static void test_ntc_beta(void)
{
    const ks_ntc_t n = {10000.0f, 25.0f, 3950.0f, 10000.0f};
    KT_NEAR(ks_ntc_temp_c(&n, 0.5f), 25.0, 1e-3);
    KT_NEAR(ks_ntc_temp_c(&n, 0.26406f), 50.0, 0.02);
    KT_CHECK(ks_ntc_temp_c(&n, 0.0f) != ks_ntc_temp_c(&n, 0.0f)); /* NaN */
}

int main(void)
{
    printf("test_sense\n");
    KT_RUN(test_rotor_constant_speed);
    KT_RUN(test_rotor_median_rejects_jitter_and_glitches);
    KT_RUN(test_rotor_stop_and_wrap);
    KT_RUN(test_anemometer_transfer);
    KT_RUN(test_vane);
    KT_RUN(test_ntc_beta);
    KT_DONE();
}

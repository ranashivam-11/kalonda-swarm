/*
 * test_safety.c - supervisor escalation, dwell and dust/heat detectors.
 */
#include <math.h>
#include <string.h>

#include "ks/ks_safety.h"
#include "ktest.h"

static ks_safety_in_t nominal(void)
{
    ks_safety_in_t in;
    memset(&in, 0, sizeof in);
    in.rpm = 200.0f;
    in.u_ms = 5.0f;
    in.anemo_valid = true;
    in.dir_deg = 270.0f;
    in.vane_valid = true;
    in.v_bus = 26.0f;
    in.i_bus = 1.0f;
    in.t_stator_c = 40.0f;
    in.t_mcu_c = 45.0f;
    in.n_alive = 3;
    in.consensus_ok = true;
    return in;
}

static ks_params_t params(void)
{
    ks_params_t p;
    ks_params_default(&p);
    p.mode_dwell_ms = 10000u;
    return p;
}

static uint32_t run(ks_safety_t *s, const ks_params_t *p, ks_safety_in_t *in, uint32_t t, int secs)
{
    for (int i = 0; i < secs; ++i) {
        t += 1000u;
        in->dir_deg = 270.0f + 3.0f * sinf((float)t * 1e-3f); /* live vane */
        ks_safety_update(s, p, in, t);
    }
    return t;
}

static void test_starts_safe_then_normal(void)
{
    ks_params_t p = params();
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    KT_CHECK(s.mode == KS_MODE_SAFE);
    ks_safety_in_t in = nominal();
    uint32_t t = run(&s, &p, &in, 0u, 5);
    KT_CHECK(s.mode == KS_MODE_SAFE); /* dwell not elapsed */
    t = run(&s, &p, &in, t, 10);
    KT_CHECK(s.mode == KS_MODE_NORMAL);
}

static void test_overspeed_shutdown_and_dwell(void)
{
    ks_params_t p = params();
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    ks_safety_in_t in = nominal();
    uint32_t t = run(&s, &p, &in, 0u, 20);
    in.rpm = 500.0f;
    t = run(&s, &p, &in, t, 1);
    KT_CHECK(s.mode == KS_MODE_NORMAL); /* 1 s is not enough */
    t = run(&s, &p, &in, t, 2);
    KT_CHECK(s.mode == KS_MODE_SHUTDOWN && (s.faults & KS_F_OVERSPEED));
    in.rpm = 200.0f;
    t = run(&s, &p, &in, t, 5);
    KT_CHECK(s.mode == KS_MODE_SHUTDOWN); /* holds through the dwell */
    t = run(&s, &p, &in, t, 10);
    KT_CHECK(s.mode == KS_MODE_NORMAL);
}

static void test_magnet_heat_derates_after_thermal_lag(void)
{
    ks_params_t p = params();
    p.tau_th_s = 100.0f;
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    ks_safety_in_t in = nominal();
    uint32_t t = run(&s, &p, &in, 0u, 20);
    in.t_stator_c = 95.0f; /* below the 110 C stator limit */
    t = run(&s, &p, &in, t, 30);
    KT_CHECK(s.mode == KS_MODE_NORMAL); /* magnets still cool */
    t = run(&s, &p, &in, t, 200);
    KT_CHECK(s.mode == KS_MODE_DERATE && (s.faults & KS_F_MAGNET_HOT));
    KT_CHECK(ks_safety_magnet_temp(&s) > p.t_magnet_max_c);
}

static void test_isolation_goes_safe(void)
{
    ks_params_t p = params();
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    ks_safety_in_t in = nominal();
    uint32_t t = run(&s, &p, &in, 0u, 20);
    in.n_alive = 0;
    t = run(&s, &p, &in, t, 29);
    KT_CHECK(s.mode == KS_MODE_NORMAL);
    t = run(&s, &p, &in, t, 2);
    KT_CHECK(s.mode == KS_MODE_SAFE && (s.faults & KS_F_RADIO_ISOLATED));
}

static void test_dust_jammed_anemometer(void)
{
    ks_params_t p = params();
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    ks_safety_in_t in = nominal();
    uint32_t t = run(&s, &p, &in, 0u, 20);
    in.u_ms = 0.0f; /* cups stopped while the rotor turns */
    t = run(&s, &p, &in, t, 59);
    KT_CHECK(!(s.faults & KS_F_ANEMO_STUCK));
    t = run(&s, &p, &in, t, 2);
    KT_CHECK(s.faults & KS_F_ANEMO_STUCK);
    KT_CHECK(!ks_safety_anemo_usable(&s));
    in.u_ms = 5.0f;
    t = run(&s, &p, &in, t, 1);
    KT_CHECK(ks_safety_anemo_usable(&s));
}

static void test_startup_transient_does_not_latch_dust_fault(void)
{
    ks_params_t p = params();
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    ks_safety_in_t in = nominal();
    in.u_ms = 0.6f; /* first second: cups still spinning up */
    uint32_t t = run(&s, &p, &in, 0u, 1);
    in.u_ms = 5.0f;
    t = run(&s, &p, &in, t, 5);
    KT_CHECK(!(s.faults & KS_F_ANEMO_DRIFT));
}

static void test_frozen_vane_detected(void)
{
    ks_params_t p = params();
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    ks_safety_in_t in = nominal();
    uint32_t t = 0u;
    for (int i = 0; i < 1300; ++i) { /* > two 10-min windows, constant vane */
        t += 1000u;
        in.dir_deg = 270.0f;
        ks_safety_update(&s, &p, &in, t);
    }
    KT_CHECK(s.faults & KS_F_VANE_STUCK);
    KT_CHECK(!ks_safety_vane_usable(&s));
}

static void test_demagnetisation_flag(void)
{
    ks_params_t p = params();
    ks_safety_t s;
    ks_safety_init(&s, 0u);
    ks_safety_in_t in = nominal();
    /* 200 rpm = 20.94 rad/s; at 40 C expect ke = 1.30 * (1 - 0.0012*20) = 1.2688,
     * so a healthy bus reads 1.2688*20.94 - 0.8 - 1.4 = 24.37 V at 1 A. */
    in.v_bus = 24.37f;
    uint32_t t = run(&s, &p, &in, 0u, 1200);
    KT_NEAR(ks_safety_ke_ratio(&s), 1.0, 0.01);
    KT_CHECK(!(s.faults & KS_F_DEMAG_SUSPECT));
    in.v_bus = 0.88f * 24.37f; /* 10 % flux loss beyond the reversible model */
    t = run(&s, &p, &in, t, 3600);
    KT_CHECK(s.faults & KS_F_DEMAG_SUSPECT);
}

int main(void)
{
    printf("test_safety\n");
    KT_RUN(test_starts_safe_then_normal);
    KT_RUN(test_overspeed_shutdown_and_dwell);
    KT_RUN(test_magnet_heat_derates_after_thermal_lag);
    KT_RUN(test_isolation_goes_safe);
    KT_RUN(test_dust_jammed_anemometer);
    KT_RUN(test_startup_transient_does_not_latch_dust_fault);
    KT_RUN(test_frozen_vane_detected);
    KT_RUN(test_demagnetisation_flag);
    KT_DONE();
}

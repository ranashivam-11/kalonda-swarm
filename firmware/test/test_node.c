/*
 * test_node.c - single-node behaviour: shutdown policy and safe defaults.
 */
#include <math.h>
#include <string.h>

#include "ks/ks_node.h"
#include "ktest.h"

static ks_node_t node;
static ks_outputs_t out;

static void run(ks_node_t *n, uint32_t *t, uint32_t seconds, float u, float rpm, float t_stator)
{
    for (uint32_t k = 0; k < seconds * 10u; ++k) {
        *t += KS_STEP_MS;
        ks_sensors_t s;
        memset(&s, 0, sizeof s);
        s.t_ms = *t;
        s.rpm = rpm;
        s.v_bus = 48.0f;
        s.i_bus = 1.0f;
        s.u_ms = u;
        s.anemo_valid = true;
        s.dir_deg = 270.0f + 3.0f * sinf((float)*t * 1e-3f);
        s.vane_valid = true;
        s.yaw_deg = 270.0f;
        s.t_stator_c = t_stator;
        s.t_mcu_c = 40.0f;
        ks_node_step(n, &s, &out);
    }
}

static void init(uint32_t *t)
{
    ks_params_t p;
    ks_params_default(&p);
    p.node_id = 1;
    p.mode_dwell_ms = 1000u;
    p.tau_th_s = 10.0f;
    ks_node_init(&node, &p, 1u, 0u);
    *t = 0u;
}

static void test_isolated_node_never_steers(void)
{
    uint32_t t;
    init(&t);
    run(&node, &t, 120u, 5.0f, 200.0f, 40.0f);
    KT_CHECK(out.mode == KS_MODE_SAFE); /* no neighbours: radio isolated */
    KT_NEAR(out.offset_deg, 0.0, 1e-6);
    KT_CHECK(out.yaw_enable);
    KT_NEAR(ks_angdiff(out.yaw_cmd_deg, 270.0f), 0.0, 4.0); /* faces its own vane */
}

static void test_cutout_brakes_when_cool(void)
{
    uint32_t t;
    init(&t);
    run(&node, &t, 10u, 18.0f, 300.0f, 40.0f); /* 3-s gust above 16 m/s */
    KT_CHECK(out.mode == KS_MODE_SHUTDOWN);
    KT_CHECK(out.brake);
    KT_NEAR(out.offset_deg, 70.0, 1e-6); /* furled */
}

static void test_hot_rotor_is_furled_not_braked(void)
{
    uint32_t t;
    init(&t);
    run(&node, &t, 120u, 5.0f, 200.0f, 85.0f); /* heat the magnet estimate */
    run(&node, &t, 10u, 18.0f, 300.0f, 85.0f);
    KT_CHECK(out.mode == KS_MODE_SHUTDOWN);
    KT_CHECK(!out.brake);
    KT_NEAR(out.offset_deg, 70.0, 1e-6);
    /* ... but an overspeeding rotor is braked whatever the temperature */
    run(&node, &t, 5u, 18.0f, 600.0f, 85.0f);
    KT_CHECK(out.brake);
}

int main(void)
{
    printf("test_node\n");
    KT_RUN(test_isolated_node_never_steers);
    KT_RUN(test_cutout_brakes_when_cool);
    KT_RUN(test_hot_rotor_is_furled_not_braked);
    KT_DONE();
}

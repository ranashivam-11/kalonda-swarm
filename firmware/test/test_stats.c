/*
 * test_stats.c
 */
#include "ks/ks_stats.h"
#include "ktest.h"

static void test_welford_matches_two_pass(void)
{
    const float x[] = {3.1f, 2.9f, 3.4f, 2.2f, 5.0f, 4.1f, 3.3f, 2.8f};
    const int n = 8;
    ks_welford_t w;
    ks_welford_reset(&w);
    double s = 0.0;
    for (int i = 0; i < n; ++i) {
        ks_welford_add(&w, x[i]);
        s += x[i];
    }
    const double m = s / n;
    double ss = 0.0;
    for (int i = 0; i < n; ++i) ss += (x[i] - m) * (x[i] - m);
    KT_NEAR(w.mean, m, 1e-6);
    KT_NEAR(ks_welford_var(&w), ss / (n - 1), 1e-5);
}

static void test_welford_merge(void)
{
    ks_welford_t a, b, all;
    ks_welford_reset(&a);
    ks_welford_reset(&b);
    ks_welford_reset(&all);
    for (int i = 0; i < 10; ++i) {
        const float v = (float)(i * i) * 0.1f;
        if (i < 4) ks_welford_add(&a, v); else ks_welford_add(&b, v);
        ks_welford_add(&all, v);
    }
    const ks_welford_t m = ks_welford_merge(&a, &b);
    KT_CHECK(m.n == all.n);
    KT_NEAR(m.mean, all.mean, 1e-5);
    KT_NEAR(ks_welford_var(&m), ks_welford_var(&all), 1e-4);
    ks_welford_t z;
    ks_welford_reset(&z);
    const ks_welford_t m2 = ks_welford_merge(&z, &z);
    KT_CHECK(m2.n == 0);
}

static void test_angles(void)
{
    KT_NEAR(ks_wrap360(-10.0f), 350.0, 1e-4);
    KT_NEAR(ks_wrap360(720.5f), 0.5, 1e-4);
    KT_NEAR(ks_wrap180(190.0f), -170.0, 1e-4);
    KT_NEAR(ks_angdiff(5.0f, 355.0f), 10.0, 1e-4);
    KT_NEAR(ks_angdiff(355.0f, 5.0f), -10.0, 1e-4);
    const float w = ks_wrap360(-1e-7f);
    KT_CHECK(w >= 0.0f && w < 360.0f);
}

static void test_circular_mean_across_north(void)
{
    ks_circ_ewma_t c;
    ks_circ_reset(&c);
    for (int i = 0; i < 200; ++i) {
        ks_circ_update(&c, (i & 1) ? 350.0f : 10.0f, 1.0f, 5.0f);
    }
    const float v = ks_circ_value(&c);
    KT_CHECK(v < 3.0f || v > 357.0f); /* arithmetic mean would say 180 */
    KT_CHECK(ks_circ_resultant(&c) > 0.9f);
}

static void test_median(void)
{
    const float a[] = {5, 1, 4, 2, 3};
    KT_NEAR(ks_median(a, 5), 3.0, 0);
    const float b[] = {4, 1, 3, 2};
    KT_NEAR(ks_median(b, 4), 2.5, 0);
    KT_NEAR(ks_median(a, 0), 0.0, 0);
}

static void test_ewma_step_response(void)
{
    ks_ewma_t e;
    ks_ewma_reset(&e);
    ks_ewma_update(&e, 0.0f, 1.0f, 10.0f);
    for (int i = 0; i < 10; ++i) ks_ewma_update(&e, 1.0f, 1.0f, 10.0f);
    KT_NEAR(e.y, 1.0 - exp(-1.0), 1e-4); /* one time constant */
}

int main(void)
{
    printf("test_stats\n");
    KT_RUN(test_welford_matches_two_pass);
    KT_RUN(test_welford_merge);
    KT_RUN(test_angles);
    KT_RUN(test_circular_mean_across_north);
    KT_RUN(test_median);
    KT_RUN(test_ewma_step_response);
    KT_DONE();
}

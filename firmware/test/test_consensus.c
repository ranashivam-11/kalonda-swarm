/*
 * test_consensus.c - wind-direction consensus with biased and jammed vanes.
 */
#include "ks/ks_consensus.h"
#include "ks/ks_params.h"
#include "ktest.h"

enum { N = 6 };

static void run(ks_cons_t *k, const ks_params_t *p, const float *truth_bias, const bool *valid,
                float wind, int steps)
{
    for (int t = 0; t < steps; ++t) {
        float est[N];
        for (int i = 0; i < N; ++i) est[i] = k[i].init ? ks_cons_dir(&k[i]) : wind + truth_bias[i];
        for (int i = 0; i < N; ++i) {
            float nb[N];
            int m = 0;
            for (int j = 0; j < N; ++j) {
                if (j != i) nb[m++] = est[j];
            }
            ks_cons_update(&k[i], p, ks_wrap360(wind + truth_bias[i]), valid[i], nb, m, 1.0f);
        }
    }
}

static void test_agreement_with_biased_vanes(void)
{
    ks_params_t p;
    ks_params_default(&p);
    ks_cons_t k[N];
    for (int i = 0; i < N; ++i) ks_cons_init(&k[i]);
    const float bias[N] = {-6, -3, 0, 2, 4, 3};      /* mean 0 */
    const bool valid[N] = {true, true, true, true, true, true};
    run(k, &p, bias, valid, 268.0f, 400);
    for (int i = 0; i < N; ++i) {
        KT_NEAR(ks_angdiff(ks_cons_dir(&k[i]), 268.0f), 0.0, 1.0);
        KT_CHECK(k[i].ok);
        KT_CHECK(k[i].sector == 27);
    }
}

static void test_jammed_vane_is_outvoted(void)
{
    ks_params_t p;
    ks_params_default(&p);
    ks_cons_t k[N];
    for (int i = 0; i < N; ++i) ks_cons_init(&k[i]);
    const float bias[N] = {0, 0, 0, 0, 0, 120};       /* node 5 jammed at +120 */
    const bool valid[N] = {true, true, true, true, true, true};
    run(k, &p, bias, valid, 90.0f, 600);
    for (int i = 0; i < N - 1; ++i) {
        KT_NEAR(ks_angdiff(ks_cons_dir(&k[i]), 90.0f), 0.0, 2.0);
    }
    /* once the supervisor flags the vane, node 5 rejoins the array */
    const bool valid2[N] = {true, true, true, true, true, false};
    run(k, &p, bias, valid2, 90.0f, 600);
    KT_NEAR(ks_angdiff(ks_cons_dir(&k[5]), 90.0f), 0.0, 2.0);
}

static void test_wraparound_at_north(void)
{
    ks_params_t p;
    ks_params_default(&p);
    ks_cons_t k[N];
    for (int i = 0; i < N; ++i) ks_cons_init(&k[i]);
    const float bias[N] = {-4, -2, -1, 1, 2, 4};
    const bool valid[N] = {true, true, true, true, true, true};
    run(k, &p, bias, valid, 0.5f, 300);
    for (int i = 0; i < N; ++i) {
        KT_NEAR(ks_angdiff(ks_cons_dir(&k[i]), 0.5f), 0.0, 1.0);
        KT_CHECK(k[i].sector == 0);
    }
}

static void test_sector_hysteresis(void)
{
    ks_params_t p;
    ks_params_default(&p);
    ks_cons_t k;
    ks_cons_init(&k);
    for (int t = 0; t < 200; ++t) ks_cons_update(&k, &p, 274.0f, true, NULL, 0, 1.0f);
    KT_CHECK(k.sector == 27);
    /* 276 is past the 275 boundary but inside the 2-degree hysteresis */
    for (int t = 0; t < 400; ++t) ks_cons_update(&k, &p, 276.0f, true, NULL, 0, 1.0f);
    KT_CHECK(k.sector == 27);
    for (int t = 0; t < 400; ++t) ks_cons_update(&k, &p, 278.5f, true, NULL, 0, 1.0f);
    KT_CHECK(k.sector == 28);
}

int main(void)
{
    printf("test_consensus\n");
    KT_RUN(test_agreement_with_biased_vanes);
    KT_RUN(test_jammed_vane_is_outvoted);
    KT_RUN(test_wraparound_at_north);
    KT_RUN(test_sector_hysteresis);
    KT_DONE();
}

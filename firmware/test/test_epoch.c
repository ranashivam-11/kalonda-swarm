/*
 * test_epoch.c - bounded max-consensus phase synchronisation.
 */
#include "ks/ks_epoch.h"
#include "ktest.h"

static void test_rollover(void)
{
    ks_epoch_t e;
    ks_epoch_init(&e, 1000u, 500u);
    KT_CHECK(ks_epoch_advance(&e, 1499u) == 0u);
    KT_CHECK(ks_epoch_advance(&e, 1500u) == 1u && e.epoch == 1u);
    KT_CHECK(ks_epoch_advance(&e, 5600u) == 4u && e.epoch == 5u);
    KT_CHECK(ks_epoch_ms_into(&e, 5600u) == 100u);
}

static void test_rollover_across_uint32_wrap(void)
{
    ks_epoch_t e;
    ks_epoch_init(&e, 1000u, 0xFFFFFF00u);
    KT_CHECK(ks_epoch_advance(&e, 0x00000300u) == 1u); /* 0x400 ms later */
    KT_CHECK(ks_epoch_ms_into(&e, 0x00000300u) == 0x400u - 1000u);
}

static void test_fresh_node_adopts_network_time(void)
{
    ks_epoch_t e;
    ks_epoch_init(&e, 60000u, 10u);
    /* neighbour is deep into epoch 5000 */
    KT_CHECK(ks_epoch_observe(&e, 4, 5000u, 1234u, 20u, 3u) == 1);
    KT_CHECK(e.epoch == 5000u && e.synced);
    KT_NEAR(ks_epoch_ms_into(&e, 20u), 1237.0, 0.5);
}

static void test_stale_timestamp_does_not_explode(void)
{
    ks_epoch_t e;
    ks_epoch_init(&e, 1000u, 5000u);
    KT_CHECK(ks_epoch_advance(&e, 4990u) == 0u); /* 10 ms before start */
    KT_CHECK(ks_epoch_ms_into(&e, 4990u) == 0u);
    KT_CHECK(e.epoch == 0u);
}

static void test_never_moves_backwards(void)
{
    ks_epoch_t e;
    ks_epoch_init(&e, 1000u, 0u);
    ks_epoch_observe(&e, 1, 100u, 500u, 0u, 0u);
    const uint64_t g = ks_epoch_global(&e, 10u);
    KT_CHECK(ks_epoch_observe(&e, 2, 90u, 0u, 10u, 0u) == 0);
    KT_CHECK(ks_epoch_global(&e, 10u) == g);
}

static void test_small_forward_step_accepted(void)
{
    ks_epoch_t e;
    ks_epoch_init(&e, 1000u, 0u);
    ks_epoch_observe(&e, 1, 100u, 0u, 0u, 0u); /* synced at 100.000 */
    /* 200 ms ahead (< len/2): accepted from a single source */
    KT_CHECK(ks_epoch_observe(&e, 2, 100u, 300u, 100u, 0u) == 1);
    KT_NEAR((double)ks_epoch_global(&e, 100u), 100300.0, 0.5);
}

static void test_large_jump_needs_two_sources(void)
{
    ks_epoch_t e;
    ks_epoch_init(&e, 1000u, 0u);
    ks_epoch_observe(&e, 1, 100u, 0u, 0u, 0u);
    /* one node claims to be an hour ahead: ignored */
    KT_CHECK(ks_epoch_observe(&e, 9, 3700u, 0u, 10u, 0u) == 0);
    KT_CHECK(e.epoch == 100u && e.n_rejected == 1u);
    /* the same node repeating itself does not confirm */
    KT_CHECK(ks_epoch_observe(&e, 9, 3700u, 500u, 510u, 0u) == 0);
    KT_CHECK(e.epoch == 100u);
    /* a second, independent node agrees: accepted */
    KT_CHECK(ks_epoch_observe(&e, 5, 3700u, 600u, 610u, 0u) == 1);
    KT_CHECK(e.epoch == 3700u);
}

static void test_network_converges_to_fastest_clock(void)
{
    /* Five nodes on a line, only neighbours hear each other. Node 0's
     * crystal runs 100 ppm fast. Each hop may lag its upstream neighbour
     * by up to the 20 ms sync tolerance, so the far end is within 4 * 21 ms. */
    enum { N = 5 };
    ks_epoch_t e[N];
    for (int i = 0; i < N; ++i) ks_epoch_init(&e[i], 10000u, (uint32_t)(i * 777));
    uint32_t t = 4000u; /* after every node's boot instant */
    for (int round = 0; round < 600; ++round) {
        t += 1000u;
        uint32_t local[N];
        for (int i = 0; i < N; ++i) {
            local[i] = (i == 0) ? t + t / 10000u : t; /* +100 ppm on node 0 */
            ks_epoch_advance(&e[i], local[i]);
        }
        for (int i = 0; i < N; ++i) {
            for (int d = -1; d <= 1; d += 2) {
                const int j = i + d;
                if (j < 0 || j >= N) continue;
                const uint32_t into = ks_epoch_ms_into(&e[i], local[i]);
                ks_epoch_observe(&e[j], (uint16_t)i, e[i].epoch, (uint16_t)into, local[j], 0u);
            }
        }
    }
    const double g0 = (double)ks_epoch_global(&e[0], t + t / 10000u);
    for (int i = 1; i < N; ++i) {
        KT_NEAR((double)ks_epoch_global(&e[i], t), g0, 21.0 * i);
    }
}

int main(void)
{
    printf("test_epoch\n");
    KT_RUN(test_rollover);
    KT_RUN(test_rollover_across_uint32_wrap);
    KT_RUN(test_fresh_node_adopts_network_time);
    KT_RUN(test_stale_timestamp_does_not_explode);
    KT_RUN(test_never_moves_backwards);
    KT_RUN(test_small_forward_step_accepted);
    KT_RUN(test_large_jump_needs_two_sources);
    KT_RUN(test_network_converges_to_fastest_clock);
    KT_DONE();
}

/*
 * ks_epoch.c
 */
#include "ks/ks_epoch.h"

#define SYNC_TOL_MS 20u      /* ignore sub-20 ms differences (radio jitter) */
#define CONFIRM_TOL_MS 1000  /* two sources agree on a big jump within 1 s  */
#define CONFIRM_WINDOW_MS 120000u

void ks_epoch_init(ks_epoch_t *e, uint32_t len_ms, uint32_t now_ms)
{
    e->epoch = 0u;
    e->start_ms = now_ms;
    e->len_ms = len_ms ? len_ms : 1u;
    e->synced = false;
    e->n_adjust = 0u;
    e->n_rejected = 0u;
    e->pend_valid = false;
    e->pend_src = KS_NODE_NONE;
    e->pend_g = 0u;
    e->pend_at_ms = 0u;
}

/* Signed elapsed time: a timestamp slightly older than start_ms (a frame
 * stamped before an adjustment) must read as "just before", not as 49 days. */
static int32_t elapsed(const ks_epoch_t *e, uint32_t now_ms) { return (int32_t)(now_ms - e->start_ms); }

uint32_t ks_epoch_advance(ks_epoch_t *e, uint32_t now_ms)
{
    uint32_t n = 0u;
    while (elapsed(e, now_ms) >= (int32_t)e->len_ms) {
        e->start_ms += e->len_ms;
        e->epoch += 1u;
        n += 1u;
    }
    return n;
}

uint32_t ks_epoch_ms_into(const ks_epoch_t *e, uint32_t now_ms)
{
    const int32_t d = elapsed(e, now_ms);
    return d > 0 ? (uint32_t)d : 0u;
}

uint64_t ks_epoch_global(const ks_epoch_t *e, uint32_t now_ms)
{
    return (uint64_t)e->epoch * (uint64_t)e->len_ms + (uint64_t)ks_epoch_ms_into(e, now_ms);
}

static void adopt(ks_epoch_t *e, uint64_t g, uint32_t at_ms)
{
    e->epoch = (uint32_t)(g / e->len_ms);
    const uint32_t into = (uint32_t)(g % e->len_ms);
    e->start_ms = at_ms - into;
    e->synced = true;
    e->n_adjust += 1u;
    e->pend_valid = false;
}

int ks_epoch_observe(ks_epoch_t *e, uint16_t src, uint32_t n_epoch, uint16_t n_ms_into,
                     uint32_t rx_ms, uint32_t latency_ms)
{
    /* Sender's phase at the moment we received the frame. */
    const uint64_t g_n = (uint64_t)n_epoch * e->len_ms + n_ms_into + latency_ms;
    const int64_t gi_s = (int64_t)e->epoch * e->len_ms + elapsed(e, rx_ms);
    const uint64_t g_i = gi_s > 0 ? (uint64_t)gi_s : 0u;

    if (g_n <= g_i + SYNC_TOL_MS) {
        /* Level or ahead: the neighbour will converge to us, so we already
         * belong to a synchronised array. */
        e->synced = true;
        return 0;
    }

    const uint64_t d = g_n - g_i;
    if (!e->synced || d <= e->len_ms / 2u) {
        adopt(e, g_n, rx_ms);
        return 1;
    }

    /* Large forward jump from an already-synchronised node: require a
     * second source that agrees within CONFIRM_TOL_MS. */
    if (e->pend_valid && e->pend_src != src &&
        (uint32_t)(rx_ms - e->pend_at_ms) <= CONFIRM_WINDOW_MS) {
        const uint64_t pend_now = e->pend_g + (uint32_t)(rx_ms - e->pend_at_ms);
        const int64_t gap = (int64_t)(g_n - pend_now);
        if (gap < CONFIRM_TOL_MS && gap > -CONFIRM_TOL_MS) {
            adopt(e, g_n > pend_now ? g_n : pend_now, rx_ms);
            return 1;
        }
    }
    e->pend_valid = true;
    e->pend_src = src;
    e->pend_g = g_n;
    e->pend_at_ms = rx_ms;
    e->n_rejected += 1u;
    return 0;
}

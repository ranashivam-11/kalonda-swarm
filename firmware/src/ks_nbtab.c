/*
 * ks_nbtab.c
 */
#include "ks/ks_nbtab.h"

#include <string.h>

void ks_nbtab_init(ks_nbtab_t *t) { memset(t, 0, sizeof(*t)); }

ks_nb_t *ks_nbtab_find(ks_nbtab_t *t, uint16_t id)
{
    for (int i = 0; i < KS_MAX_NODES; ++i) {
        if (t->e[i].used && t->e[i].id == id) {
            return &t->e[i];
        }
    }
    return NULL;
}

const ks_nb_t *ks_nbtab_cfind(const ks_nbtab_t *t, uint16_t id)
{
    for (int i = 0; i < KS_MAX_NODES; ++i) {
        if (t->e[i].used && t->e[i].id == id) {
            return &t->e[i];
        }
    }
    return NULL;
}

static int is_fresh(const ks_nb_t *n, const ks_hdr_t *h)
{
    const int16_t db = (int16_t)(uint16_t)(h->boot - n->boot);
    if (db != 0) {
        return db > 0;
    }
    if (h->epoch != n->epoch) {
        return h->epoch > n->epoch;
    }
    return (int16_t)(uint16_t)(h->seq - n->seq) > 0;
}

ks_nb_t *ks_nbtab_accept(ks_nbtab_t *t, const ks_hdr_t *h, uint32_t now_ms, int8_t rssi)
{
    ks_nb_t *n = ks_nbtab_find(t, h->src);
    if (n == NULL) {
        for (int i = 0; i < KS_MAX_NODES; ++i) {
            if (!t->e[i].used) {
                n = &t->e[i];
                memset(n, 0, sizeof(*n));
                n->used = true;
                n->id = h->src;
                n->rssi_ewma = (float)rssi;
                break;
            }
        }
        if (n == NULL) {
            return NULL;
        }
    } else if (!is_fresh(n, h)) {
        n->rx_stale += 1u;
        return NULL;
    }
    n->boot = h->boot;
    n->epoch = h->epoch;
    n->seq = h->seq;
    n->last_ms = now_ms;
    n->rx_ok += 1u;
    n->rssi_ewma += 0.1f * ((float)rssi - n->rssi_ewma);
    return n;
}

bool ks_nb_alive(const ks_nb_t *n, uint32_t now_ms, uint32_t timeout_ms)
{
    return n->used && n->has_beacon && (uint32_t)(now_ms - n->last_beacon_ms) <= timeout_ms;
}

int ks_nbtab_count_alive(const ks_nbtab_t *t, uint32_t now_ms, uint32_t timeout_ms)
{
    int c = 0;
    for (int i = 0; i < KS_MAX_NODES; ++i) {
        if (ks_nb_alive(&t->e[i], now_ms, timeout_ms)) {
            ++c;
        }
    }
    return c;
}

/*
 * ks_epoch.h - leaderless phase synchronisation by bounded max-consensus.
 *
 * Every node keeps (epoch, start_ms) on its own clock. A beacon carries the
 * sender's epoch and the milliseconds elapsed in it; a receiver that finds
 * itself behind jumps forward to the sender's phase. Time only ever moves
 * forward, so the array converges to its fastest clock within one diameter
 * of beacons. A single node cannot drag the array more than half an epoch
 * forward: larger jumps need a second, independent node to confirm them
 * (a Byzantine-lite guard against one corrupted clock).
 */
#ifndef KS_EPOCH_H
#define KS_EPOCH_H

#include "ks_types.h"

typedef struct {
    uint32_t epoch;
    uint32_t start_ms;   /* local time at which `epoch` began         */
    uint32_t len_ms;
    bool synced;         /* adopted or confirmed a neighbour at least once */
    uint32_t n_adjust;   /* forward adjustments applied                */
    uint32_t n_rejected; /* unconfirmed large jumps ignored            */
    /* pending large jump awaiting confirmation */
    bool pend_valid;
    uint16_t pend_src;
    uint64_t pend_g;     /* proposed global ms at pend_at_ms            */
    uint32_t pend_at_ms;
} ks_epoch_t;

void ks_epoch_init(ks_epoch_t *e, uint32_t len_ms, uint32_t now_ms);

/* Roll the epoch counter forward to `now`. Returns epochs advanced. */
uint32_t ks_epoch_advance(ks_epoch_t *e, uint32_t now_ms);

uint32_t ks_epoch_ms_into(const ks_epoch_t *e, uint32_t now_ms);

/* Global phase in ms: epoch * len + ms_into. */
uint64_t ks_epoch_global(const ks_epoch_t *e, uint32_t now_ms);

/* Process a neighbour's (epoch, ms_into) observed at local time rx_ms.
 * Returns 1 if the local phase was moved forward. */
int ks_epoch_observe(ks_epoch_t *e, uint16_t src, uint32_t n_epoch, uint16_t n_ms_into,
                     uint32_t rx_ms, uint32_t latency_ms);

#endif /* KS_EPOCH_H */

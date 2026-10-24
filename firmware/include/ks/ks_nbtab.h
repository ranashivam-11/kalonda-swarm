/*
 * ks_nbtab.h - neighbour table: last authenticated state of every peer,
 * freshness (replay) state and link quality.
 */
#ifndef KS_NBTAB_H
#define KS_NBTAB_H

#include "ks_types.h"
#include "ks_wire.h"

typedef struct {
    bool used;
    uint16_t id;
    /* freshness: frames are ordered by (boot, epoch, seq) */
    uint16_t boot;
    uint32_t epoch;
    uint16_t seq;
    uint32_t last_ms;        /* local time of last accepted frame */
    uint32_t last_beacon_ms; /* local time of last beacon         */
    bool has_beacon;
    ks_beacon_t b;           /* last beacon                        */
    float rssi_ewma;
    uint32_t rx_ok;
    uint32_t rx_stale;
} ks_nb_t;

typedef struct {
    ks_nb_t e[KS_MAX_NODES];
} ks_nbtab_t;

void ks_nbtab_init(ks_nbtab_t *t);
ks_nb_t *ks_nbtab_find(ks_nbtab_t *t, uint16_t id);
const ks_nb_t *ks_nbtab_cfind(const ks_nbtab_t *t, uint16_t id);

/* Freshness check and update. Returns the entry if the frame is fresh,
 * NULL if it is a replay/duplicate or the table is full. */
ks_nb_t *ks_nbtab_accept(ks_nbtab_t *t, const ks_hdr_t *h, uint32_t now_ms, int8_t rssi);

bool ks_nb_alive(const ks_nb_t *n, uint32_t now_ms, uint32_t timeout_ms);
int ks_nbtab_count_alive(const ks_nbtab_t *t, uint32_t now_ms, uint32_t timeout_ms);

#endif /* KS_NBTAB_H */

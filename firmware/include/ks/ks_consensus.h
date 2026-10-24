/*
 * ks_consensus.h - distributed wind-direction estimate.
 *
 * Every vane is biased (installation north error, dust in the bearing), and
 * the nodes must agree on one wind direction to agree on who is upstream.
 * Each node runs a consensus + innovations filter (Kar & Moura, IEEE Signal
 * Processing Magazine 30(3), 2013) on the unit circle:
 *
 *   c_i <- normalise( c_i + a * mean_j (c_j - c_i) + b * (z_i - c_i) )
 *
 * where c_j are neighbours' broadcast estimates and z_i is the node's own
 * low-passed vane. Neighbours more than 45 degrees from the robust centre
 * are ignored, so one jammed vane cannot drag the array.
 */
#ifndef KS_CONSENSUS_H
#define KS_CONSENSUS_H

#include "ks_params.h"
#include "ks_stats.h"

typedef struct {
    bool init;
    float c, s;               /* consensus unit vector                    */
    ks_circ_ewma_t own;       /* low-passed own vane                       */
    float spread_deg;         /* max |difference| to included neighbours   */
    int n_used;               /* neighbours used in the last update        */
    int n_rejected;           /* neighbours rejected as outliers           */
    bool ok;
    bool sector_valid;
    uint8_t sector;
} ks_cons_t;

void ks_cons_init(ks_cons_t *k);

void ks_cons_update(ks_cons_t *k, const ks_params_t *p, float own_dir_deg, bool own_valid,
                    const float *nb_dir_deg, int n_nb, float dt_s);

float ks_cons_dir(const ks_cons_t *k);

/* Sector index (0..KS_SECTORS-1) for a direction, without hysteresis. */
uint8_t ks_sector_of(float dir_deg);

#endif /* KS_CONSENSUS_H */

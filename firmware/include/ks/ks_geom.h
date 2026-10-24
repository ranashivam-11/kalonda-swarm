/*
 * ks_geom.h - who is downstream of whom, and who may experiment together.
 *
 * Deliberately crude: a widened top-hat wake cone (Jensen/Katic style) is
 * used only to choose *participants* (whose measured power counts toward a
 * trial's score) and to colour the conflict graph. The cooperative benefit
 * itself is never predicted on the node; it is measured.
 */
#ifndef KS_GEOM_H
#define KS_GEOM_H

#include "ks_types.h"

typedef struct {
    uint16_t id;
    float x, y;
} ks_site_t;

typedef struct {
    float rotor_d;
    float k;          /* cone expansion per unit streamwise distance */
    float margin_d;   /* extra lateral margin [D] for steered wakes  */
    float max_d;      /* farthest streamwise coupling considered [D] */
} ks_cone_t;

typedef struct {
    bool valid;
    uint8_t sector;
    int n;                          /* nodes in the graph (incl. self)   */
    ks_site_t site[KS_MAX_NODES];   /* sorted by id                       */
    uint8_t color[KS_MAX_NODES];
    uint8_t n_colors;
    int self;                       /* index of self in site[]            */
    uint8_t n_down;                 /* self's downstream participants      */
    uint16_t down[KS_MAX_PART];     /* ordered by streamwise distance      */
    uint8_t n_up;                   /* nodes whose wake reaches self       */
} ks_geom_t;

/* Wind blows FROM dir_from_deg (meteorological). Returns the streamwise
 * distance from a to b [m] if b lies inside a's widened wake cone, else -1. */
float ks_geom_in_wake(const ks_cone_t *c, float ax, float ay, float bx, float by,
                      float dir_from_deg);

/* Same test, true if b is inside the cone for any direction across the
 * sector (centre and both edges) - robust to small consensus differences. */
float ks_geom_in_wake_sector(const ks_cone_t *c, float ax, float ay, float bx, float by,
                             uint8_t sector);

float ks_sector_center_deg(uint8_t sector);

/* Build the graph for `sector` from n sites (any order, self included). */
void ks_geom_build(ks_geom_t *g, const ks_cone_t *c, const ks_site_t *sites, int n,
                   uint16_t self_id, uint8_t sector);

/* Colour of a node (0xFF if unknown). */
uint8_t ks_geom_color_of(const ks_geom_t *g, uint16_t id);

#endif /* KS_GEOM_H */

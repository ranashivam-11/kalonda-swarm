/*
 * ks_geom.c
 */
#include "ks/ks_geom.h"

#include <math.h>
#include <string.h>

float ks_sector_center_deg(uint8_t sector) { return (float)sector * KS_SECTOR_DEG; }

float ks_geom_in_wake(const ks_cone_t *c, float ax, float ay, float bx, float by,
                      float dir_from_deg)
{
    /* Unit vector the wind travels along (towards dir_from + 180). */
    const float r = dir_from_deg * KS_DEG2RAD;
    const float wx = -sinf(r), wy = -cosf(r);
    const float dx = bx - ax, dy = by - ay;
    const float s = dx * wx + dy * wy;           /* streamwise, + downstream */
    const float n = fabsf(-dx * wy + dy * wx);   /* crosswind magnitude      */
    const float D = c->rotor_d;
    if (s < 0.5f * D || s > c->max_d * D) {
        return -1.0f;
    }
    const float half_width = 0.5f * D + c->k * s + c->margin_d * D;
    /* b's rotor (radius D/2) must overlap the widened wake disc. */
    return (n < half_width + 0.5f * D) ? s : -1.0f;
}

float ks_geom_in_wake_sector(const ks_cone_t *c, float ax, float ay, float bx, float by,
                             uint8_t sector)
{
    const float ctr = ks_sector_center_deg(sector);
    const float half = 0.5f * KS_SECTOR_DEG;
    float best = -1.0f;
    const float dirs[3] = {ctr - half, ctr, ctr + half};
    for (int i = 0; i < 3; ++i) {
        const float s = ks_geom_in_wake(c, ax, ay, bx, by, dirs[i]);
        if (s > 0.0f && (best < 0.0f || s < best)) {
            best = s;
        }
    }
    return best;
}

/* Influence set of node i: itself plus its downstream nodes. */
typedef struct {
    int n;
    int idx[KS_MAX_PART + 1];
} infl_t;

static void build_influence(const ks_geom_t *g, const ks_cone_t *c, int i, infl_t *out)
{
    float dist[KS_MAX_PART];
    out->n = 1;
    out->idx[0] = i;
    int nd = 0;
    for (int j = 0; j < g->n; ++j) {
        if (j == i) continue;
        const float s = ks_geom_in_wake_sector(c, g->site[i].x, g->site[i].y, g->site[j].x,
                                               g->site[j].y, g->sector);
        if (s <= 0.0f) continue;
        /* keep the KS_MAX_PART nearest, ordered by distance */
        int pos = nd;
        if (nd == KS_MAX_PART) {
            if (s >= dist[nd - 1]) continue;
            pos = nd - 1;
        } else {
            ++nd;
        }
        while (pos > 0 && dist[pos - 1] > s) {
            dist[pos] = dist[pos - 1];
            out->idx[1 + pos] = out->idx[pos];
            --pos;
        }
        dist[pos] = s;
        out->idx[1 + pos] = j;
    }
    out->n = 1 + nd;
}

static bool intersects(const infl_t *a, const infl_t *b)
{
    for (int i = 0; i < a->n; ++i) {
        for (int j = 0; j < b->n; ++j) {
            if (a->idx[i] == b->idx[j]) return true;
        }
    }
    return false;
}

void ks_geom_build(ks_geom_t *g, const ks_cone_t *c, const ks_site_t *sites, int n,
                   uint16_t self_id, uint8_t sector)
{
    memset(g, 0, sizeof(*g));
    if (n > KS_MAX_NODES) n = KS_MAX_NODES;
    g->sector = sector;
    g->self = -1;

    /* insertion sort by id, dropping duplicates */
    int m = 0;
    for (int i = 0; i < n; ++i) {
        int pos = m;
        bool dup = false;
        for (int k = 0; k < m; ++k) {
            if (g->site[k].id == sites[i].id) { dup = true; break; }
        }
        if (dup) continue;
        while (pos > 0 && g->site[pos - 1].id > sites[i].id) {
            g->site[pos] = g->site[pos - 1];
            --pos;
        }
        g->site[pos] = sites[i];
        ++m;
    }
    g->n = m;

    static infl_t infl[KS_MAX_NODES]; /* scratch; ks_geom_build is not reentrant */
    for (int i = 0; i < m; ++i) {
        build_influence(g, c, i, &infl[i]);
        if (g->site[i].id == self_id) g->self = i;
    }

    /* Greedy colouring in id order: identical on every node that sees the
     * same sites and sector, so no colouring messages are needed. */
    g->n_colors = 0;
    for (int i = 0; i < m; ++i) {
        uint64_t used = 0;
        for (int k = 0; k < i; ++k) {
            if (intersects(&infl[i], &infl[k]) && g->color[k] < 64u) {
                used |= (uint64_t)1u << g->color[k];
            }
        }
        uint8_t col = 0;
        while (col < 63u && (used & ((uint64_t)1u << col))) ++col;
        g->color[i] = col;
        if (col + 1u > g->n_colors) g->n_colors = (uint8_t)(col + 1u);
    }

    if (g->self >= 0) {
        const infl_t *me = &infl[g->self];
        g->n_down = (uint8_t)(me->n - 1);
        for (int k = 1; k < me->n; ++k) {
            g->down[k - 1] = g->site[me->idx[k]].id;
        }
        int up = 0;
        for (int i = 0; i < m; ++i) {
            if (i == g->self) continue;
            for (int k = 1; k < infl[i].n; ++k) {
                if (infl[i].idx[k] == g->self) { ++up; break; }
            }
        }
        g->n_up = (uint8_t)up;
    }
    g->valid = (g->self >= 0);
}

uint8_t ks_geom_color_of(const ks_geom_t *g, uint16_t id)
{
    for (int i = 0; i < g->n; ++i) {
        if (g->site[i].id == id) return g->color[i];
    }
    return 0xFFu;
}

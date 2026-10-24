/*
 * test_geom.c - wake cone, downstream sets and conflict-graph colouring on a
 * 3 x 4 grid (5D streamwise, 3D spanwise, D = 2 m).
 */
#include <string.h>

#include "ks/ks_consensus.h"
#include "ks/ks_geom.h"
#include "ktest.h"

static const ks_cone_t CONE = {2.0f, 0.10f, 0.5f, 15.0f};

static int grid(ks_site_t *s)
{
    int n = 0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            s[n].id = (uint16_t)(1 + r * 4 + c);
            s[n].x = 10.0f * (float)r;
            s[n].y = 6.0f * (float)c;
            ++n;
        }
    }
    return n;
}

static bool has(const ks_geom_t *g, uint16_t id)
{
    for (int i = 0; i < g->n_down; ++i) {
        if (g->down[i] == id) return true;
    }
    return false;
}

static void test_west_wind_rows(void)
{
    ks_site_t s[12];
    const int n = grid(s);
    ks_geom_t g;
    const uint8_t west = ks_sector_of(270.0f);
    ks_geom_build(&g, &CONE, s, n, 1, west);
    KT_CHECK(g.valid);
    KT_CHECK(has(&g, 5) && has(&g, 9));            /* straight downstream */
    KT_CHECK(!has(&g, 2) && !has(&g, 3));          /* same row            */
    KT_CHECK(g.down[0] == 5);                      /* nearest first       */
    KT_CHECK(g.n_up == 0);                         /* front row           */
    ks_geom_build(&g, &CONE, s, n, 9, west);
    KT_CHECK(g.n_down == 0 && g.n_up >= 2);        /* back row            */
}

static void test_east_wind_reverses(void)
{
    ks_site_t s[12];
    const int n = grid(s);
    ks_geom_t g;
    ks_geom_build(&g, &CONE, s, n, 9, ks_sector_of(90.0f));
    KT_CHECK(has(&g, 5) && has(&g, 1));
    ks_geom_build(&g, &CONE, s, n, 1, ks_sector_of(90.0f));
    KT_CHECK(g.n_down == 0);
}

static void test_north_wind_columns(void)
{
    ks_site_t s[12];
    const int n = grid(s);
    ks_geom_t g;
    ks_geom_build(&g, &CONE, s, n, 4, ks_sector_of(0.0f)); /* (0,18) */
    KT_CHECK(has(&g, 3) && has(&g, 2));
    KT_CHECK(!has(&g, 8));
}

static bool influences(const ks_site_t *a, const ks_site_t *b, uint8_t sector)
{
    return ks_geom_in_wake_sector(&CONE, a->x, a->y, b->x, b->y, sector) > 0.0f;
}

static void test_colouring_is_proper(void)
{
    ks_site_t s[12];
    const int n = grid(s);
    for (int deg = 0; deg < 360; deg += 10) {
        const uint8_t sec = ks_sector_of((float)deg);
        ks_geom_t g;
        ks_geom_build(&g, &CONE, s, n, 1, sec);
        for (int i = 0; i < n; ++i) {
            for (int k = i + 1; k < n; ++k) {
                if (g.color[i] != g.color[k]) continue;
                /* same colour => disjoint influence sets */
                KT_CHECK(!influences(&g.site[i], &g.site[k], sec));
                KT_CHECK(!influences(&g.site[k], &g.site[i], sec));
                for (int j = 0; j < n; ++j) {
                    if (j == i || j == k) continue;
                    KT_CHECK(!(influences(&g.site[i], &g.site[j], sec) &&
                               influences(&g.site[k], &g.site[j], sec)));
                }
            }
        }
    }
}

static void test_colouring_independent_of_input_order(void)
{
    ks_site_t s[12], r[12];
    const int n = grid(s);
    for (int i = 0; i < n; ++i) r[i] = s[n - 1 - i];
    ks_geom_t a, b;
    const uint8_t sec = ks_sector_of(250.0f);
    ks_geom_build(&a, &CONE, s, n, 6, sec);
    ks_geom_build(&b, &CONE, r, n, 6, sec);
    KT_CHECK(a.n_colors == b.n_colors);
    KT_CHECK(memcmp(a.color, b.color, sizeof a.color) == 0);
    KT_CHECK(a.n_down == b.n_down);
}

static void test_sector_index(void)
{
    KT_CHECK(ks_sector_of(0.0f) == 0);
    KT_CHECK(ks_sector_of(4.9f) == 0);
    KT_CHECK(ks_sector_of(5.1f) == 1);
    KT_CHECK(ks_sector_of(355.1f) == 0);
    KT_CHECK(ks_sector_of(270.0f) == 27);
    KT_CHECK(ks_sector_of(-90.0f) == 27);
}

int main(void)
{
    printf("test_geom\n");
    KT_RUN(test_west_wind_rows);
    KT_RUN(test_east_wind_reverses);
    KT_RUN(test_north_wind_columns);
    KT_RUN(test_colouring_is_proper);
    KT_RUN(test_colouring_independent_of_input_order);
    KT_RUN(test_sector_index);
    KT_DONE();
}

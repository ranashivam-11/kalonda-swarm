/*
 * ktest.h - a dependency-free unit-test harness for the firmware core.
 */
#ifndef KTEST_H
#define KTEST_H

#include <math.h>
#include <stdio.h>

static int kt_failures = 0;
static int kt_checks = 0;

#define KT_CHECK(cond)                                                              \
    do {                                                                            \
        ++kt_checks;                                                                \
        if (!(cond)) {                                                              \
            ++kt_failures;                                                          \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                           \
    } while (0)

#define KT_NEAR(a, b, tol)                                                          \
    do {                                                                            \
        ++kt_checks;                                                                \
        const double kt_a_ = (double)(a), kt_b_ = (double)(b);                     \
        if (!(fabs(kt_a_ - kt_b_) <= (double)(tol))) {                             \
            ++kt_failures;                                                          \
            fprintf(stderr, "%s:%d: NEAR failed: %s = %.9g, %s = %.9g, tol %g\n",   \
                    __FILE__, __LINE__, #a, kt_a_, #b, kt_b_, (double)(tol));       \
        }                                                                           \
    } while (0)

#define KT_RUN(fn)                                                                  \
    do {                                                                            \
        const int kt_before_ = kt_failures;                                         \
        fn();                                                                       \
        printf("  %-44s %s\n", #fn, kt_failures == kt_before_ ? "ok" : "FAIL");     \
    } while (0)

#define KT_DONE()                                                                   \
    do {                                                                            \
        printf("%d checks, %d failures\n", kt_checks, kt_failures);                \
        return kt_failures ? 1 : 0;                                                 \
    } while (0)

#endif /* KTEST_H */

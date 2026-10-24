/*
 * ks_sil.h - flat C ABI over the firmware core for software-in-the-loop use
 * from Python (ctypes). Only scalars, byte buffers and parameter *names*
 * cross the boundary, so Python never mirrors a C struct layout.
 */
#ifndef KS_SIL_H
#define KS_SIL_H

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
#define KS_SIL_API __attribute__((visibility("default")))
#else
#define KS_SIL_API
#endif

/* Sensor vector layout for ks_sil_step(). */
enum {
    KS_SIL_T_MS = 0,
    KS_SIL_RPM,
    KS_SIL_VBUS,
    KS_SIL_IBUS,
    KS_SIL_U,
    KS_SIL_ANEMO_VALID,
    KS_SIL_DIR,
    KS_SIL_VANE_VALID,
    KS_SIL_YAW,
    KS_SIL_TSTATOR,
    KS_SIL_TMCU,
    KS_SIL_BATT,
    KS_SIL_DRVFAULTS,
    KS_SIL_NSENS
};

/* Output vector layout. */
enum {
    KS_SIL_O_YAWCMD = 0,
    KS_SIL_O_OFFSET,
    KS_SIL_O_YAWEN,
    KS_SIL_O_BRAKE,
    KS_SIL_O_MODE,
    KS_SIL_O_STATUS,
    KS_SIL_O_FAULTS,
    KS_SIL_O_NTX,
    KS_SIL_O_NLOG,
    KS_SIL_O_PERSIST,
    KS_SIL_NOUT
};

KS_SIL_API int ks_sil_abi_version(void);
KS_SIL_API void *ks_sil_new(void);
KS_SIL_API void ks_sil_free(void *h);
KS_SIL_API int ks_sil_set(void *h, const char *name, double value);
KS_SIL_API int ks_sil_get_param(void *h, const char *name, double *value);
KS_SIL_API void ks_sil_set_key(void *h, const uint8_t key[16]);
KS_SIL_API int ks_sil_init(void *h, uint16_t boot, uint32_t now_ms);
KS_SIL_API int ks_sil_rx(void *h, const uint8_t *data, int len, int rssi, uint32_t t_ms);
KS_SIL_API void ks_sil_step(void *h, const double *sens, double *outv);
/* Step n nodes in one call: sens is (n, KS_SIL_NSENS), outv is (n, KS_SIL_NOUT),
 * both row-major. Saves n-1 foreign-function round trips per control step. */
KS_SIL_API void ks_sil_step_batch(void **h, int n, const double *sens, double *outv);
KS_SIL_API int ks_sil_tx(void *h, int i, uint8_t *buf, int cap);
KS_SIL_API int ks_sil_log(void *h, int i, uint8_t *buf64);
KS_SIL_API double ks_sil_query(void *h, const char *what);
KS_SIL_API void ks_sil_lut_set(void *h, int sector, double off_deg);
KS_SIL_API double ks_sil_lut_get(void *h, int sector, const char *field);

/* Stateless helpers exposed for cross-language tests. */
KS_SIL_API uint64_t ks_sil_siphash(const uint8_t key[16], const uint8_t *msg, int len);
KS_SIL_API uint16_t ks_sil_crc16(const uint8_t *msg, int len);
KS_SIL_API int ks_sil_sector_of(double dir_deg);
/* Build the wake graph; writes colour per site (in input order) and the
 * downstream ids of site `self_idx` into down[]. Returns n_down, or -1. */
KS_SIL_API int ks_sil_geom(const uint16_t *ids, const double *x, const double *y, int n,
                           int self_idx, int sector, double rotor_d, double k, double margin_d,
                           double max_d, uint8_t *colors, uint16_t *down, int *n_up);

#endif /* KS_SIL_H */

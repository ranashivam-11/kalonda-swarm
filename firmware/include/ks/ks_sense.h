/*
 * ks_sense.h - portable signal conditioning used by the hardware drivers.
 *
 * Why Hall sensors: optical encoders foul within weeks in the dust of the
 * Gangetic plain; Hall switches read the rotor's own magnet field through a
 * sealed housing. Speed comes from edge *timing* (not counting), because at
 * 2 m/s cut-in the rotor turns at ~80 rpm and a count window of 100 ms would
 * often contain a single edge.
 */
#ifndef KS_SENSE_H
#define KS_SENSE_H

#include "ks_types.h"

/* ---------- rotor speed from Hall edge timestamps ---------------------- */
#define KS_ROTOR_EDGES 9

typedef struct {
    uint32_t t_us[KS_ROTOR_EDGES];
    uint8_t head;
    uint8_t count;
    uint32_t n_glitch;
} ks_rotor_t;

void ks_rotor_init(ks_rotor_t *r);
/* Record a Hall edge (timestamp in microseconds, wraps at 2^32). */
void ks_rotor_edge(ks_rotor_t *r, uint32_t t_us);
/* Speed [rpm]: median edge period, bounded by the time since the last edge
 * (so a stopping rotor decays smoothly), 0 after timeout_us. */
float ks_rotor_rpm(const ks_rotor_t *r, uint32_t now_us, float edges_per_rev, uint32_t timeout_us);

/* ---------- cup anemometer (IEC 61400-12-1 linear transfer function) --- */
typedef struct {
    float slope;        /* m/s per Hz                                  */
    float offset;       /* m/s                                         */
    float threshold_hz; /* below: cups stalled, report 0               */
} ks_anemo_cal_t;

float ks_anemo_speed(const ks_anemo_cal_t *c, float f_hz);

/* ---------- potentiometer wind vane ----------------------------------- */
typedef struct {
    uint16_t code_min, code_max; /* ADC codes at 0 and (360 - dead band) deg */
    float deadband_deg;          /* open arc of the potentiometer track     */
    float north_offset_deg;      /* installation alignment                   */
} ks_vane_cal_t;

/* Returns false inside the dead band or outside the code range. */
bool ks_vane_dir(const ks_vane_cal_t *c, uint16_t code, float *dir_deg);

/* ---------- NTC thermistor (beta equation) ----------------------------- */
typedef struct {
    float r0_ohm;    /* resistance at t0           */
    float t0_c;
    float beta_k;
    float r_series;  /* divider: Vcc - R_series - NTC - GND */
} ks_ntc_t;

/* ratio = V_ntc / V_cc in (0,1). Returns NAN when out of range. */
float ks_ntc_temp_c(const ks_ntc_t *n, float ratio);

#endif /* KS_SENSE_H */

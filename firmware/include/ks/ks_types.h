/*
 * ks_types.h - shared types, status and fault bits.
 */
#ifndef KS_TYPES_H
#define KS_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ks_config.h"

#define KS_PI 3.14159265358979323846f
#define KS_DEG2RAD (KS_PI / 180.0f)
#define KS_RAD2DEG (180.0f / KS_PI)

#define KS_NODE_NONE 0xFFFFu
#define KS_NODE_BROADCAST 0xFFFFu

/* Which objective a node optimises when it scores a yaw trial. */
typedef enum {
    KS_UTIL_SELFISH = 0,     /* own measured power only                    */
    KS_UTIL_COOPERATIVE = 1  /* own power + beta * measured downstream gain */
} ks_utility_t;

/* Supervisor mode, ordered by severity. */
typedef enum {
    KS_MODE_NORMAL = 0,
    KS_MODE_DERATE = 1,   /* thermal: yaw out of the wind to unload the stator */
    KS_MODE_SAFE = 2,     /* zero offset, no trials                           */
    KS_MODE_SHUTDOWN = 3  /* brake + active furl                              */
} ks_mode_t;

/* Toggle schedule for validation campaigns (Fleming et al. 2019 style). */
typedef enum {
    KS_TOGGLE_ALWAYS_ON = 0,
    KS_TOGGLE_ALTERNATE = 1,  /* on/off every toggle_period_s                */
    KS_TOGGLE_RANDOMIZED = 2  /* keyed pseudo-random, balanced, unaliased    */
} ks_toggle_t;

/* Status bits (16 bit, carried in beacons). */
enum {
    KS_ST_CONSENSUS_OK = 1u << 0,
    KS_ST_SYNCED = 1u << 1,
    KS_ST_TOGGLE_ON = 1u << 2,
    KS_ST_FRONT_ROW = 1u << 3,
    KS_ST_TRIAL_INIT = 1u << 4,
    KS_ST_TRIAL_PART = 1u << 5,
    KS_ST_REF_VALID = 1u << 6,
    KS_ST_ANEMO_OK = 1u << 7,  /* own anemometer usable as free-stream ref */
    KS_ST_VANE_OK = 1u << 8,   /* own vane usable for consensus            */
    KS_ST_MODE_SHIFT = 12      /* bits 12..13 carry ks_mode_t             */
};

/* Fault bits (16 bit). Drivers may also raise bits through the sensor
 * struct; the supervisor raises the rest. */
enum {
    KS_F_ANEMO_STUCK = 1u << 0,
    KS_F_ANEMO_DRIFT = 1u << 1,
    KS_F_VANE_STUCK = 1u << 2,
    KS_F_HALL_TIMEOUT = 1u << 3,
    KS_F_ADC_RANGE = 1u << 4,
    KS_F_OVERSPEED = 1u << 5,
    KS_F_OVERTEMP_STATOR = 1u << 6,
    KS_F_OVERTEMP_MCU = 1u << 7,
    KS_F_MAGNET_HOT = 1u << 8,
    KS_F_DEMAG_SUSPECT = 1u << 9,
    KS_F_YAW_STALL = 1u << 10,
    KS_F_RADIO_ISOLATED = 1u << 11,
    KS_F_BATTERY_LOW = 1u << 12,
    KS_F_HIGH_WIND = 1u << 13,
    KS_F_CUTOUT = 1u << 14,
    KS_F_AUTH = 1u << 15
};

/* Trial outcomes (DECIDE message, logs). */
typedef enum {
    KS_OUT_NONE = 0,
    KS_OUT_ACCEPT = 1,
    KS_OUT_REJECT = 2,
    KS_OUT_ABORT = 3,
    KS_OUT_HOLD = 4     /* inconclusive: evidence pooled, the trial repeats */
} ks_outcome_t;

/* Quality bits a participant attaches to its report. */
enum {
    KS_Q_FEW_BLOCKS = 1u << 0,  /* too few valid blocks in A or B           */
    KS_Q_DRIFT = 1u << 1,       /* A1 and A2 disagree: non-stationary wind  */
    KS_Q_SECTOR = 1u << 2,      /* wind left the sector during the trial    */
    KS_Q_MODE = 1u << 3,        /* supervisor left NORMAL during the trial  */
    KS_Q_NOREF = 1u << 4,       /* no free-stream reference for some blocks */
    KS_Q_MISSED = 1u << 5,      /* started late / missed a phase            */
    KS_Q_SELF = 1u << 6         /* participant's own offset changed (side flip
                                   or bin change): its contrast is not the
                                   initiator's alone                        */
};

#endif /* KS_TYPES_H */

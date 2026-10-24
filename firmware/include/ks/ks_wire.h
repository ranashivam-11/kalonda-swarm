/*
 * ks_wire.h - the yaw-negotiation wire format (protocol version 1).
 *
 * Frame = 16-byte header | payload | 8-byte SipHash-2-4 tag.
 * All integers little-endian, floats IEEE-754 binary32. Byte-exact layout is
 * specified in docs/protocol.md and mirrored by python/kalonda/telemetry/wire.py.
 *
 *   off size field
 *     0   1  version<<4 | type
 *     1   1  flags (reserved, 0)
 *     2   2  src node id
 *     4   2  dst node id (0xFFFF = broadcast)
 *     6   2  boot counter of src (persisted in NVS)
 *     8   2  seq (per src, per boot)
 *    10   4  epoch (synchronised phase counter)
 *    14   2  ms into the epoch at transmission (saturates at 65535)
 */
#ifndef KS_WIRE_H
#define KS_WIRE_H

#include "ks_types.h"

#define KS_PROTO_VERSION 1u
#define KS_HDR_BYTES 16u
#define KS_TAG_BYTES 8u

typedef enum {
    KS_MSG_BEACON = 1,
    KS_MSG_PROPOSE = 2,
    KS_MSG_REPORT = 3,
    KS_MSG_DECIDE = 4
} ks_msg_type_t;

typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t flags;
    uint16_t src, dst, boot, seq;
    uint32_t epoch;
    uint16_t epoch_ms;
} ks_hdr_t;

/* 1 Hz state broadcast (30 bytes, fixed-point on the wire). */
typedef struct {
    float x_m, y_m;          /* surveyed position (sent as decimetres)     */
    float u_ms;              /* own anemometer, 1-s mean                   */
    float dir_deg;           /* own vane, 1-s circular mean                */
    float cons_dir_deg;      /* own consensus estimate                     */
    float uref_ms;           /* own view of the free-stream reference      */
    float power_w;           /* electrical power, 1-s mean                 */
    float rpm;
    float yaw_off_deg;       /* applied yaw offset                         */
    uint16_t status;
    uint16_t faults;
    int8_t temp_c;           /* stator temperature, rounded                */
    uint8_t sector;          /* LUT bin: sector + 36 * stability class      */
    uint8_t beta_u8;         /* cooperation weight * 255 (0 = selfish)     */
    uint8_t n_nb;            /* alive neighbours                           */
    uint32_t uptime_s;
} ks_beacon_t;

/* Trial announcement (12 + 2*n_part bytes). */
typedef struct {
    uint16_t trial_id;
    uint8_t sector;
    uint8_t n_part;
    float base_deg, trial_deg;
    uint32_t start_epoch;    /* epoch of phase A1                          */
    uint16_t part[KS_MAX_PART];
} ks_propose_t;

/* Participant measurement (19 bytes). */
typedef struct {
    uint16_t trial_id;
    uint16_t initiator;
    float delta_w;           /* mean(B) - mean(A), referenced to trial-mean u_ref^3 [W] */
    float se_w;              /* standard error of delta_w [W]              */
    float base_w;            /* baseline power, same reference [W]         */
    uint16_t n_blocks;
    uint8_t quality;         /* KS_Q_* */
} ks_report_t;

/* Outcome broadcast (20 bytes). */
typedef struct {
    uint16_t trial_id;
    uint8_t sector;
    uint8_t outcome;         /* ks_outcome_t */
    float new_deg;
    float score_w, score_se_w;   /* pooled score the decision used [W]     */
    float coop_score_w;          /* own + downstream, logged in every mode */
    uint8_t n_reports;
    uint8_t n_part;
} ks_decide_t;

typedef union {
    ks_beacon_t beacon;
    ks_propose_t propose;
    ks_report_t report;
    ks_decide_t decide;
} ks_payload_t;

/* Payload size on the wire for a message type (0 if unknown). */
size_t ks_wire_payload_len(uint8_t type, const ks_payload_t *pl);

/* Encode header + payload + tag into buf. Returns frame length or 0. */
size_t ks_wire_encode(uint8_t *buf, size_t cap, const ks_hdr_t *h,
                      const ks_payload_t *pl, const uint8_t key[16]);

/* Decode and authenticate. Returns 0 on success or a negative error:
 *   -1 too short / too long, -2 bad version, -3 bad tag, -4 unknown type,
 *   -5 payload length mismatch, -6 payload field out of range. */
int ks_wire_decode(const uint8_t *buf, size_t len, const uint8_t key[16],
                   ks_hdr_t *h, ks_payload_t *pl);

#endif /* KS_WIRE_H */

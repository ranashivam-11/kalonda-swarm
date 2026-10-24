/*
 * ks_config.h - compile-time limits for the Kalonda swarm firmware core.
 *
 * Everything in the core is statically allocated: no malloc, no recursion,
 * bounded loops. The limits below size every table in ks_node_t.
 */
#ifndef KS_CONFIG_H
#define KS_CONFIG_H

/* Largest array the neighbour table can hold (the whole micro-array is a
 * single ESP-NOW broadcast domain, so every node tracks every other node). */
#ifndef KS_MAX_NODES
#define KS_MAX_NODES 64
#endif

/* Maximum number of downstream participants in one yaw trial. */
#ifndef KS_MAX_PART
#define KS_MAX_PART 8
#endif

/* Concurrent trials a node may observe as a participant. Graph colouring
 * makes overlapping trials impossible when every node agrees on the wind
 * sector; the second slot absorbs the transient when they briefly do not. */
#ifndef KS_MAX_OBSERVE
#define KS_MAX_OBSERVE 2
#endif

/* ESP-NOW v1.0 payload limit (v2.0 allows 1470 B, but v1.0 receivers
 * truncate anything above 250 B, so the protocol never exceeds it). */
#define KS_FRAME_MAX 250

/* Frames a node may emit in one step, log records per step. */
#define KS_TX_MAX 6
#define KS_LOG_MAX 8

/* Received frames buffered between two steps. */
#define KS_RX_QUEUE 32

/* Wind-direction sectors of the yaw-offset look-up table (10 degrees). */
#define KS_SECTORS 36
#define KS_SECTOR_DEG (360.0f / (float)KS_SECTORS)

/* Atmospheric-stability classes measured from the free-stream turbulence
 * intensity: wake steering pays in stable (low-TI) air and not in convective
 * (high-TI) air, so the learned table is indexed by (sector, class). */
#define KS_TI_CLASSES 2
#define KS_BINS (KS_SECTORS * KS_TI_CLASSES)

/* Control step period. The port calls ks_node_step() at this rate. */
#define KS_STEP_MS 100u

/* Phases of one ABBA trial (A1, B1, B2, A2). */
#define KS_PHASES 4

/* Log record size on the wire / on flash. */
#define KS_LOG_REC_BYTES 64

#endif /* KS_CONFIG_H */

/*
 * ks_crypto.h - integrity primitives.
 *
 * Frames are broadcast over ESP-NOW, which cannot encrypt broadcast traffic,
 * so every frame carries a 64-bit SipHash-2-4 tag (Aumasson & Bernstein,
 * INDOCRYPT 2012) under an array-wide key. That stops spoofed proposals and
 * forged credit reports from outside the array; it does not stop a keyed
 * node from lying (see docs/multi_agent.md, "incentive compatibility").
 *
 * Log records use CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), which only
 * detects storage corruption.
 */
#ifndef KS_CRYPTO_H
#define KS_CRYPTO_H

#include <stddef.h>
#include <stdint.h>

uint64_t ks_siphash24(const uint8_t key[16], const uint8_t *msg, size_t len);

uint16_t ks_crc16_ccitt(const uint8_t *data, size_t len);

/* Constant-time comparison of n bytes; returns 1 if equal. */
int ks_ct_equal(const uint8_t *a, const uint8_t *b, size_t n);

#endif /* KS_CRYPTO_H */

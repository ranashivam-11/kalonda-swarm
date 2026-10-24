/*
 * ks_bytes.h - explicit little-endian packing. The wire and log formats never
 * depend on struct layout or host endianness.
 */
#ifndef KS_BYTES_H
#define KS_BYTES_H

#include <stdint.h>
#include <string.h>

static inline void ks_put_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}

static inline void ks_put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static inline void ks_put_f32(uint8_t *b, float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    ks_put_u32(b, u);
}

static inline uint16_t ks_get_u16(const uint8_t *b)
{
    return (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
}

static inline uint32_t ks_get_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) |
           ((uint32_t)b[3] << 24);
}

static inline float ks_get_f32(const uint8_t *b)
{
    uint32_t u = ks_get_u32(b);
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}

#endif /* KS_BYTES_H */

#pragma once

#include <stdint.h>

static inline void vga_bitmap_init_pairs(uint16_t *pairs, const uint8_t *signals)
{
    for (unsigned i = 0; i < 64; ++i)
        pairs[i] = (uint16_t)(signals[i >> 3] | ((uint16_t)signals[i & 7] << 8));
}

/* The DMA buffer is aligned; I2S outputs each word in 2,3,0,1 order. */
static inline __attribute__((always_inline)) void
vga_bitmap_expand8(uint16_t *dest, const uint8_t *src, const uint16_t *pairs)
{
    uint32_t packed = ((uint32_t)src[0] << 16) |
                      ((uint32_t)src[1] << 8) | src[2];
    dest[0] = pairs[(packed >> 12) & 63];
    dest[1] = pairs[(packed >> 18) & 63];
    dest[2] = pairs[packed & 63];
    dest[3] = pairs[(packed >> 6) & 63];
}

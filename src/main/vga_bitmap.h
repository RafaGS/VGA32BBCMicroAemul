#pragma once

#include <stdint.h>

#ifndef VGA_BITMAP_MARGIN_Y
#define VGA_BITMAP_MARGIN_Y 32u
#endif

static inline uint32_t vga_bitmap_source_y(uint32_t output_y,
                                           uint32_t source_height,
                                           uint32_t output_height)
{
    /* Bias the final source row into the VGA line before the display edge. */
    return ((output_y + 1u) * source_height - 1u) / output_height;
}

static inline int32_t vga_bitmap_viewport_source_y(uint32_t output_y,
                                                   uint32_t source_height,
                                                   uint32_t output_height)
{
    if (output_height <= 2u * VGA_BITMAP_MARGIN_Y ||
        output_y < VGA_BITMAP_MARGIN_Y ||
        output_y >= output_height - VGA_BITMAP_MARGIN_Y)
        return -1;

    return (int32_t)vga_bitmap_source_y(
        output_y - VGA_BITMAP_MARGIN_Y, source_height,
        output_height - 2u * VGA_BITMAP_MARGIN_Y);
}

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

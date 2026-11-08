/*
 * bbc_video_ula.c — Acorn Video ULA emulation for BBC Micro / ESP32-ESP-IDF
 *
 * References: beebwiki.mdfs.net/Video_ULA, B-em video.c (GPL-2, reference only),
 *             BeebFpga vidproc.vhd (reference only).
 *
 * Key implementation notes:
 *   - The palette &FE21 write: bits 7-4 = logical colour, bits 3-0 = physical.
 *     The physical bits are stored XOR'd with 0x07 (BBC hardware quirk).
 *   - Palette selection follows the ULA's four taps on the shift register.
 *
 * Licence: zlib
 * Copyright (c) 2026 esp-beep project
 */

#include <string.h>
#include "bbc_video_ula.h"

#ifdef ESP_PLATFORM
#  include "esp_log.h"
#  include "esp_attr.h"
#  define ULA_LOGD(fmt, ...) ESP_LOGD("bbc_ula", fmt, ##__VA_ARGS__)
#  define RENDER_IRAM IRAM_ATTR
#else
#  define ULA_LOGD(fmt, ...) /* no-op */
#  define RENDER_IRAM
#endif

/* --------------------------------------------------------------------------
 * Default palette — MOS 1.20 default: logical colour N → physical colour N
 * for logical colours 0-7; entries 8-15 initially alias those colours.
 * -------------------------------------------------------------------------- */
static const uint8_t s_default_palette[16] = {
    /* Logical 0-7: direct 1:1 */
    BBC_COL_BLACK,   /* 0 */
    BBC_COL_RED,     /* 1 */
    BBC_COL_GREEN,   /* 2 */
    BBC_COL_YELLOW,  /* 3 */
    BBC_COL_BLUE,    /* 4 */
    BBC_COL_MAGENTA, /* 5 */
    BBC_COL_CYAN,    /* 6 */
    BBC_COL_WHITE,   /* 7 */
    /* Logical 8-15: initial aliases */
    BBC_COL_BLACK,   /* 8  */
    BBC_COL_RED,     /* 9  */
    BBC_COL_GREEN,   /* 10 */
    BBC_COL_YELLOW,  /* 11 */
    BBC_COL_BLUE,    /* 12 */
    BBC_COL_MAGENTA, /* 13 */
    BBC_COL_CYAN,    /* 14 */
    BBC_COL_WHITE,   /* 15 */
};

/* --------------------------------------------------------------------------
 * Colour table: physical colour (0-7, BBB_COL_*) → RGB
 * BBC uses 3-bit RGB: bit 0=R, bit 1=G, bit 2=B
 * -------------------------------------------------------------------------- */
static const bbc_rgb_t s_colour_table[8] = {
    { 0,   0,   0   }, /* 0: Black   */
    { 255, 0,   0   }, /* 1: Red     */
    { 0,   255, 0   }, /* 2: Green   */
    { 255, 255, 0   }, /* 3: Yellow  */
    { 0,   0,   255 }, /* 4: Blue    */
    { 255, 0,   255 }, /* 5: Magenta */
    { 0,   255, 255 }, /* 6: Cyan    */
    { 255, 255, 255 }, /* 7: White   */
};

/* --------------------------------------------------------------------------
 * Rebuild lookup tables after any palette change
 * -------------------------------------------------------------------------- */
void bbc_video_ula_rebuild_tables(bbc_video_ula_t *ula)
{
    for (int byte = 0; byte < 256; byte++) {
        uint8_t shift = (uint8_t)byte;
        for (int px = 0; px < 8; px++) {
            uint8_t logical = (uint8_t)(((shift >> 4) & 8) |
                                        ((shift >> 3) & 4) |
                                        ((shift >> 2) & 2) |
                                        ((shift >> 1) & 1));
            uint8_t colour = ula->palette[logical];
            if ((colour & 8) && ula->flash_state)
                colour ^= 7;
            colour &= 7;
            ula->lut_8_pixels[byte][px] = colour;
            if (px < 4) ula->lut_4_pixels[byte][px] = colour;
            if (px < 2) ula->lut_2_pixels[byte][px] = colour;
            shift = (uint8_t)((shift << 1) | 1);
        }
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void bbc_video_ula_init(bbc_video_ula_t *ula)
{
    memset(ula, 0, sizeof(*ula));
    /* Install colour table */
    for (int i = 0; i < 8; i++)
        ula->colour_table[i] = s_colour_table[i];
    bbc_video_ula_reset(ula);
}

void bbc_video_ula_reset(bbc_video_ula_t *ula)
{
    ula->control       = 0;
    ula->teletext_mode = false;
    ula->pixels_per_byte = 2;
    ula->crtc_2mhz     = false;
    ula->flash_state   = false;

    /* Copy default palette */
    for (int i = 0; i < 16; i++)
        ula->palette[i] = s_default_palette[i];

    bbc_video_ula_rebuild_tables(ula);
    ULA_LOGD("reset");
}

void bbc_video_ula_write(bbc_video_ula_t *ula, uint8_t addr, uint8_t data)
{
    if (!(addr & 1)) {
        /*
         * &FE20 — Control register
         *
         * Bit 0: flash colour state (MOS toggles this ~1 Hz)
         * Bit 1: teletext mode select
         * Bits 3-2: pixel clock (2, 4, 8 or 16 MHz)
         * Bit 4: CRTC 2 MHz (1 = 2 MHz, 0 = 1 MHz)
         * Bits 7-5: cursor segments
         */
        ula->control     = data;
        ula->flash_state = (data & ULA_CTRL_FLASH_STATE) != 0;
        ula->teletext_mode = (data & ULA_CTRL_TELETEXT) != 0;
        ula->crtc_2mhz   = (data & ULA_CTRL_CRTC_2MHZ) != 0;

        uint8_t rate = (data & ULA_CTRL_PIXEL_RATE_MASK) >> ULA_CTRL_PIXEL_RATE_SHIFT;
        ula->pixels_per_byte = (uint8_t)((2u << rate) >> (ula->crtc_2mhz ? 1 : 0));
        /* Rebuild tables as flash_state affects palette output */
        bbc_video_ula_rebuild_tables(ula);
        ULA_LOGD("ctrl=%02X teletext=%d bpp=%d 2mhz=%d",
                 data, ula->teletext_mode, ula->pixels_per_byte, ula->crtc_2mhz);
    } else {
        /*
         * &FE21 — Palette register
         *
         * Bits 7-4: logical colour index (0-15)
         * Bits 3-0: physical colour data
         *
         * The physical colour bits are inverted (XOR 7); bit 3 selects
         * flashing. Writing 0 to bits 2-0 stores white, 7 stores black.
         */
        uint8_t logical  = (data >> 4) & 0x0F;
        uint8_t physical = (data & 0x0F) ^ 0x07;
        ula->palette[logical] = physical;
        bbc_video_ula_rebuild_tables(ula);
        ULA_LOGD("palette[%d] = %d (raw=%02X)", logical, physical, data);
    }
}

RENDER_IRAM int bbc_video_ula_serialize(const bbc_video_ula_t *ula,
                             uint8_t data_byte,
                             uint8_t *out_colours,
                             bool cursor_active)
{
    int n;
    switch (ula->pixels_per_byte) {
        case 16:
        case 8:
            n = ula->pixels_per_byte;
            for (int i = 0; i < 8; i++)
                out_colours[i] = ula->lut_8_pixels[data_byte][i];
            for (int i = 8; i < n; i++)
                out_colours[i] = ula->lut_8_pixels[0xff][0];
            break;
        case 4:
            n = 4;
            for (int i = 0; i < 4; i++)
                out_colours[i] = ula->lut_4_pixels[data_byte][i];
            break;
        case 2:
            n = 2;
            for (int i = 0; i < 2; i++)
                out_colours[i] = ula->lut_2_pixels[data_byte][i];
            break;
        default:
            n = 1;
            out_colours[0] = ula->lut_2_pixels[data_byte][0];
            break;
    }

    /* Cursor: XOR physical colour with 7 = invert all 3 bits */
    if (cursor_active) {
        for (int i = 0; i < n; i++)
            out_colours[i] ^= 7;
    }
    return n;
}

void bbc_video_ula_toggle_flash(bbc_video_ula_t *ula)
{
    ula->flash_state = !ula->flash_state;
    ula->control ^= ULA_CTRL_FLASH_STATE;
    bbc_video_ula_rebuild_tables(ula);
    ULA_LOGD("flash=%d", ula->flash_state);
}

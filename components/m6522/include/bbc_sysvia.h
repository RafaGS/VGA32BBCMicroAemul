/*
 * bbc_sysvia.h — BBC Micro System VIA (IC3, &FE40-&FE4F) for ESP32/ESP-IDF
 *
 * Wraps the generic m6522_t and adds:
 *   - Addressable latch IC32 (74LS259) on Port B bits 0-3
 *   - Slow data bus on Port A (keyboard, SN76489 sound chip)
 *   - VSYNC → CA1 (generates frame-rate IRQ)
 *
 * Licence: zlib (adaptation of floooh/chips) / GPL-2.0 (B-em derived logic)
 */

#pragma once
#include "m6522.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Addressable latch IC32 bit positions
 * -------------------------------------------------------------------------- */
#define BBC_LATCH_SOUND_WE      0  /* Sound chip (SN76489) write enable — active LOW  */
#define BBC_LATCH_SPEECH_RD     1  /* Speech chip read (not implemented)               */
#define BBC_LATCH_TAPE_MOTOR    2  /* Cassette motor relay — active HIGH               */
#define BBC_LATCH_KB_AUTOSCAN   3  /* Keyboard auto-scan enable                        */
#define BBC_LATCH_VIDEO_C0      4  /* IC32 Q4: video address wrap control C0           */
#define BBC_LATCH_VIDEO_C1      5  /* IC32 Q5: video address wrap control C1           */
#define BBC_LATCH_CAPS_LED      6  /* Caps Lock LED — active LOW                       */
#define BBC_LATCH_SHIFT_LED     7  /* Shift Lock LED — active LOW                      */

/* --------------------------------------------------------------------------
 * Callbacks
 * -------------------------------------------------------------------------- */
typedef struct {
    /*
     * Write a byte to the SN76489 sound chip.
     * Called when the sound write-enable latch bit goes LOW while Port A has data.
     */
    void (*sound_write)(void *user_ctx, uint8_t data);

    /*
     * Check if a specific key is pressed.
     * row: 0-7  (bits 6:4 of Port A — MOS writes (row<<4)|col to Port A)
     * col: 0-14 (bits 3:0 of Port A)
     * Returns: true if the key at (row, col) is currently pressed.
     *
     * BBC hardware: bit 7 of Port A reads LOW when the key is pressed
     * (active-low). The sysvia layer handles the inversion; this callback
     * just returns the logical pressed state.
     */
    bool (*keyboard_read)(void *user_ctx, uint8_t row, uint8_t col);

    /*
     * Called whenever the IC32 addressable latch value changes.
     * latch_bits: current 8-bit state of IC32.
     * The host can check individual bits using BBC_LATCH_* constants.
     */
    void (*latch_changed)(void *user_ctx, uint8_t latch_bits);

    /*
     * IRQ signal from the System VIA changed state.
     * The System VIA IRQ is OR'd with the ACIA IRQ and fed to the 6502 /IRQ pin.
     */
    void (*irq)(void *user_ctx, bool state);

    /*
     * Tape motor control via CB2 output.
     * Called whenever CB2 changes state (true = motor on, false = motor off).
     */
    void (*motor_changed)(void *user_ctx, bool on);

    void *user_ctx;
} bbc_sysvia_callbacks_t;

/* --------------------------------------------------------------------------
 * System VIA state
 * -------------------------------------------------------------------------- */
typedef struct {
    m6522_t  via;            /* Generic 6522 VIA                        */
    uint8_t  latch;          /* Current IC32 state (8 bits)             */
    bbc_sysvia_callbacks_t cb;
} bbc_sysvia_t;

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void    bbc_sysvia_init(bbc_sysvia_t *sv, const bbc_sysvia_callbacks_t *callbacks);
void    bbc_sysvia_reset(bbc_sysvia_t *sv);

/* Register access at &FE40-&FE4F (pass addr & 0x0F as reg) */
uint8_t bbc_sysvia_read(bbc_sysvia_t *sv, uint8_t reg);
void    bbc_sysvia_write(bbc_sysvia_t *sv, uint8_t reg, uint8_t val);

/* Clock tick — call once per 1 MHz BBC CPU cycle */
void    bbc_sysvia_tick(bbc_sysvia_t *sv, int32_t cycles);

/* VSYNC from 6845 CRTC → CA1 (generates frame IRQ at ~50 Hz) */
void    bbc_sysvia_vsync(bbc_sysvia_t *sv, bool state);

/* Read current IC32 latch state */
uint8_t bbc_sysvia_get_latch(const bbc_sysvia_t *sv);

/*
 * Notify sysvia that keyboard state has changed (key pressed/released).
 * Call this from the host whenever a key event occurs so that CA2 is
 * updated immediately, allowing the MOS keyboard scanner to wake up.
 */
void    bbc_sysvia_keyboard_updated(bbc_sysvia_t *sv);

#ifdef __cplusplus
}
#endif

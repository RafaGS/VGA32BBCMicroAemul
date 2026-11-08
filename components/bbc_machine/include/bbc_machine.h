/*
 * bbc_machine.h — BBC Micro top-level integration
 *
 * Holds all hardware components and wires them together:
 *
 *   CPU (6502)       ← bbc_cpu
 *   Memory           ← bbc_memory  (16 KB RAM, fixed BASIC/OS ROM, I/O)
 *   System VIA IC3   ← bbc_sysvia  (&FE40–&FE4F)
 *   CRTC MC6845      ← bbc_video   (&FE00–&FE01)
 *   Video ULA        ← bbc_video   (&FE20–&FE21)
 *   SN76489 PSG      ← sn76489     (written via IC32 latch bit 0 LOW)
 *
 * Signal routing:
 *   VSYNC  → bbc_sysvia_vsync()  → CA1  → IFR.1 → /IRQ
 *   sysvia IRQ                   → bbc_cpu_irq()
 *   IC32 SOUND_WE (bit 0) LOW    → sn76489_write() with sysvia port-A data
 *
 * Usage:
 *
 *   static bbc_machine_t machine;
 *   bbc_machine_init(&machine, os_rom, os_size, basic_rom, basic_size);
 *   bbc_machine_reset(&machine);
 *
 *   // In emulator loop:
 *   int cycles = bbc_machine_step(&machine);   // step CPU + tick peripherals
 *
 *   // To push a key event:
 *   bbc_machine_key_event(&machine, row, col, pressed);
 *
 * Licence: zlib
 * Copyright (c) 2026 esp-beep project
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "bbc_memory.h"
#include "bbc_cpu.h"
#include "bbc_sysvia.h"
#include "sn76489.h"
#include "bbc_video.h"
#include "bbc_tape.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BBC_MACHINE_MODEL_A 1

/* ------------------------------------------------------------------
 * Keyboard matrix
 *
 * BBC keyboard: 8 rows × 10 columns.
 * MOS writes (row<<4)|col to Port A; row = bits 6:4, col = bits 3:0.
 * Row 0: Shift(col0), Ctrl(col1)
 * Rows 1-7: letter/number/function keys (cols 0-9)
 * ------------------------------------------------------------------ */
#define BBC_KB_ROWS   8
#define BBC_KB_COLS  10

typedef struct {
    bool pressed[BBC_KB_ROWS][BBC_KB_COLS];
} bbc_keyboard_t;

/* ------------------------------------------------------------------
 * IRQ tracking — multiple sources share /IRQ line
 * ------------------------------------------------------------------ */
typedef struct {
    bool sysvia;
    bool acia;     /* ACIA (tape) IRQ */
} bbc_irq_state_t;

/* ------------------------------------------------------------------
 * Main machine struct
 * ------------------------------------------------------------------ */
typedef struct {
    /* Core */
    BBCMemory       *mem;
    BBCCPU          *cpu;

    /* Peripherals (stack-allocated, no heap) */
    bbc_sysvia_t     sysvia;
    sn76489_t        psg;
    bbc_video_t      video;

    /* Keyboard */
    bbc_keyboard_t   keyboard;

    /* IRQ line state */
    bbc_irq_state_t  irq;

    /* CPU is 2 MHz; preserve residual cycles in the 1 MHz VIA and selected
     * 1/2 MHz CRTC clock domains. */
    int32_t          sysvia_acc;
    int32_t          crtc_acc;

    /* Tape (UEF) */
    bbc_tape_t       tape;

    /* Video framebuffer output (set by caller before init) */
    bbc_video_output_t *fb_output;   /* NULL = no display */

    /* Optional frame-ready callback */
    void (*on_frame)(void *ctx);
    void  *on_frame_ctx;
} bbc_machine_t;

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

/*
 * Initialise the machine.  Loads ROMs, wires all callbacks, resets
 * everything. Framebuffer configuration may be supplied separately.
 */
void bbc_machine_init(bbc_machine_t *m,
                      const uint8_t *os_rom,    uint32_t os_size,
                      const uint8_t *basic_rom, uint32_t basic_size);

/* Hard reset — all chips reset, CPU reset vector fetched. */
void bbc_machine_reset(bbc_machine_t *m);

/*
 * Soft BREAK — resets CPU and peripherals but preserves RAM contents.
 * This matches the BBC hardware BREAK key behaviour: the 6502 /RESET line
 * is asserted but the RAM is not cleared. shift_held is recorded in the
 * MOS BREAK-state byte.
 */
void bbc_machine_break(bbc_machine_t *m, bool shift_held);

/*
 * Step the emulator: execute one CPU instruction and tick all
 * peripherals by the resulting number of cycles.
 * Returns the number of cycles consumed.
 */
int bbc_machine_step(bbc_machine_t *m);

/* Mount a UEF tape image. Pass NULL path to unmount. Returns 0 or -1. */
int bbc_machine_mount_tape(bbc_machine_t *m, const char *uef_path);

/*
 * Set / clear a key in the keyboard matrix.
 *   row  0–7, col  0–9
 */
void bbc_machine_key_event(bbc_machine_t *m, uint8_t row, uint8_t col, bool pressed);

/* Set framebuffer output (can be changed at any time) */
void bbc_machine_set_video_output(bbc_machine_t *m, bbc_video_output_t *out);

/* Set frame-ready callback */
void bbc_machine_set_frame_callback(bbc_machine_t *m,
                                     void (*cb)(void *ctx), void *ctx);

#ifdef __cplusplus
}
#endif

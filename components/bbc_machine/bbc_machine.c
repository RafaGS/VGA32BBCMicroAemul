/*
 * bbc_machine.c — BBC Micro top-level integration
 *
 * Licence: zlib
 * Copyright (c) 2026 esp-beep project
 */

#include "bbc_machine.h"
#include "bbc_tape.h"
#include <string.h>
#include <stdio.h>

#if defined(BBC_MACHINE_PROFILE)
#include <stdint.h>

static inline uint32_t machine_profile_ccount(void)
{
    uint32_t count;
    __asm__ __volatile__("rsr %0, ccount" : "=a"(count));
    return count;
}

//static uint32_t mp_divider = 0;
static uint32_t mp_rng = 0xA341316Cu;
static uint64_t mp_total = 0;

static uint32_t mp_samples = 0;

static uint64_t mp_cpu = 0;
static uint64_t mp_sysvia = 0;
static uint64_t mp_tape = 0;
static uint64_t mp_video = 0;
#endif

/* ======================================================================
 * Forward declarations for static callback functions
 * ====================================================================== */

/* sysvia callbacks */
static void     sv_sound_write  (void *ctx, uint8_t data);
static bool     sv_keyboard_read(void *ctx, uint8_t row, uint8_t col);
static void     sv_latch_changed(void *ctx, uint8_t latch_bits);
static void     sv_irq          (void *ctx, bool state);

/* tape callbacks and IO handlers */
static void     tape_irq        (void *ctx, bool state);
static void     sv_motor_changed(void *ctx, bool on);
static uint8_t  io_acia_read        (uint16_t addr, void *ctx);
static void     io_acia_write       (uint16_t addr, uint8_t val, void *ctx);
static void     io_serial_ula_write (uint16_t addr, uint8_t val, void *ctx);

/* video vsync callback */
static void     video_vsync_cb(void *ctx, bool state);

/* bbc_memory I/O callbacks (per Sheila address range) */
static uint8_t  io_crtc_read     (uint16_t addr, void *ctx);
static void     io_crtc_write    (uint16_t addr, uint8_t val, void *ctx);
static uint8_t  io_vidproc_read  (uint16_t addr, void *ctx);
static void     io_vidproc_write (uint16_t addr, uint8_t val, void *ctx);
static uint8_t  io_sysvia_read   (uint16_t addr, void *ctx);
static void     io_sysvia_write  (uint16_t addr, uint8_t val, void *ctx);

/* ======================================================================
 * IRQ helpers
 * ====================================================================== */

/* Recalculate the CPU /IRQ line from System VIA and ACIA. */
static void update_irq(bbc_machine_t *m) {
    if (m->irq.sysvia || m->irq.acia) {
        bbc_cpu_irq(m->cpu);
    } else {
        bbc_cpu_clear_irq(m->cpu);
    }
}

/* ======================================================================
 * bbc_machine_init
 * ====================================================================== */

void bbc_machine_init(bbc_machine_t *m,
                      const uint8_t *os_rom,    uint32_t os_size,
                      const uint8_t *basic_rom, uint32_t basic_size) {
    memset(m, 0, sizeof(*m));

    /* ----- Memory --------------------------------------------------- */
    m->mem = bbc_memory_create();

    if (os_rom    && os_size)    bbc_memory_load_rom(m->mem, os_rom,    os_size,    0xC000);
    if (basic_rom && basic_size) bbc_memory_load_rom(m->mem, basic_rom, basic_size, 0x8000);

    /* ----- CPU ------------------------------------------------------ */
    /* The CPU read/write callbacks just call through to bbc_memory.    */
    /* We store m itself as userData so the wrappers can access m->mem. */
    m->cpu = bbc_cpu_create(
            /* read  */ bbc_memory_read,
            /* write */ bbc_memory_write,
            /* ctx   */ m->mem
            );


    /* ----- System VIA ---------------------------------------------- */
    {
        bbc_sysvia_callbacks_t cb = {
            .sound_write   = sv_sound_write,
            .keyboard_read = sv_keyboard_read,
            .latch_changed = sv_latch_changed,
            .irq           = sv_irq,
            .motor_changed = sv_motor_changed,
            .user_ctx      = m,
        };
        bbc_sysvia_init(&m->sysvia, &cb);
    }

    /* ----- Tape (ACIA / Serial ULA) --------------------------------- */
    bbc_tape_init(&m->tape);
    bbc_tape_set_irq_cb(&m->tape, tape_irq, m);

    /* ----- SN76489 PSG --------------------------------------------- */
    sn76489_init(&m->psg, 22050);

    /* ----- Video ---------------------------------------------------- */
    {
        uint8_t *ram = bbc_memory_get_ram(m->mem);
        bbc_video_init(&m->video, ram, BBC_MEMORY_RAM_SIZE);
        bbc_video_set_vsync_callback(&m->video, video_vsync_cb, m);
    }

    /* ----- Wire I/O callbacks into bbc_memory ----------------------- */

    /* CRTC MC6845: &FE00–&FE01 */
    bbc_memory_set_range_callbacks(m->mem, 0xFE00, 2,
        io_crtc_read, io_crtc_write, m);

    /* Video ULA: &FE20–&FE21 */
    bbc_memory_set_range_callbacks(m->mem, 0xFE20, 2,
        io_vidproc_read, io_vidproc_write, m);

    /* System VIA: &FE40–&FE4F (16 registers, mirrored every 16 bytes) */
    bbc_memory_set_range_callbacks(m->mem, 0xFE40, 16,
        io_sysvia_read, io_sysvia_write, m);

    /* MC6850 ACIA: &FE08 (status/ctrl) &FE09 (data) */
    bbc_memory_set_range_callbacks(m->mem, 0xFE08, 2,
        io_acia_read, io_acia_write, m);

    /* Serial ULA control: &FE10 (write-only).
     * bit 0 = RS423/cassette select (0=cassette, 1=RS423) — NOT motor relay.
     * bits 2:1 = TX baud rate select
     * bits 4:3 = RX baud rate select
     * Motor relay is controlled by System VIA CB2 only (sv_motor_changed). */
    bbc_memory_set_write_callback(m->mem, 0xFE10, io_serial_ula_write, m);
}

/* ======================================================================
 * bbc_machine_reset
 * ====================================================================== */

void bbc_machine_reset(bbc_machine_t *m) {
    bbc_sysvia_reset(&m->sysvia);
    sn76489_reset(&m->psg);
    bbc_video_reset(&m->video);

    m->irq.sysvia  = false;
    m->irq.acia    = false;
    m->sysvia_acc  = 0;
    m->crtc_acc    = 0;

    if (m->fb_output) {
        bbc_video_set_output(&m->video, m->fb_output);
    }
    if (m->on_frame) {
        bbc_video_set_frame_callback(&m->video, m->on_frame, m->on_frame_ctx);
    }

    bbc_cpu_reset(m->cpu);
}

/* ======================================================================
 * bbc_machine_break
 *
 * Soft BREAK: reset CPU + peripherals without clearing RAM.
 *
 * The OS reads $0258 to distinguish BREAK-key modifiers. Preserve RAM,
 * record whether Shift is held, and reset the installed hardware.
 * ====================================================================== */

void bbc_machine_break(bbc_machine_t *m, bool shift_held) {
    uint8_t *ram = bbc_memory_get_ram(m->mem);

    /* $0258: BBC OS BREAK-key state — bit 1 indicates Shift. */
    if (ram) {
        ram[0x0258] = shift_held ? 0x02 : 0x00;
    }

    /* Reset peripherals (preserves RAM) */
    bbc_sysvia_reset(&m->sysvia);
    sn76489_reset(&m->psg);
    bbc_video_reset(&m->video);

    m->irq.sysvia  = false;
    m->irq.acia    = false;
    m->sysvia_acc  = 0;
    m->crtc_acc    = 0;

    if (m->fb_output) {
        bbc_video_set_output(&m->video, m->fb_output);
    }
    if (m->on_frame) {
        bbc_video_set_frame_callback(&m->video, m->on_frame, m->on_frame_ctx);
    }

    bbc_cpu_reset(m->cpu);
}

/* ======================================================================
 * bbc_machine_step
 * ====================================================================== */

int bbc_machine_step(bbc_machine_t *m) {
#if defined(BBC_MACHINE_PROFILE)
    /* Muestreo pseudoaleatorio: aproximadamente 1 de cada 64 pasos. */
    mp_rng = mp_rng * 1664525u + 1013904223u;
    const int mp_sample = (mp_rng < 0x04000000u);

    uint32_t mp_start = 0;
    uint32_t mp_end = 0;
    uint32_t mp_whole_start = 0;

    if (mp_sample) {
        mp_whole_start = machine_profile_ccount();
        mp_start = mp_whole_start;
    }
#endif

    int cycles = bbc_cpu_step(m->cpu);

#if defined(BBC_MACHINE_PROFILE)
    if (mp_sample) {
        mp_end = machine_profile_ccount();
        mp_cpu += (uint32_t)(mp_end - mp_start);
        mp_start = mp_end;
    }
#endif

    if (cycles <= 0) cycles = 1;

    /* The VIA is clocked at 1 MHz while the CPU is nominally 2 MHz.
     * Instruction-granularity CPU stepping makes this an approximation,
     * not bus-cycle or wait-state accurate. */
    m->sysvia_acc += cycles;
    int32_t sysvia_ticks = m->sysvia_acc / 2;
    m->sysvia_acc %= 2;
    if (sysvia_ticks > 0)
        bbc_sysvia_tick(&m->sysvia, sysvia_ticks);

#if defined(BBC_MACHINE_PROFILE)
    if (mp_sample) {
        mp_end = machine_profile_ccount();
        mp_sysvia += (uint32_t)(mp_end - mp_start);
        mp_start = mp_end;
    }
#endif

    bbc_tape_tick   (&m->tape,    cycles);

#if defined(BBC_MACHINE_PROFILE)
    if (mp_sample) {
        mp_end = machine_profile_ccount();
        mp_tape += (uint32_t)(mp_end - mp_start);
        mp_start = mp_end;
    }
#endif

    /* The ULA selects 1 MHz (CPU/2) or 2 MHz (one tick per CPU cycle).
     * Carry the residual cycles across instructions and clock changes. */

#if defined(BBC_MACHINE_PROFILE)
    if (mp_sample)
        mp_start = machine_profile_ccount();
#endif

    const int32_t crtc_divisor = m->video.ula.crtc_2mhz ? 1 : 2;
    m->crtc_acc += cycles;
    while (m->crtc_acc >= crtc_divisor) {
        bbc_video_tick(&m->video);
        m->crtc_acc -= crtc_divisor;
    }

#if defined(BBC_MACHINE_PROFILE)
    if (mp_sample) {
        mp_end = machine_profile_ccount();
        mp_video += (uint32_t)(mp_end - mp_start);
        mp_total += (uint32_t)(mp_end - mp_whole_start);
        if (++mp_samples >= 4096U) {
            fprintf(stderr,
                    "[machine-prof] avg CC/step"
                    " TOTAL=%llu CPU=%llu SYSVIA=%llu"
                    " TAPE=%llu VIDEO=%llu n=%u\n",
                    (unsigned long long)(mp_total / mp_samples),
                    (unsigned long long)(mp_cpu / mp_samples),
                    (unsigned long long)(mp_sysvia / mp_samples),
                    (unsigned long long)(mp_tape / mp_samples),
                    (unsigned long long)(mp_video / mp_samples),
                    (unsigned)mp_samples);
            mp_cpu = mp_sysvia = 0;
            mp_tape = mp_video = 0;
            mp_samples = 0;
            mp_total = 0;
        }
    }
#endif

    /* SN76489 ticking is handled by sn76489_audio_push() / sn76489_render() */

    return cycles;
}

/* ======================================================================
 * bbc_machine_key_event
 * ====================================================================== */

void bbc_machine_key_event(bbc_machine_t *m, uint8_t row, uint8_t col, bool pressed) {
    if (row < BBC_KB_ROWS && col < BBC_KB_COLS) {
        m->keyboard.pressed[row][col] = pressed;
        /* Notify sysvia immediately so CA2 ("any key") is updated.
         * Without this, the MOS keyboard scanner never wakes up. */
        bbc_sysvia_keyboard_updated(&m->sysvia);
    }
}

/* ======================================================================
 * bbc_machine_set_video_output
 * ====================================================================== */

void bbc_machine_set_video_output(bbc_machine_t *m, bbc_video_output_t *out) {
    m->fb_output = out;
    if (out) {
        bbc_video_set_output(&m->video, out);
    }
}

/* ======================================================================
 * bbc_machine_set_frame_callback
 * ====================================================================== */

void bbc_machine_set_frame_callback(bbc_machine_t *m,
                                     void (*cb)(void *ctx), void *ctx) {
    m->on_frame     = cb;
    m->on_frame_ctx = ctx;
    bbc_video_set_frame_callback(&m->video, cb, ctx);
}

/* ======================================================================
 * System VIA callbacks
 * ====================================================================== */

static void sv_latch_changed(void *ctx, uint8_t latch_bits) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    bbc_video_set_scroll_control(
        &m->video,
        (latch_bits & (1u << BBC_LATCH_VIDEO_C0)) != 0,
        (latch_bits & (1u << BBC_LATCH_VIDEO_C1)) != 0);
}

static void sv_sound_write(void *ctx, uint8_t data) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    sn76489_write(&m->psg, data);
}

static bool sv_keyboard_read(void *ctx, uint8_t row, uint8_t col) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    /*
     * BBC keyboard scan: MOS writes (row<<4)|col to Port A, then reads
     * bit 7 — LOW means key pressed (active low on real hardware).
     * We return true if the key at (row, col) is pressed.
     */
    if (row < BBC_KB_ROWS && col < BBC_KB_COLS)
        return m->keyboard.pressed[row][col];
    return false;
}

static void sv_irq(void *ctx, bool state) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    m->irq.sysvia = state;
    update_irq(m);
}

/* ======================================================================
 * Video VSYNC callback
 * ====================================================================== */

static void video_vsync_cb(void *ctx, bool state) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    /*
     * VSYNC → System VIA CA1.  CA1 is configured for negative or positive
     * edge by PCR bit 0; bbc_sysvia_vsync() handles both edges.
     */
    bbc_sysvia_vsync(&m->sysvia, state);
}

/* ======================================================================
 * bbc_memory I/O callbacks
 * ====================================================================== */

/* CRTC MC6845 — &FE00 (address register), &FE01 (data register) */
static uint8_t io_crtc_read(uint16_t addr, void *ctx) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    return bbc_video_crtc_read(&m->video, (uint8_t)(addr & 1));
}
static void io_crtc_write(uint16_t addr, uint8_t val, void *ctx) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    bbc_video_crtc_write(&m->video, (uint8_t)(addr & 1), val);
}

/* Video ULA — &FE20 (control), &FE21 (palette) */
static uint8_t io_vidproc_read(uint16_t addr, void *ctx) {
    /* Video ULA registers are write-only; reads return 0xFF */
    (void)addr; (void)ctx;
    return 0xFF;
}
static void io_vidproc_write(uint16_t addr, uint8_t val, void *ctx) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    bbc_video_vidproc_write(&m->video, (uint8_t)(addr & 1), val);
}

/* System VIA IC3 — &FE40–&FE4F */
static uint8_t io_sysvia_read(uint16_t addr, void *ctx) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    return bbc_sysvia_read(&m->sysvia, (uint8_t)(addr & 0x0F));
}
static void io_sysvia_write(uint16_t addr, uint8_t val, void *ctx) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    bbc_sysvia_write(&m->sysvia, (uint8_t)(addr & 0x0F), val);
}

/* ======================================================================
 * Serial ULA / ACIA — &FE08 (status/control) and &FE09 (data)
 * ====================================================================== */

static uint8_t io_acia_read(uint16_t addr, void *ctx) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    return bbc_tape_read(&m->tape, (uint8_t)(addr & 1));
}

static void io_acia_write(uint16_t addr, uint8_t val, void *ctx) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    bbc_tape_write(&m->tape, (uint8_t)(addr & 1), val);
}

/* ======================================================================
 * Serial ULA control register — &FE10 (write only)
 *
 * BBC Micro Serial ULA (IC57):
 *   bit 0   = RS423/cassette select (0=cassette, 1=RS423)
 *   bits 2:1 = transmit baud rate select
 *   bits 4:3 = receive baud rate select
 *   bits 7:5 = other (RS423 control, ignored here)
 *
 * NOTE: the tape motor relay is controlled by System VIA CB2, NOT here.
 * ====================================================================== */

static void io_serial_ula_write(uint16_t addr, uint8_t val, void *ctx) {
    (void)addr;
    (void)ctx;
    /* Baud rate and RS423/cassette select — not yet used, log only */
    (void)val; /* baud rate / RS423 select — not yet used */
}

/* ======================================================================
 * Tape IRQ — routes ACIA interrupt to CPU IRQ
 * ====================================================================== */

static void tape_irq(void *ctx, bool state) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    /* ACIA IRQ shares /IRQ line with VIA IRQs.
     * Track state so update_irq() can properly deassert the line
     * when no source is active (fixes: VIA clearing IRQ cancelled ACIA). */
    m->irq.acia = state;
    update_irq(m);
}

/* ======================================================================
 * Tape motor — called from sysvia CB2 output
 * ====================================================================== */

static void sv_motor_changed(void *ctx, bool on) {
    bbc_machine_t *m = (bbc_machine_t *)ctx;
    bbc_tape_set_motor(&m->tape, on);
}

/* ======================================================================
 * bbc_machine_mount_tape
 * ====================================================================== */

int bbc_machine_mount_tape(bbc_machine_t *m, const char *uef_path) {
    if (!uef_path) {
        bbc_tape_free(&m->tape);
        return 0;
    }
    return bbc_tape_load_uef(&m->tape, uef_path);
}

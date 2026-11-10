/*
 * main.cpp — ESP32 BBC Micro emulator entry point
 *
 * Hardware target: TTGO VGA32 v1.4 (ESP32-PICO-D4, 4 MB Flash + PSRAM)
 * Pin assignments: see board.h
 *
 * Display: FabGL VGADirectController (64-colour, 8 GPIOs)
 *   - The render task publishes double-buffered compact frames in SRAM.
 *   - The ISR expands these into FabGL's DMA scan-line buffer.
 *
 * Keyboard: FabGL PS2Controller → BBC Micro keyboard matrix
 *   - PS/2 on GPIO 33 (CLK) / 32 (DATA) per board.h
 *   - VirtualKey → (row, col) mapping for the BBC keyboard matrix
 *   - keyboardTask polls getNextVirtualKey() and calls bbc_machine_key_event()
 *
 * Task layout:
 *   Core 0  emulatorTask  — BBC Micro main loop (CPU + peripherals)
 *   Core 1  (FabGL ISR)   — VGA scanline DMA (runs autonomously)
 *   Core 1  audioTask     — SN76489 audio rendering
 *   Core 1  keyboardTask  — PS/2 → BBC matrix events
 */
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

#include "fabgl.h"
#include "esp32-hal-psram.h"

#include "board.h"
#include "bbc_machine.h"
#include "sn76489.h"
#include "roms.h"
#include "emu_scheduler.h"
#include "default_tape.h"
#include "vga_bitmap.h"

#ifndef BBC_REALTIME_SCHEDULER
#define BBC_REALTIME_SCHEDULER 1
#endif

/* -----------------------------------------------------------------------
 * VGA / BBC output width
 * ----------------------------------------------------------------------- */
/* BBC content renders at 640 pixels wide (1:1 with VGA 640×480).
 * Teletext (Mode 7): 40 cols × 12 px = 480 px, centred with 80 px border.
 * Bitmap modes: up to 640 px at 2 MHz clock — fits exactly. */
#define BBC_BUF_W   640     /* pixels per VGA line / BBC row */

/* Video diagnostics (counters only, no printf in the ISR).  Set to 0 to
 * compile all of it out.  Written by the VGA ISR (Core 1) / render and
 * emulator tasks (Core 0), read by the 1 Hz report in videoRenderTask;
 * races are benign (statistics only). */
#ifndef VIDEO_DIAG
#define VIDEO_DIAG 0
#endif

static volatile uint32_t s_vga_cb_calls = 0;
static volatile uint32_t s_vga_line0_calls = 0;
static volatile uint32_t s_vga_swap_calls = 0;
static volatile int s_vga_last_line = -1;
static volatile int s_vga_max_line = -1;

#if VIDEO_DIAG
static volatile uint32_t s_vga_wraps = 0;        /* scanLine < previous      */
static volatile int      s_vga_wrap_prev_min = 100000; /* line before a wrap  */
static volatile int      s_vga_wrap_prev_max = -1;
static volatile uint32_t s_vga_skips = 0;        /* calls with delta > 1     */
static volatile int      s_vga_max_gap = 0;      /* largest forward delta    */
static volatile uint32_t s_vga_isr_cyc_sum = 0;  /* modular, use deltas      */
static volatile uint32_t s_vga_isr_cyc_max = 0;  /* reset by the reporter    */
static volatile uint32_t s_vid_published = 0;    /* frames published         */
static volatile uint32_t s_vid_render_us_sum = 0;
static volatile uint32_t s_vid_render_us_max = 0;
static volatile uint32_t s_emu_busy_us = 0;      /* time spent executing CPU */
static volatile int s_vga_src_at_last = -1;
static volatile uint8_t s_vga_display_at_last = 0xFF;
static volatile unsigned s_vga_red_pixels_at_last = 0;

static inline uint32_t IRAM_ATTR diag_ccount(void)
{
    uint32_t c;
    __asm__ __volatile__("rsr %0, ccount" : "=a"(c));
    return c;
}
#endif

/* -----------------------------------------------------------------------
 * BBC colour palette (index 0-7 → RGB888)
 * bit0=R, bit1=G, bit2=B (VideoULA convention)
 * ----------------------------------------------------------------------- */
static const fabgl::RGB888 s_bbc_palette[8] = {
    {   0,   0,   0 },   /* 0 Black   */
    { 255,   0,   0 },   /* 1 Red     */
    {   0, 255,   0 },   /* 2 Green   */
    { 255, 255,   0 },   /* 3 Yellow  */
    {   0,   0, 255 },   /* 4 Blue    */
    { 255,   0, 255 },   /* 5 Magenta */
    {   0, 255, 255 },   /* 6 Cyan    */
    { 255, 255, 255 },   /* 7 White   */
};

/* -----------------------------------------------------------------------
 * BBC Micro keyboard matrix mapping
 *
 * Use the machine's native 8-row x 10-column coordinates directly.
 * This follows the matrix used by src/host/main_sdl.c:
 *   row 1: Q 3 4 5 F4 8 F7 - ^ LEFT
 *   row 2: F0 W E T 7 I 9 0 £ DOWN
 *   row 3: 1 2 D R 6 U O P [ UP
 *   row 4: CAPS A X F Y J K @ : RETURN
 *   row 5: SHIFTLK S C G H N L ; ] DELETE
 *   row 6: TAB Z SPACE V B M , . / COPY
 *   row 7: ESC F1 F2 F3 F5 F6 F8 F9 \\ RIGHT
 *
 * Coordinates returned here are passed unchanged to bbc_machine_key_event().
 * ----------------------------------------------------------------------- */
struct BbcKeyPos { uint8_t row; uint8_t col; };

static BbcKeyPos vk_to_bbc(fabgl::VirtualKey vk)
{
    using namespace fabgl;
    switch (vk) {
    /* Row 0 */
    case VK_LSHIFT: case VK_RSHIFT:                    return {0, 0};
    case VK_LCTRL:  case VK_RCTRL:                     return {0, 1};

    /* Row 1 */
    case VK_q: case VK_Q:                             return {1, 0};
    case VK_3:                                        return {1, 1};
    case VK_4:                                        return {1, 2};
    case VK_5:                                        return {1, 3};
    case VK_F4:                                       return {1, 4};
    case VK_8:                                        return {1, 5};
    case VK_F7:                                       return {1, 6};
    case VK_MINUS:                                    return {1, 7};
    case VK_EQUALS: case VK_CARET: case VK_TILDE:     return {1, 8};
    case VK_LEFT: case VK_KP_LEFT:                    return {1, 9};

    /* Row 2 */
    case VK_F10:                                      return {2, 0}; /* BBC F0 */
    case VK_w: case VK_W:                             return {2, 1};
    case VK_e: case VK_E:                             return {2, 2};
    case VK_t: case VK_T:                             return {2, 3};
    case VK_7:                                        return {2, 4};
    case VK_i: case VK_I:                             return {2, 5};
    case VK_9:                                        return {2, 6};
    case VK_0:                                        return {2, 7};
    case VK_POUND:                                    return {2, 8};
    case VK_DOWN: case VK_KP_DOWN:                    return {2, 9};

    /* Row 3 */
    case VK_1:                                        return {3, 0};
    case VK_2:                                        return {3, 1};
    case VK_d: case VK_D:                             return {3, 2};
    case VK_r: case VK_R:                             return {3, 3};
    case VK_6:                                        return {3, 4};
    case VK_u: case VK_U:                             return {3, 5};
    case VK_o: case VK_O:                             return {3, 6};
    case VK_p: case VK_P:                             return {3, 7};
    case VK_LEFTBRACKET: case VK_LEFTBRACE:           return {3, 8};
    case VK_UP: case VK_KP_UP:                        return {3, 9};

    /* Row 4 */
    case VK_CAPSLOCK:                                 return {4, 0};
    case VK_a: case VK_A:                             return {4, 1};
    case VK_x: case VK_X:                             return {4, 2};
    case VK_f: case VK_F:                             return {4, 3};
    case VK_y: case VK_Y:                             return {4, 4};
    case VK_j: case VK_J:                             return {4, 5};
    case VK_k: case VK_K:                             return {4, 6};
    case VK_AT: case VK_GRAVEACCENT:                  return {4, 7};
    case VK_COLON: case VK_QUOTE: case VK_ASTERISK:    return {4, 8};
    case VK_RETURN: case VK_KP_ENTER:                 return {4, 9};

    /* Row 5 */
    case VK_SCROLLLOCK:                               return {5, 0}; /* Shift Lock */
    case VK_s: case VK_S:                             return {5, 1};
    case VK_c: case VK_C:                             return {5, 2};
    case VK_g: case VK_G:                             return {5, 3};
    case VK_h: case VK_H:                             return {5, 4};
    case VK_n: case VK_N:                             return {5, 5};
    case VK_l: case VK_L:                             return {5, 6};
    case VK_SEMICOLON: case VK_PLUS:                  return {5, 7};
    case VK_RIGHTBRACKET: case VK_RIGHTBRACE:         return {5, 8};
    case VK_DELETE: case VK_BACKSPACE:                return {5, 9};

    /* Row 6 */
    case VK_TAB:                                      return {6, 0};
    case VK_z: case VK_Z:                             return {6, 1};
    case VK_SPACE:                                    return {6, 2};
    case VK_v: case VK_V:                             return {6, 3};
    case VK_b: case VK_B:                             return {6, 4};
    case VK_m: case VK_M:                             return {6, 5};
    case VK_COMMA: case VK_LESS:                      return {6, 6};
    case VK_PERIOD: case VK_GREATER:                  return {6, 7};
    case VK_SLASH: case VK_QUESTION:                  return {6, 8};
    case VK_END: case VK_KP_END:                      return {6, 9}; /* COPY */

    /* Row 7 */
    case VK_ESCAPE:                                   return {7, 0};
    case VK_F1:                                       return {7, 1};
    case VK_F2:                                       return {7, 2};
    case VK_F3:                                       return {7, 3};
    case VK_F5:                                       return {7, 4};
    case VK_F6:                                       return {7, 5};
    case VK_F8:                                       return {7, 6};
    case VK_F9:                                       return {7, 7};
    case VK_BACKSLASH:                                return {7, 8};
    case VK_RIGHT: case VK_KP_RIGHT:                  return {7, 9};

    /* Shifted symbols that FabGL reports as distinct virtual keys. */
    case VK_EXCLAIM:                                  return {3, 0}; /* Shift+1 */
    case VK_HASH:                                     return {1, 1}; /* Shift+3 */
    case VK_DOLLAR:                                   return {1, 2}; /* Shift+4 */
    case VK_PERCENT:                                  return {1, 3}; /* Shift+5 */
    case VK_AMPERSAND:                                return {3, 4}; /* Shift+6 */
    case VK_QUOTEDBL:                                 return {3, 1}; /* Shift+2 */
    case VK_LEFTPAREN:                                return {1, 5}; /* Shift+8 */
    case VK_RIGHTPAREN:                               return {2, 6}; /* Shift+9 */
    default:                                          return {0xFF, 0xFF};
    }
}

/* -----------------------------------------------------------------------
 * Global objects
 * ----------------------------------------------------------------------- */
static fabgl::VGADirectController s_vga;
static fabgl::PS2Controller       s_ps2;

#define VGA_OUT_HEIGHT 480

// Mode 7: mantener sincronizado con BBC_TT_LINES_PER_ROW=10
// en components/bbc_video/bbc_video.c.
#define VGA_MODE7_CONTENT_W 480
#define VGA_MODE7_LINES_PER_ROW 10
#define VGA_MODE7_CONTENT_H (25 * VGA_MODE7_LINES_PER_ROW)
#define VGA_MODE7_X ((BBC_BUF_W - VGA_MODE7_CONTENT_W) / 2)
#define VGA_MODE7_Y ((VGA_OUT_HEIGHT - VGA_MODE7_CONTENT_H) / 2 - 20)
#define VGA_MODE7_PACKED_STRIDE (VGA_MODE7_CONTENT_W / 2)
#define VGA_MODE7_PACKED_BYTES \
    (VGA_MODE7_PACKED_STRIDE * VGA_MODE7_CONTENT_H)
#define VGA_BITMAP_PACKED_BYTES \
    (BBC_BITMAP_PACKED_STRIDE * BBC_BITMAP_PACKED_HEIGHT)
#define VGA_PACKED_BYTES \
    ((VGA_BITMAP_PACKED_BYTES > VGA_MODE7_PACKED_BYTES) ? \
     VGA_BITMAP_PACKED_BYTES : VGA_MODE7_PACKED_BYTES)

static uint8_t *s_packed_fb[2] = { nullptr, nullptr };

struct vga_sig_pair_t {
    uint8_t first;
    uint8_t second;
};

static vga_sig_pair_t s_sig_pair[256];

static volatile bool s_fb_mode7[2] = { false, false };

static uint8_t *s_render_ram = nullptr;
static bbc_video_t s_render_video;

static uint8_t s_fb_front = 0;
static uint8_t s_fb_pending = 1;
static uint8_t s_fb_display = 0;
static bool s_fb_ready = false;

static portMUX_TYPE s_fb_mux = portMUX_INITIALIZER_UNLOCKED;

/* Pre-computed 8bpp VGA signal bytes for each BBC colour index. */
static uint8_t s_sig[8];
static uint16_t s_bitmap_pairs[64];

/* BBC machine state — prefer internal SRAM for frequently accessed devices. */
static bbc_machine_t *machine = nullptr;

static void mountDefaultTape(bbc_machine_t *m)
{
    if (!fabgl::FileBrowser::mountSDCard(false, BOARD_SD_MOUNT, 4, 16384,
                                         BOARD_SD_MISO, BOARD_SD_MOSI,
                                         BOARD_SD_CLK, BOARD_SD_CS)) {
        printf("[tape] SD mount failed; starting without tape\n");
        return;
    }

    char path[512];
    default_tape_result_t result = default_tape_path(BOARD_SD_MOUNT, path,
                                                     sizeof(path));
    if (result != DEFAULT_TAPE_OK) {
        const char *reason = result == DEFAULT_TAPE_NO_FILE
            ? "tape.uef not found" : result == DEFAULT_TAPE_IO_ERROR
            ? "cannot access tape.uef" : "tape.uef path too long";
        printf("[tape] %s; starting without tape\n", reason);
    } else if (bbc_machine_mount_tape(m, path) != 0) {
        printf("[tape] cannot mount %s; starting without tape\n", path);
    } else {
        printf("[tape] mounted %s\n", path);
    }
    fabgl::FileBrowser::unmountSDCard();
}

/* -----------------------------------------------------------------------
 * VGADirectController draw-scanline callback (IRAM, Core 1 ISR context)
 * ----------------------------------------------------------------------- */
/* VGA is 640×480.  BBC native output is 640×256 (1:1 horizontally).
 * Bitmap output is scaled into the vertical viewport with black margins.
 *
 * The render task snapshots video state and RAM outside the ISR. Completed
 * frames are swapped only at a VGA frame boundary.
 *
 * I2S LCD serialises 32-bit words as [byte2, byte3, byte0, byte1].
 * Writing dest[x ^ 2] compensates so the output pin order is [0,1,2,3].
 * x_offset=80 (teletext border) is a multiple of 4 — word-aligned.
 */
static void IRAM_ATTR draw_scanline_impl(
    void *arg, uint8_t *dest, int scanLine)
{
    (void)arg;

    // Detectar el inicio de un nuevo fotograma mediante el retroceso
    // del contador de líneas VGA.
    static int previousScanLine = -1;

    int previous = previousScanLine;
    previousScanLine = scanLine;

    if (previous >= 0 && scanLine < previous) {
        portENTER_CRITICAL_ISR(&s_fb_mux);

        if (s_fb_ready) {
            s_fb_front = s_fb_pending;
            s_fb_ready = false;
            s_vga_swap_calls++;
        }

        s_fb_display = s_fb_front;

        portEXIT_CRITICAL_ISR(&s_fb_mux);
    }

    const uint8_t display = s_fb_display;

    // Comprobaciones de seguridad.
    if (scanLine < 0 ||
        scanLine >= VGA_OUT_HEIGHT ||
        !s_packed_fb[display]) {
        memset(dest, s_sig[0], BBC_BUF_W);
        return;
    }

    /*
     * MODE 7: leer el framebuffer compacto en RAM interna.
     *
     * Dos píxeles por byte en el búfer empaquetado.
     * El contenido se convierte a señal VGA y se reordena
     * para respetar el orden de bytes del periférico I2S.
     */
    if (s_fb_mode7[display]) {

        if (scanLine < VGA_MODE7_Y ||
            scanLine >= VGA_MODE7_Y + VGA_MODE7_CONTENT_H) {
            memset(dest, s_sig[0], BBC_BUF_W);
            return;
        }

        const uint8_t *packed =
            s_packed_fb[display] +
            (size_t)(scanLine - VGA_MODE7_Y) *
            VGA_MODE7_PACKED_STRIDE;

        // Margen izquierdo negro.
        memset(dest, s_sig[0], VGA_MODE7_X);

        // Generar cuatro píxeles por iteración.
        for (int x = 0; x < VGA_MODE7_CONTENT_W; x += 4) {

            // Dos primeros píxeles empaquetados.
            const uint8_t p01 = packed[x / 2];

            // Dos siguientes píxeles empaquetados.
            const uint8_t p23 = packed[x / 2 + 1];

            const vga_sig_pair_t &a = s_sig_pair[p23];
            const vga_sig_pair_t &b = s_sig_pair[p01];

            // Mantener la permutación original de bytes I2S.
            dest[VGA_MODE7_X + x]     = a.first;
            dest[VGA_MODE7_X + x + 1] = a.second;
            dest[VGA_MODE7_X + x + 2] = b.first;
            dest[VGA_MODE7_X + x + 3] = b.second;
        }

        // Margen derecho negro.
        memset(
            dest + VGA_MODE7_X + VGA_MODE7_CONTENT_W,
            s_sig[0],
            BBC_BUF_W - VGA_MODE7_X - VGA_MODE7_CONTENT_W
        );

        return;
    }

    const int32_t source_y = vga_bitmap_viewport_source_y(
        (uint32_t)scanLine, BBC_BITMAP_PACKED_HEIGHT, VGA_OUT_HEIGHT);
    if (source_y < 0) {
        memset(dest, s_sig[0], BBC_BUF_W);
        return;
    }

    const uint8_t *src = s_packed_fb[display] +
        (size_t)source_y * BBC_BITMAP_PACKED_STRIDE;
    for (int x = 0; x < BBC_BUF_W; x += 8) {
        vga_bitmap_expand8((uint16_t *)(dest + x), src, s_bitmap_pairs);
        src += 3;
    }
}

#if VIDEO_DIAG
/* Wrapper used only for measurement: it observes the sequence of scanLine
 * values and the time spent in draw_scanline_impl(); it does not change what
 * is drawn. */
static void IRAM_ATTR draw_scanline(
    void *arg, uint8_t *dest, int scanLine)
{
    static int prev = -1;
    const uint32_t t0 = diag_ccount();

    draw_scanline_impl(arg, dest, scanLine);

    const uint32_t dt = diag_ccount() - t0;
    if (scanLine == VGA_OUT_HEIGHT - (int)VGA_BITMAP_MARGIN_Y - 1) {
        s_vga_src_at_last = (int)vga_bitmap_viewport_source_y(
            (uint32_t)scanLine, BBC_BITMAP_PACKED_HEIGHT, VGA_OUT_HEIGHT);
        s_vga_display_at_last = s_fb_display;
        unsigned red_pixels = 0;
        for (int x = 0; x < BBC_BUF_W; ++x)
            red_pixels += dest[x ^ 2] == s_sig[1];
        s_vga_red_pixels_at_last = red_pixels;
    }

    s_vga_cb_calls = s_vga_cb_calls + 1;
    if (scanLine == 0) s_vga_line0_calls = s_vga_line0_calls + 1;
    s_vga_last_line = scanLine;
    if (scanLine > s_vga_max_line) s_vga_max_line = scanLine;

    if (prev >= 0) {
        if (scanLine < prev) {
            s_vga_wraps = s_vga_wraps + 1;
            if (prev < s_vga_wrap_prev_min) s_vga_wrap_prev_min = prev;
            if (prev > s_vga_wrap_prev_max) s_vga_wrap_prev_max = prev;
        } else if (scanLine - prev > 1) {
            s_vga_skips = s_vga_skips + 1;
            if (scanLine - prev > s_vga_max_gap)
                s_vga_max_gap = scanLine - prev;
        }
    }
    prev = scanLine;

    s_vga_isr_cyc_sum = s_vga_isr_cyc_sum + dt;
    if (dt > s_vga_isr_cyc_max) s_vga_isr_cyc_max = dt;
}
#else
#define draw_scanline draw_scanline_impl
#endif

/* -----------------------------------------------------------------------
 * Keyboard task
 * ----------------------------------------------------------------------- */
static void keyboardTask(void *arg)
{
    (void)arg;

    fabgl::Keyboard *kb = s_ps2.keyboard();
    if (!kb) {
        printf("[kbd] no keyboard object — task exiting\n");
        vTaskDelete(NULL);
        return;
    }

    printf("[kbd] PS/2 keyboard task started\n");

    while (true) {
        fabgl::VirtualKeyItem item;
        if (kb->getNextVirtualKey(&item, 10)) {
            BbcKeyPos pos = vk_to_bbc(item.vk);

            printf("[kbd] vk=%d down=%d ascii=%d matrix=(%u,%u)\n",
                    (int)item.vk,
                    item.down ? 1 : 0,
                    (int)kb->virtualKeyToASCII(item.vk),
                    (unsigned)pos.row,
                    (unsigned)pos.col);

            if (!machine)
                continue;

            if (pos.row == 0xFF) {
                printf("[kbd] tecla sin asignar: vk=%d\n", (int)item.vk);
                continue;
            }

            bbc_machine_key_event(machine, pos.row, pos.col, item.down);
        }
    }
}

/* -----------------------------------------------------------------------
 * Audio task
 * ----------------------------------------------------------------------- */
static void audioTask(void *arg)
{
    (void)arg;

    if (sn76489_audio_init() != ESP_OK) {
        printf("[audio] audio init failed\n");
        vTaskDelete(NULL);
        return;
    }

    printf("[audio] audio started at %lu Hz\n",
           (unsigned long)sn76489_audio_sample_rate());

    /* Each buffer is 256 samples @ 22050 Hz ≈ 11.6 ms.  Delay after each
     * push so IDLE1 always gets CPU time regardless of whether
     * xQueueReceive blocked or returned immediately. */
    while (true) {
        if (machine)
            sn76489_audio_push(&machine->psg);
        vTaskDelay(pdMS_TO_TICKS(11));
    }
}

/* -----------------------------------------------------------------------
 * VIA tick statistics (only with -DM6522_STATS; counters live in m6522.c and
 * add up both VIAs) and a build tag so every log can be tied to its variant.
 * ----------------------------------------------------------------------- */
#ifdef M6522_STATS
extern "C" {
extern uint32_t m6522_stat_fast_cycles;
extern uint32_t m6522_stat_slow_cycles;
extern volatile bool m6522_fast_enable;   /* runtime A/B switch (patch 07) */
}
#endif

#define EMU_STR2(x) #x
#define EMU_STR(x)  EMU_STR2(x)

static void printBuildTag(void)
{
#ifdef M6522_FAST_TICK
    const char *fast = EMU_STR(M6522_FAST_TICK);
#else
    const char *fast = "1 (default)";
#endif
#ifdef M6522_STATS
    const char *stats = "on";
#else
    const char *stats = "off";
#endif
    printf("[emu] build %s %s | M6522_FAST_TICK=%s | M6522_STATS=%s | "
           "VIDEO_DIAG=%d\n",
           __DATE__, __TIME__, fast, stats, (int)VIDEO_DIAG);
}

#ifdef M6522_STATS
/* Mirrors the conditions of tick_fast_ok() in m6522.c, one by one, so a log
 * line shows which one keeps a VIA out of the fast path.  Called from
 * emulatorTask itself, i.e. between instructions: the snapshot is consistent. */
static void printViaState(const char *name, const m6522_t *v)
{
    const bool pip_ok  = (v->t1.pip == 0x0003 && v->t2.pip == 0x0003);
    const bool pb6_ok  = !(v->acr & M6522_ACR_T2_COUNT_PB6);
    const bool irq_ok  = (v->irq_pip == 0) &&
                         ((v->ifr & v->ier & 0x7F) == 0);
    const bool trig_ok = !(v->pa.c1_triggered | v->pa.c2_triggered |
                           v->pb.c1_triggered | v->pb.c2_triggered);

    printf("[via-state] %s acr=%02X pcr=%02X ifr=%02X ier=%02X irq_pip=%02X "
           "t1.pip=%02X t2.pip=%02X t1=%04X t2=%04X t2bit=%d "
           "| ok: pip=%d pb6=%d irq=%d trig=%d\n",
           name,
           (unsigned)v->acr, (unsigned)v->pcr,
           (unsigned)v->ifr, (unsigned)v->ier, (unsigned)v->irq_pip,
           (unsigned)v->t1.pip, (unsigned)v->t2.pip,
           (unsigned)v->t1.counter, (unsigned)v->t2.counter,
           (int)v->t2.t_bit,
           (int)pip_ok, (int)pb6_ok, (int)irq_ok, (int)trig_ok);
}

/* Compact signature of what decides the fast path (ACR, PCR, IER and the four
 * conditions).  IFR is left out on purpose: it changes all the time. */
static uint32_t viaSignature(const m6522_t *v)
{
    const uint32_t pip_ok  = (v->t1.pip == 0x0003 && v->t2.pip == 0x0003);
    const uint32_t pb6_ok  = !(v->acr & M6522_ACR_T2_COUNT_PB6);
    const uint32_t irq_ok  = (v->irq_pip == 0) &&
                             ((v->ifr & v->ier & 0x7F) == 0);
    const uint32_t trig_ok = !(v->pa.c1_triggered | v->pa.c2_triggered |
                               v->pb.c1_triggered | v->pb.c2_triggered);
    return pip_ok | (pb6_ok << 1) | (irq_ok << 2) | (trig_ok << 3) |
           ((uint32_t)v->acr << 4) | ((uint32_t)v->ier << 12) |
           ((uint32_t)v->pcr << 20);
}
#endif

/* -----------------------------------------------------------------------
 * Emulator task
 * ----------------------------------------------------------------------- */
static void emulatorTask(void *arg)
{
    (void)arg;

    bbc_machine_reset(machine);
    printf("[emu] BBC Micro reset, running...\n");
    printBuildTag();

#if BBC_REALTIME_SCHEDULER
    emu_scheduler_t scheduler;
    emu_scheduler_init(&scheduler, micros());
#else
    int cycles_budget = 0;
#endif

#ifdef M6522_STATS
    uint32_t via_fast0 = m6522_stat_fast_cycles;
    uint32_t via_slow0 = m6522_stat_slow_cycles;
#endif

    uint32_t speed_cycles = 0;
    uint32_t speed_instructions = 0;
    uint32_t speed_t0 = millis();

    while (true) {
#if BBC_REALTIME_SCHEDULER
        int cycles_budget = emu_scheduler_budget(&scheduler, micros());
#else
        cycles_budget += 2000;
#endif

#if VIDEO_DIAG
        const uint32_t emu_t0 = micros();
#endif

        while (cycles_budget > 0) {
            int c = bbc_machine_step(machine);
            if (c <= 0)
                c = 1;

            speed_cycles += (uint32_t)c;
            speed_instructions++;

            cycles_budget -= c;
#if BBC_REALTIME_SCHEDULER
            emu_scheduler_consume(&scheduler, c);
#endif
        }

#if VIDEO_DIAG
        s_emu_busy_us += micros() - emu_t0;
#endif

#if BBC_REALTIME_SCHEDULER
        if (emu_scheduler_should_yield(&scheduler, micros())) {
            vTaskDelay(1);
            emu_scheduler_yielded(&scheduler, micros());
        }
#else
        vTaskDelay(1);
#endif

        // Calcular el tiempo transcurrido desde la última medición.
        uint32_t now = millis();
        uint32_t elapsed = now - speed_t0;

        if (elapsed >= 1000) {
            uint32_t cycles_per_sec =
                (uint32_t)((uint64_t)speed_cycles * 1000 / elapsed);

            uint32_t instructions_per_sec =
                (uint32_t)((uint64_t)speed_instructions * 1000 / elapsed);

            uint32_t speed_pct =
                (uint32_t)((uint64_t)cycles_per_sec * 100 / 2000000);

            printf("[emu-speed] cycles/s=%lu speed=%lu%% "
                   "instructions/s=%lu window=%lums\n",
                   (unsigned long)cycles_per_sec,
                   (unsigned long)speed_pct,
                   (unsigned long)instructions_per_sec,
                   (unsigned long)elapsed);

#ifdef M6522_STATS
            {
                /* System VIA counters for this window. */
                const uint32_t f = m6522_stat_fast_cycles - via_fast0;
                const uint32_t sl = m6522_stat_slow_cycles - via_slow0;
                via_fast0 = m6522_stat_fast_cycles;
                via_slow0 = m6522_stat_slow_cycles;
                const uint32_t tot = f + sl;

                printf("[via-stats] fastpath=%s fast=%lu slow=%lu fast%%=%lu\n",
                       m6522_fast_enable ? "ON" : "OFF",
                       (unsigned long)f, (unsigned long)sl,
                       (unsigned long)(tot ? (uint64_t)f * 100 / tot : 0));

                /* Send 'f' on the serial console to toggle the VIA fast path
                 * without rebuilding (same binary, same layout). */
                while (Serial.available() > 0) {
                    const int ch = Serial.read();
                    if (ch == 'f' || ch == 'F') {
                        m6522_fast_enable = !m6522_fast_enable;
                        printf("[emu] VIA fast path -> %s\n",
                               m6522_fast_enable ? "ON" : "OFF");
                    }
                }

                /* Print System VIA state on change and periodically. */
                static uint32_t sig_sys = 0xFFFFFFFFu;
                static uint32_t via_state_n = 0;
                const uint32_t ns = viaSignature(&machine->sysvia.via);
                const bool beat = (via_state_n % 30) == 0;
                if (ns != sig_sys || beat) {
                    printViaState("SYS", &machine->sysvia.via);
                    sig_sys = ns;
                }
                via_state_n++;
            }
#endif

            speed_cycles = 0;
            speed_instructions = 0;
            speed_t0 = now;
        }
    }
}

static void videoRenderTask(void *arg)
{
    (void)arg;

    while (!machine || !s_packed_fb[0] ||
            !s_packed_fb[1] || !s_render_ram) {
        vTaskDelay(1);
    }

    for (;;) {
        uint8_t back;
        static uint32_t last_bitmap_diag_ms[2] = {0, 0};

        // No sobrescribir un fotograma que aún está pendiente
        // de ser adoptado por el callback VGA.
        for (;;) {
            bool ready;
            uint8_t front;

            portENTER_CRITICAL(&s_fb_mux);
            ready = s_fb_ready;
            front = s_fb_front;
            portEXIT_CRITICAL(&s_fb_mux);

            vTaskDelay(pdMS_TO_TICKS(20));

            static uint32_t last_diag_ms = 0;
            uint32_t now = millis();

            if (now - last_diag_ms >= 1000) {
                uint8_t front, pending, display;
                bool ready;

                portENTER_CRITICAL(&s_fb_mux);
                front = s_fb_front;
                pending = s_fb_pending;
                display = s_fb_display;
                ready = s_fb_ready;
                portEXIT_CRITICAL(&s_fb_mux);

#if VIDEO_DIAG
                /* Per-window deltas (window ~1 s, dt_ms is the real length). */
                static uint32_t p_cb, p_wraps, p_swaps, p_skips, p_cyc,
                                p_pub, p_rus, p_emu;
                const uint32_t dt_ms = last_diag_ms ? (now - last_diag_ms) : 1000;
                const uint32_t cb    = s_vga_cb_calls   - p_cb;
                const uint32_t wraps = s_vga_wraps      - p_wraps;
                const uint32_t swaps = s_vga_swap_calls - p_swaps;
                const uint32_t skips = s_vga_skips      - p_skips;
                const uint32_t cyc   = s_vga_isr_cyc_sum - p_cyc;
                const uint32_t pub   = s_vid_published  - p_pub;
                const uint32_t rus   = s_vid_render_us_sum - p_rus;
                const uint32_t emu   = s_emu_busy_us    - p_emu;
                p_cb = s_vga_cb_calls;   p_wraps = s_vga_wraps;
                p_swaps = s_vga_swap_calls; p_skips = s_vga_skips;
                p_cyc = s_vga_isr_cyc_sum;  p_pub = s_vid_published;
                p_rus = s_vid_render_us_sum; p_emu = s_emu_busy_us;

                /* Line budget at 640x480@60: ~31.8 us = ~7630 cycles @240MHz. */
                printf("[vdiag] %lums cb/s=%lu wraps/s=%lu swaps/s=%lu "
                       "cb/wrap=%lu skips=%lu maxgap=%d wrapPrev=%d..%d "
                       "isr_avg=%lucyc isr_max=%lucyc | pub/s=%lu "
                       "render_avg=%luus render_max=%luus | emu_busy=%lu%% "
                       "| front=%u pending=%u display=%u ready=%u\n",
                       (unsigned long)dt_ms,
                       (unsigned long)(cb * 1000 / dt_ms),
                       (unsigned long)(wraps * 1000 / dt_ms),
                       (unsigned long)(swaps * 1000 / dt_ms),
                       (unsigned long)(wraps ? cb / wraps : 0),
                       (unsigned long)skips,
                       (int)s_vga_max_gap,
                       (int)s_vga_wrap_prev_min, (int)s_vga_wrap_prev_max,
                       (unsigned long)(cb ? cyc / cb : 0),
                       (unsigned long)s_vga_isr_cyc_max,
                       (unsigned long)(pub * 1000 / dt_ms),
                       (unsigned long)(pub ? rus / pub : 0),
                       (unsigned long)s_vid_render_us_max,
                       (unsigned long)(emu / (dt_ms * 10)),
                       (unsigned)front, (unsigned)pending,
                       (unsigned)display, (unsigned)ready);

                s_vga_isr_cyc_max = 0;
                s_vid_render_us_max = 0;
                s_vga_wrap_prev_min = 100000;
                s_vga_wrap_prev_max = -1;
                s_vga_max_gap = 0;
#else
                printf("[video] swaps=%lu front=%u pending=%u display=%u "
                       "ready=%u\n",
                       (unsigned long)s_vga_swap_calls,
                       (unsigned)front, (unsigned)pending,
                       (unsigned)display, (unsigned)ready);
#endif
                last_diag_ms = now;
            }

            if (!ready) {
                back = front ^ 1;
                break;
            }

            vTaskDelay(1);
        }

        // Instantánea del estado del vídeo.
        s_render_video = machine->video;

        s_render_video.vsync_cb  = nullptr;
        s_render_video.vsync_ctx = nullptr;
        s_render_video.frame_cb  = nullptr;
        s_render_video.frame_ctx = nullptr;

        // Copiar la RAM que verá el renderizador.
        uint32_t n = s_render_video.ram_size;
        if (n > BBC_MEMORY_RAM_SIZE) n = BBC_MEMORY_RAM_SIZE;

        if (s_render_video.system_ram && n) {
            memcpy(s_render_ram, s_render_video.system_ram, n);
        }

        if (n < BBC_MEMORY_RAM_SIZE) {
            memset(s_render_ram + n, 0, BBC_MEMORY_RAM_SIZE - n);
        }

        s_render_video.system_ram = s_render_ram;
        /* Keep physical RAM size for the video address translator. */
        s_render_video.ram_size = n;

        const bool mode7 = s_render_video.ula.teletext_mode;

#if VIDEO_DIAG
        const uint32_t render_t0 = micros();
#endif
        if (mode7) {
            if (!bbc_video_render_mode7_packed(&s_render_video,
                    s_packed_fb[back], VGA_MODE7_PACKED_STRIDE)) {
                printf("[video] Mode 7 packed render failed\n");
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
        } else {
            if (!bbc_video_render_bitmap_packed(&s_render_video,
                    s_packed_fb[back], BBC_BITMAP_PACKED_STRIDE)) {
                printf("[video] bitmap packed render failed\n");
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
        }
        s_fb_mode7[back] = mode7;

#if VIDEO_DIAG
        {
            const uint32_t us = micros() - render_t0;
            s_vid_render_us_sum = s_vid_render_us_sum + us;
            if (us > s_vid_render_us_max) s_vid_render_us_max = us;
            s_vid_published = s_vid_published + 1;

            const uint32_t diag_now = millis();
            if (!mode7 && diag_now - last_bitmap_diag_ms[back] >= 1000) {
                printf("[fbdiag] rendered=%u R6=%u R9=%u start=%04X "
                       "vga%u=src%d/fb%u/red%u\n",
                       (unsigned)back,
                       (unsigned)s_render_video.crtc.v_displayed,
                       (unsigned)s_render_video.crtc.max_scanline_addr,
                       (unsigned)mc6845_get_start_addr(&s_render_video.crtc),
                       (unsigned)(VGA_OUT_HEIGHT - VGA_BITMAP_MARGIN_Y - 1),
                       (int)s_vga_src_at_last,
                       (unsigned)s_vga_display_at_last,
                       s_vga_red_pixels_at_last);
                last_bitmap_diag_ms[back] = diag_now;
            }
        }
#endif

        // Publicar el nuevo fotograma; VGA lo adoptará en scanLine 0.
        portENTER_CRITICAL(&s_fb_mux);
        s_fb_pending = back;
        s_fb_ready = true;
        portEXIT_CRITICAL(&s_fb_mux);
    
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* -----------------------------------------------------------------------
 * setup
 * ----------------------------------------------------------------------- */
void setup(void)
{
    Serial.begin(115200);
    delay(1000);

    /* --- PSRAM diagnostics --------------------------------------------- */
    Serial.printf("[mem] psramFound = %s\n",
            psramFound() ? "true" : "false");
    Serial.printf("[mem] psramSize = %u bytes\n",
            (unsigned int)ESP.getPsramSize());
    Serial.printf("[mem] freePsram = %u bytes\n",
            (unsigned int)ESP.getFreePsram());

    printf("BBC Micro Emulator for ESP32 (PSRAM build)\n");
    printf("[mem] free PSRAM at start: %lu bytes\n",
            (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* --- VGA initialization ------------------------------------------- */
    s_vga.begin(BOARD_VGA_R1,    BOARD_VGA_R0,
            BOARD_VGA_G1,    BOARD_VGA_G0,
            BOARD_VGA_B1,    BOARD_VGA_B0,
            BOARD_VGA_HSYNC, BOARD_VGA_VSYNC);

    s_vga.setDrawScanlineCallback(draw_scanline, nullptr);

    /*
     * setResolution() performs APLL calibration. Temporarily extend the
     * watchdog timeout during this one-time initialization.
     */
    ESP_ERROR_CHECK(esp_task_wdt_init(30, false));

    s_vga.setResolution(VGA_640x480_60Hz, 640, 480);

    printf("[vga] screen=%dx%d viewport=%dx%d row=%d\n",
            s_vga.getScreenWidth(),
            s_vga.getScreenHeight(),
            s_vga.getViewPortWidth(),
            s_vga.getViewPortHeight(),
            s_vga.getViewPortRow());

    ESP_ERROR_CHECK(
            esp_task_wdt_init(CONFIG_ESP_TASK_WDT_TIMEOUT_S, false)
            );

    /*
     * createRawPixel() must run after setResolution(), because the VGA
     * sync configuration is established during resolution setup.
     */
    for (int i = 0; i < 8; ++i) {
        s_sig[i] = s_vga.createRawPixel(s_bbc_palette[i]);
    }
    vga_bitmap_init_pairs(s_bitmap_pairs, s_sig);

    for (int b = 0; b < 256; ++b) {
        s_sig_pair[b].first  = s_sig[(b >> 4) & 7];
        s_sig_pair[b].second = s_sig[b & 7];
    }

    printf("[vga] VGADirectController running\n");

    /* --- PS/2 keyboard initialization --------------------------------- */
    s_ps2.begin(fabgl::PS2Preset::KeyboardPort0);

    printf("[kbd] PS/2 controller on CLK=%d DATA=%d\n",
           (int)BOARD_PS2_KBD_CLK,
           (int)BOARD_PS2_KBD_DATA);

    /* --- BBC machine allocation --------------------------------------- */
    machine = (bbc_machine_t *)heap_caps_malloc(
        sizeof(bbc_machine_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (machine) {
        printf("[mem] machine state: %u bytes in internal SRAM\n",
               (unsigned)sizeof(bbc_machine_t));
    } else {
        printf("[mem] internal machine allocation failed; trying PSRAM\n");
        machine = (bbc_machine_t *)heap_caps_malloc(
            sizeof(bbc_machine_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (machine)
            printf("[mem] machine state in PSRAM; performance may be reduced\n");
    }

    if (!machine) {
        printf("[main] machine allocation failed in SRAM and PSRAM\n");
        return;
    }

    const uint32_t os_size =
        (uint32_t)(os12_rom_end - os12_rom);

    const uint32_t basic_size =
        (uint32_t)(basic2_rom_end - basic2_rom);

    printf("[main] OS ROM: %lu B  BASIC ROM: %lu B\n",
           (unsigned long)os_size,
           (unsigned long)basic_size);

    bbc_machine_init(machine,
                     os12_rom, os_size,
                     basic2_rom, basic_size);

    mountDefaultTape(machine);

    /*
     * Private RAM snapshot used by the rendering task.
     * This avoids rendering directly from RAM while the emulated CPU
     * is modifying it.
     */
    s_render_ram = (uint8_t *)heap_caps_malloc(
            BBC_MEMORY_RAM_SIZE,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            );

    if (!s_render_ram) {
        printf("[video] internal RAM snapshot allocation failed\n");

        heap_caps_free(machine);
        machine = nullptr;
        return;
    }
    memset(s_render_ram, 0, BBC_MEMORY_RAM_SIZE);
    printf("[mem] Model A render snapshot: %u bytes in internal SRAM\n",
           (unsigned)BBC_MEMORY_RAM_SIZE);

    for (int i = 0; i < 2; ++i) {
        s_packed_fb[i] = static_cast<uint8_t *>(
            heap_caps_malloc(VGA_PACKED_BYTES,
                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (!s_packed_fb[i]) {
            printf("[video] internal packed buffer %d allocation failed\n", i);
            for (int j = 0; j < i; ++j) {
                heap_caps_free(s_packed_fb[j]);
                s_packed_fb[j] = nullptr;
            }
            heap_caps_free(s_render_ram);
            s_render_ram = nullptr;
            heap_caps_free(machine);
            machine = nullptr;
            return;
        }
        memset(s_packed_fb[i], 0, VGA_PACKED_BYTES);
    }
    printf("[video] internal packed buffers: %u bytes each\n",
           (unsigned)VGA_PACKED_BYTES);

    /*
     * Initialize double-buffer state before starting any tasks.
     * Front buffer 0 starts black; the renderer will publish its first
     * completed frame for display at the next VGA frame boundary.
     */
    s_fb_front   = 0;
    s_fb_pending = 1;
    s_fb_display = 0;
    s_fb_ready   = false;

    printf("[mem] free PSRAM after initialization: %lu bytes\n",
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* --- Start tasks --------------------------------------------------- */
    printf("[main] starting tasks\n");

    /*
     * Audio: Core 1, priority 2.
     */
    if (xTaskCreatePinnedToCore(
            audioTask, "audio", 4096,
            nullptr, 2, nullptr, 1) != pdPASS) {
        printf("[main] failed to create audio task\n");
        return;
    }

    /*
     * Keyboard: Core 1, priority 8.
     */
    if (xTaskCreatePinnedToCore(
            keyboardTask, "kbd", 2048,
            nullptr, 8, nullptr, 1) != pdPASS) {
        printf("[main] failed to create keyboard task\n");
        return;
    }

    /*
     * Emulator: Core 0, priority 5.
     */
    if (xTaskCreatePinnedToCore(
            emulatorTask, "emu", 8192,
            nullptr, 5, nullptr, 0) != pdPASS) {
        printf("[main] failed to create emulator task\n");
        return;
    }

    /*
     * Video renderer: Core 0, priority 3.
     * It renders complete frames outside the VGA scanline callback.
     */
    if (xTaskCreatePinnedToCore(
            videoRenderTask, "vidrender", 8192,
            nullptr, 1, nullptr, 1) != pdPASS) {
        printf("[main] failed to create video renderer task\n");
        return;
    }

    printf("[main] all tasks created\n");

    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;

printf("[mem] internal free=%u largest=%u\n",
       (unsigned)heap_caps_get_free_size(caps),
       (unsigned)heap_caps_get_largest_free_block(caps));
}

void loop(void)
{
    vTaskDelay(portMAX_DELAY);
}

/*
 * bbc_video.c — BBC Micro video subsystem integration
 *
 * Integrates MC6845 CRTC + Video ULA + SAA5050 → framebuffer.
 *
 * MA+RA → RAM address through the BBC's IC32/IC39 address translator.
 *
 * Licence: zlib
 * Copyright (c) 2026 esp-beep project
 */

#include <string.h>
#include "bbc_video.h"

#ifdef ESP_PLATFORM
#  include "esp_log.h"
#  include "esp_attr.h"
#  define VID_LOGD(fmt, ...) ESP_LOGD("bbc_video", fmt, ##__VA_ARGS__)
#  define VID_LOGW(fmt, ...) ESP_LOGW("bbc_video", fmt, ##__VA_ARGS__)
/* Functions called from the FabGL draw_scanline ISR (Core 1, IRAM context)
 * must reside in IRAM so they are reachable without the instruction cache. */
#  define RENDER_IRAM IRAM_ATTR
#else
#  include <stdio.h>
#  define VID_LOGD(fmt, ...) /* no-op */
#  define VID_LOGW(fmt, ...) fprintf(stderr, "bbc_video WARN: " fmt "\n", ##__VA_ARGS__)
#  define RENDER_IRAM
#endif

/* --------------------------------------------------------------------------
 * Framebuffer pixel write helpers
 * -------------------------------------------------------------------------- */

static inline void write_pixel(uint8_t *fb, uint32_t stride,
                                int x, int y,
                                bbc_rgb_t rgb,
                                bbc_fb_format_t fmt)
{
    switch (fmt) {
    case BBC_FB_FORMAT_INDEX8: {
        uint8_t idx = (uint8_t)(((rgb.b ? 1 : 0) << 2) |
                                ((rgb.g ? 1 : 0) << 1) |
                                 (rgb.r ? 1 : 0));
        fb[y * stride + x] = idx;
        break;
    }
    case BBC_FB_FORMAT_RGB332: {
        uint8_t v = (rgb.r & 0xE0) | ((rgb.g >> 3) & 0x1C) | (rgb.b >> 6);
        fb[y * stride + x] = v;
        break;
    }
    case BBC_FB_FORMAT_RGB565: {
        uint16_t r5 = (rgb.r >> 3) & 0x1F;
        uint16_t g6 = (rgb.g >> 2) & 0x3F;
        uint16_t b5 = (rgb.b >> 3) & 0x1F;
        uint16_t pix = (r5 << 11) | (g6 << 5) | b5;
        uint8_t *p = fb + y * stride + x * 2;
        p[0] = pix & 0xFF;
        p[1] = (pix >> 8) & 0xFF;
        break;
    }
    case BBC_FB_FORMAT_RGB888: {
        uint8_t *p = fb + y * stride + x * 3;
        p[0] = rgb.r;
        p[1] = rgb.g;
        p[2] = rgb.b;
        break;
    }
    }
}

/* Translate CRTC MA/RA through the BBC address-wrap logic, then apply the
 * physical RAM aliasing of the installed model. IC32 C1:C0 selects the
 * restart address when MA12 carries the byte address into &8000. */
static inline uint32_t bbc_bitmap_ram_addr(const bbc_video_t *video,
                                           uint16_t ma, uint8_t ra)
{
    if (ra > 7 || video->ram_size == 0)
        return UINT32_MAX;

    uint32_t addr = ((uint32_t)ma << 3) | ra;
    if (ma & 0x1000u) {
        uint32_t restart;
        if (video->scroll_c1)
            restart = video->scroll_c0 ? 0x5800u : 0x3000u;
        else
            restart = video->scroll_c0 ? 0x6000u : 0x4000u;

        addr = (addr & 0x7FFFu) + restart;
    }

    addr &= 0x7FFFu;
    if (video->ram_size <= 0x4000u)
        addr &= 0x3FFFu;

    return addr < video->ram_size ? addr : UINT32_MAX;
}

static inline bool bbc_bitmap_mode_supported(const bbc_video_t *video)
{
    /* On the Model A, C0=1 selects either Mode 4/5 or Mode 6. */
    return video->scroll_c0;
}

/* Resolve a MODE 7 character address using the CRTC's live screen start.
 * MOS scrolls MODE 7 by advancing R12/R13; a fixed &3C00/&7C00 base
 * renders the old page and makes the prompt jump when the screen wraps.
 *
 * Teletext addressing uses MA[9:0] as a 1 KiB ring and MA11 to select
 * &3C00 or &7C00 on a Model B. The Model A has only 16 KiB, so both
 * logical windows alias the physical &3C00 page. */
static inline uint32_t bbc_mode7_ram_addr(const bbc_video_t *video,
                                           uint32_t row, uint32_t col)
{
    uint32_t ma = ((uint32_t)mc6845_get_start_addr(&video->crtc) +
                   row * SAA5050_COLS + col) & 0x3FFFu;
    uint32_t offset = ma & 0x03FFu;

    if (video->ram_size <= 0x4000u)
        return BBC_SCREEN_BASE_MODE7_MODEL_A + offset;

    return ((ma & 0x0800u) != 0 ? BBC_SCREEN_BASE_MODE7
                                : BBC_SCREEN_BASE_MODE7_MODEL_A) + offset;
}

/* --------------------------------------------------------------------------
 * VSYNC callback bridge
 * -------------------------------------------------------------------------- */
static void _vsync_cb(void *ctx, bool state)
{
    bbc_video_t *video = (bbc_video_t *)ctx;
    if (video->vsync_cb)
        video->vsync_cb(video->vsync_ctx, state);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void bbc_video_init(bbc_video_t *video,
                     const uint8_t *system_ram, uint32_t ram_size)
{
    memset(video, 0, sizeof(*video));
    video->system_ram = system_ram;
    video->ram_size   = ram_size;

    mc6845_init(&video->crtc, MC6845_TYPE_MC6845);
    mc6845_set_vsync_callback(&video->crtc, _vsync_cb, video);

    bbc_video_ula_init(&video->ula);
    saa5050_init(&video->teletext, NULL);

    VID_LOGD("init: ram=%p size=%lu", (const void *)system_ram,
             (unsigned long)ram_size);
}

void bbc_video_reset(bbc_video_t *video)
{
    mc6845_reset(&video->crtc);
    bbc_video_ula_reset(&video->ula);
    saa5050_reset(&video->teletext);
    video->scroll_c0 = false;
    video->scroll_c1 = false;
    video->frames_rendered = 0;
    VID_LOGD("reset");
}

void bbc_video_set_output(bbc_video_t *video,
                           const bbc_video_output_t *output)
{
    video->output = *output;
}

void bbc_video_set_vsync_callback(bbc_video_t *video,
                                   void (*cb)(void *ctx, bool state),
                                   void *ctx)
{
    video->vsync_cb  = cb;
    video->vsync_ctx = ctx;
}

void bbc_video_set_frame_callback(bbc_video_t *video,
                                   void (*cb)(void *ctx),
                                   void *ctx)
{
    video->frame_cb  = cb;
    video->frame_ctx = ctx;
}

void bbc_video_crtc_write(bbc_video_t *video, uint8_t addr, uint8_t data)
{
    mc6845_write(&video->crtc, addr, data);
}

uint8_t bbc_video_crtc_read(bbc_video_t *video, uint8_t addr)
{
    return mc6845_read(&video->crtc, addr);
}

void bbc_video_vidproc_write(bbc_video_t *video, uint8_t addr, uint8_t data)
{
    bbc_video_ula_write(&video->ula, addr, data);
}

void bbc_video_set_scroll_control(bbc_video_t *video, bool c0, bool c1)
{
    video->scroll_c0 = c0;
    video->scroll_c1 = c1;
}

/* --------------------------------------------------------------------------
 * Cycle-exact tick (called once per CRTC clock from the main loop)
 * -------------------------------------------------------------------------- */
void bbc_video_tick(bbc_video_t *video)
{
    /* Always tick the CRTC — it generates VSYNC which drives the MOS IRQ.
     * The framebuffer render is skipped if there is no framebuffer, but the
     * CRTC timing must run regardless (e.g. host build with render_row). */
    mc6845_output_t out = mc6845_tick(&video->crtc);
#if defined(BBC_VIDEO_FRAME_RENDER_ONLY)
    (void)out;
    return;
#endif
    if (!video->output.framebuffer || !video->system_ram) return;
    if (!out.display_enable) return;
    if (!video->ula.teletext_mode && !bbc_bitmap_mode_supported(video)) return;

    const mc6845_t *crtc = &video->crtc;
    uint8_t *fb = (uint8_t *)video->output.framebuffer;

    /* Approximate screen position — not perfectly cycle-exact but close */
    int char_row  = (int)crtc->v_ctr;
    int scan_line = (int)crtc->r_ctr;
    int char_col  = (int)crtc->h_ctr;
    int scans     = (int)(crtc->max_scanline_addr + 1);
    int h_disp    = (int)crtc->h_displayed;
    int v_disp    = (int)crtc->v_displayed;

    if (h_disp == 0 || v_disp == 0) return;

    int out_y = (char_row * scans + scan_line) * (int)video->output.height
                / (v_disp * scans);
    if (out_y < 0 || out_y >= (int)video->output.height) return;

    if (video->ula.teletext_mode) {
        uint32_t ram_addr = bbc_mode7_ram_addr(video,
                                                (uint32_t)char_row,
                                                (uint32_t)char_col);
        if (ram_addr >= video->ram_size) return;
        uint8_t code = video->system_ram[ram_addr] & 0x7F;

        saa5050_line_state_t ls;
        saa5050_start_scanline(&video->teletext, &ls, (uint8_t)(scan_line * 2));
        uint8_t pixels[SAA5050_PIXELS_PER_CHAR];
        saa5050_render_char(&video->teletext, &ls, code, pixels);

        int base_x = char_col * (int)video->output.width / SAA5050_COLS;
        for (int i = 0; i < SAA5050_PIXELS_PER_CHAR; i++) {
            int fx = base_x + i * (int)video->output.width / (SAA5050_COLS * SAA5050_PIXELS_PER_CHAR);
            if (fx >= (int)video->output.width) break;
            bbc_rgb_t rgb = bbc_video_ula_colour(&video->ula, pixels[i]);
            write_pixel(fb, video->output.fb_stride, fx, out_y, rgb,
                        video->output.format);
        }
    } else {
        uint32_t ram_addr = bbc_bitmap_ram_addr(video, out.ma, out.ra);
        if (ram_addr == UINT32_MAX) return;
        uint8_t data_byte = video->system_ram[ram_addr];

        uint8_t colours[ULA_MAX_PIXELS_PER_BYTE];
        int npx = bbc_video_ula_serialize(&video->ula, data_byte,
                                          colours, out.cursor);
        /* hsync_modifier: screen columns per RAM byte (BeebEm HSyncModifier).
         * 2MHz clock → 8 cols/byte, 1MHz clock → 16 cols/byte. */
        int hsync    = video->ula.crtc_2mhz ? 8 : 16;
        int px_width = hsync / npx;   /* screen cols per logical pixel */
        int base_x   = char_col * hsync;
        for (int px = 0; px < npx; px++) {
            bbc_rgb_t rgb = bbc_video_ula_colour(&video->ula, colours[px]);
            int fx = base_x + px * px_width;
            for (int d = 0; d < px_width; d++) {
                int out_x = fx + d;
                if (out_x >= (int)video->output.width) break;
                write_pixel(fb, video->output.fb_stride, out_x, out_y, rgb,
                            video->output.format);
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * Frame-at-a-time rendering (recommended for ESP32)
 * -------------------------------------------------------------------------- */

static void render_bitmap_frame(bbc_video_t *video)
{
    const mc6845_t *crtc = &video->crtc;
    const bbc_video_ula_t *ula = &video->ula;
    uint8_t *fb = (uint8_t *)video->output.framebuffer;
    uint32_t stride = video->output.fb_stride;
    bbc_fb_format_t fmt = video->output.format;

    uint16_t start_addr = mc6845_get_start_addr(crtc);
    int chars_per_line = (int)crtc->h_displayed;
    int char_rows = (int)crtc->v_displayed;
    int scans_per_char = (int)(crtc->max_scanline_addr + 1);
    uint16_t cursor_addr = mc6845_get_cursor_addr(crtc);
    int cursor_start = (int)(crtc->cursor_start & 0x1F);
    int cursor_end = (int)(crtc->cursor_end & 0x1F);
    int hsync = ula->crtc_2mhz ? 8 : 16; /* screen cols per RAM byte */
    int total_scanlines = char_rows * scans_per_char;

    if (chars_per_line == 0 || char_rows == 0 || total_scanlines == 0)
        return;
    if (!bbc_bitmap_mode_supported(video)) {
        memset(fb, 0, (size_t)stride * video->output.height);
        return;
    }

    for (int out_y = 0; out_y < (int)video->output.height; out_y++) {
        int source_y = out_y * total_scanlines / (int)video->output.height;
        int row = source_y / scans_per_char;
        int sl = source_y % scans_per_char;
        for (int col = 0; col < chars_per_line; col++) {
            uint16_t ma = (uint16_t)((start_addr + row * chars_per_line + col) & 0x3FFF);
            uint32_t ram_addr = bbc_bitmap_ram_addr(video, ma, (uint8_t)sl);
            if (ram_addr == UINT32_MAX)
                continue;

            uint8_t data_byte = video->system_ram[ram_addr];

            bool cursor = (ma == cursor_addr) && (sl >= cursor_start) && (sl <= cursor_end) &&
                          crtc->cursor_blink_state;

            uint8_t colours[ULA_MAX_PIXELS_PER_BYTE];
            int npx = bbc_video_ula_serialize(ula, data_byte, colours, cursor);
            int px_width = hsync / npx;
            int base_x = col * hsync;
            for (int px = 0; px < npx; px++) {
                bbc_rgb_t rgb = bbc_video_ula_colour(ula, colours[px]);
                int fx = base_x + px * px_width;
                for (int d = 0; d < px_width; d++) {
                    int out_x = fx + d;
                    if (out_x >= (int)video->output.width)
                        break;
                    write_pixel(fb, stride, out_x, out_y, rgb, fmt);
                }
            }
        }
    }
}

#define BBC_TT_LINES_PER_ROW 10

static RENDER_IRAM void teletext_geometry(int out_height, int *lines_per_row, int *y_offset)
{
    *lines_per_row = BBC_TT_LINES_PER_ROW;
    if (*lines_per_row * SAA5050_ROWS > out_height)
        *lines_per_row = out_height / SAA5050_ROWS;
    *y_offset = (out_height - *lines_per_row * SAA5050_ROWS) / 2 - 20;
    if (*y_offset < 0)
        *y_offset = 0;
}

static void render_teletext_frame(bbc_video_t *video,
                                  uint8_t *packed, uint32_t packed_stride)
{
    saa5050_t *tt = &video->teletext;
    const bbc_video_ula_t *ula = &video->ula;

    uint8_t *fb = packed ? packed : (uint8_t *)video->output.framebuffer;
    uint32_t stride = packed ? packed_stride : video->output.fb_stride;
    bbc_fb_format_t fmt = packed ? BBC_FB_FORMAT_INDEX8 : video->output.format;

    if (!fb || !video->system_ram ||
        (!packed && (video->output.width == 0 || video->output.height == 0))) {
        return;
    }

    const int out_width  = packed ? SAA5050_COLS * SAA5050_PIXELS_PER_CHAR
                                  : (int)video->output.width;
    const int out_height = packed ? SAA5050_ROWS * BBC_TT_LINES_PER_ROW
                                  : (int)video->output.height;

    /* Match the 250-line Mode 7 band packed by the FabGL frontend. */
    const int content_w = SAA5050_COLS * SAA5050_PIXELS_PER_CHAR;
    int lines_per_row, y_offset;
    teletext_geometry(out_height, &lines_per_row, &y_offset);
    if (lines_per_row == 0) {
        memset(fb, 0, (size_t)stride * (size_t)out_height);
        return;
    }
    const int content_h = SAA5050_ROWS * lines_per_row;

    int x_offset = (out_width - content_w) / 2;
    int copy_w   = content_w;
    if (x_offset < 0) {
        x_offset = 0;
        copy_w   = out_width;
    }

    const bool fast = (fmt == BBC_FB_FORMAT_INDEX8) &&
                      (out_width <= BBC_FB_WIDTH);

    if (packed) {
        /* Every packed row is fully overwritten below. */
    } else if (fast) {
        memset(fb, 0, (size_t)stride * (size_t)y_offset);
        memset(fb + (size_t)stride * (size_t)(y_offset + content_h), 0,
               (size_t)stride * (size_t)(out_height - y_offset - content_h));
    } else {
        memset(fb, 0, (size_t)stride * (size_t)out_height);
    }

    uint8_t line_pixels[SAA5050_COLS * SAA5050_PIXELS_PER_CHAR];
    uint8_t row_cache[BBC_FB_WIDTH];      /* one finished output line */
    /* Precompute the INDEX8 colour mapping once per frame. */
    uint8_t index_lut[8];
    for (int i = 0; i < 8; ++i) {
        bbc_rgb_t rgb = bbc_video_ula_colour(ula, (uint8_t)i);
        index_lut[i] = (uint8_t)(
            ((rgb.b ? 1 : 0) << 2) |
            ((rgb.g ? 1 : 0) << 1) |
             (rgb.r ? 1 : 0));
    }

    saa5050_reset_frame(tt);

    for (int row = 0; row < SAA5050_ROWS; ++row) {
        saa5050_start_row(tt, (uint8_t)row);

        uint8_t row_ram[SAA5050_COLS];
        for (int col = 0; col < SAA5050_COLS; ++col) {
            uint32_t addr = bbc_mode7_ram_addr(video,
                                                (uint32_t)row,
                                                (uint32_t)col);
            row_ram[col] = (addr < video->ram_size)
                ? (video->system_ram[addr] & 0x7F)
                : 0x20;
        }

        int last_lc = -1;
        for (int j = 0; j < lines_per_row; ++j) {
            const int lc = j * SAA5050_LINES_PER_ROW / lines_per_row;
            if (lc != last_lc) {
                saa5050_render_line(tt, row_ram, (uint8_t)lc, line_pixels);
                if (packed) {
                    for (int px = 0; px < content_w; px += 2)
                        row_cache[px / 2] =
                            (uint8_t)((index_lut[line_pixels[px] & 7] << 4) |
                                      index_lut[line_pixels[px + 1] & 7]);
                } else if (fast) {
                    memset(row_cache, 0, (size_t)out_width);
                    for (int px = 0; px < copy_w; ++px)
                        row_cache[x_offset + px] = index_lut[line_pixels[px] & 7];
                }
                last_lc = lc;
            }

            const int out_y = y_offset + row * lines_per_row + j;
            if (packed) {
                memcpy(fb + (size_t)out_y * stride, row_cache,
                       (size_t)content_w / 2);
            } else if (fast) {
                memcpy(fb + (size_t)out_y * stride, row_cache,
                       (size_t)out_width);
            } else {
                for (int px = 0; px < copy_w; ++px) {
                    bbc_rgb_t rgb =
                        bbc_video_ula_colour(ula, line_pixels[px] & 7);
                    write_pixel(fb, stride, x_offset + px, out_y, rgb, fmt);
                }
            }
        }

        saa5050_end_row(tt);
    }
}

/* --------------------------------------------------------------------------
 * Single-row render — called on-demand from the VGA ISR (scanline callback).
 *
 * Renders output row 'out_y' (0 .. out_height-1) into out_pixels[out_width].
 * Each byte written is a physical BBC colour index (0-7, BBC_FB_FORMAT_INDEX8).
 *
 * This is the building block for framebuffer-free, direct-to-VGA rendering:
 * the FabGL draw_scanline ISR maps each VGA line to a BBC out_y and calls
 * this function to produce the 640 pixels for that line.
 *
 * Read-only on video state: safe to call from Core 1 ISR as long as
 * video->system_ram is not simultaneously written by Core 0.  For robust
 * synchronization, take a bbc_video_snapshot_t at VSYNC (see bbc_video.h).
 * -------------------------------------------------------------------------- */
static RENDER_IRAM void render_bitmap_row(const bbc_video_t *video,
                               int out_y, int out_height,
                               uint8_t *out_pixels, int out_width)
{
    const mc6845_t      *crtc = &video->crtc;
    const bbc_video_ula_t *ula  = &video->ula;

    uint16_t start_addr   = mc6845_get_start_addr(crtc);
    int chars_per_line    = (int)crtc->h_displayed;
    int char_rows         = (int)crtc->v_displayed;
    int scans_per_char    = (int)(crtc->max_scanline_addr + 1);
    uint16_t cursor_addr  = mc6845_get_cursor_addr(crtc);
    int cursor_start      = (int)(crtc->cursor_start & 0x1F);
    int cursor_end        = (int)(crtc->cursor_end   & 0x1F);
    int hsync           = ula->crtc_2mhz ? 8 : 16; /* screen cols per RAM byte */
    int total_scanlines = char_rows * scans_per_char;

    /* Clear row to index 0 (black) */
    memset(out_pixels, 0, (size_t)out_width);

    if (chars_per_line == 0 || char_rows == 0 || total_scanlines == 0) return;
    if (!bbc_bitmap_mode_supported(video)) return;

    int source_y = out_y * total_scanlines / out_height;
    int row = source_y / scans_per_char;
    int sl = source_y % scans_per_char;

    for (int col = 0; col < chars_per_line; col++) {
        uint16_t ma = (uint16_t)((start_addr + row * chars_per_line + col) & 0x3FFF);
        uint32_t ram_addr = bbc_bitmap_ram_addr(video, ma, (uint8_t)sl);
        if (ram_addr == UINT32_MAX) continue;

        uint8_t data_byte = video->system_ram[ram_addr];

        bool cursor = (ma == cursor_addr) &&
                      (sl >= cursor_start) && (sl <= cursor_end) &&
                      crtc->cursor_blink_state;

        uint8_t colours[ULA_MAX_PIXELS_PER_BYTE];
        int npx      = bbc_video_ula_serialize(ula, data_byte, colours, cursor);
        int px_width = hsync / npx;
        int base_x   = col * hsync;
        for (int px = 0; px < npx; px++) {
            bbc_rgb_t rgb = bbc_video_ula_colour(ula, colours[px]);
            /* Convert rgb → index8 inline */
            uint8_t idx = (uint8_t)(((rgb.b ? 1 : 0) << 2) |
                                    ((rgb.g ? 1 : 0) << 1) |
                                     (rgb.r ? 1 : 0));
            int fx = base_x + px * px_width;
            for (int d = 0; d < px_width; d++) {
                int out_x = fx + d;
                if (out_x < 0 || out_x >= out_width) break;
                out_pixels[out_x] = idx;
            }
        }
    }
}

/* Render one SAA5050 row for line_counter lc into out_pixels.
 * Caller must have called saa5050_start_row(tt, row) first. */
static RENDER_IRAM void render_teletext_scanline(const bbc_video_t *video,
                                      saa5050_t *tt, int row, int lc,
                                      int x_offset, int out_width,
                                      uint8_t *out_pixels)
{
    const bbc_video_ula_t *ula = &video->ula;

    /* Gather RAM for this row */
    uint8_t row_ram[SAA5050_COLS];
    for (int col = 0; col < SAA5050_COLS; col++) {
        uint32_t ra = bbc_mode7_ram_addr(video,
                                          (uint32_t)row,
                                          (uint32_t)col);
        row_ram[col] = (ra < video->ram_size)
                       ? (video->system_ram[ra] & 0x7F) : 0x20;
    }

    uint8_t line_pixels[SAA5050_COLS * SAA5050_PIXELS_PER_CHAR];
    saa5050_render_line(tt, row_ram, (uint8_t)lc, line_pixels);

    int content_w = SAA5050_COLS * SAA5050_PIXELS_PER_CHAR;
    for (int px = 0; px < content_w; px++) {
        int fx = x_offset + px;
        if (fx < 0 || fx >= out_width) continue;
        bbc_rgb_t rgb = bbc_video_ula_colour(ula, line_pixels[px]);
        uint8_t idx = (uint8_t)(((rgb.b ? 1 : 0) << 2) |
                                ((rgb.g ? 1 : 0) << 1) |
                                 (rgb.r ? 1 : 0));
        out_pixels[fx] = idx;
    }
}

static RENDER_IRAM void render_teletext_row(const bbc_video_t *video,
                                 int out_y, int out_height,
                                 uint8_t *out_pixels, int out_width)
{
    saa5050_t *tt = (saa5050_t *)&video->teletext; /* cast away const for SAA5050 state */

    int content_w = SAA5050_COLS * SAA5050_PIXELS_PER_CHAR; /* 480 */
    int x_offset  = (out_width - content_w) / 2;
    if (x_offset < 0) x_offset = 0;

    /* Clear row to 0 (black) */
    memset(out_pixels, 0, (size_t)out_width);

    int lines_per_row, y_offset;
    teletext_geometry(out_height, &lines_per_row, &y_offset);
    if (lines_per_row == 0 || out_y < y_offset ||
        out_y >= y_offset + SAA5050_ROWS * lines_per_row) return;
    const int relative_y = out_y - y_offset;
    const int target_row = relative_y / lines_per_row;
    const int target_lc = (relative_y % lines_per_row) *
                          SAA5050_LINES_PER_ROW / lines_per_row;

    /* Pre-pass: scan all rows before target_row to accumulate DH state. */
    saa5050_reset_frame(tt);
    for (int r = 0; r < target_row; r++) {
        saa5050_start_row(tt, (uint8_t)r);
        uint8_t row_ram[SAA5050_COLS];
        uint8_t dummy[SAA5050_COLS * SAA5050_PIXELS_PER_CHAR];
        for (int col = 0; col < SAA5050_COLS; col++) {
            uint32_t ra = bbc_mode7_ram_addr(video,
                                              (uint32_t)r,
                                              (uint32_t)col);
            row_ram[col] = (ra < video->ram_size)
                           ? (video->system_ram[ra] & 0x7F) : 0x20;
        }
        /* Render last lc only (to detect double_high1 for end_row) */
        saa5050_render_line(tt, row_ram,
                            (uint8_t)(SAA5050_LINES_PER_ROW - 1), dummy);
        saa5050_end_row(tt);
    }

    /* Render the target row */
    saa5050_start_row(tt, (uint8_t)target_row);
    render_teletext_scanline(video, tt, target_row, target_lc,
                             x_offset, out_width, out_pixels);
}

RENDER_IRAM void bbc_video_render_row(const bbc_video_t *video,
                           int out_y, int out_height,
                           uint8_t *out_pixels, int out_width)
{
    if (!video->system_ram || !out_pixels || out_width <= 0) return;
    if (out_y < 0 || out_y >= out_height) return;

    if (video->ula.teletext_mode)
        render_teletext_row(video, out_y, out_height, out_pixels, out_width);
    else
        render_bitmap_row(video, out_y, out_height, out_pixels, out_width);
}

void bbc_video_render_frame(bbc_video_t *video)
{
    if (!video->output.framebuffer || !video->system_ram) {
        VID_LOGW("render_frame: no framebuffer or RAM");
        return;
    }

    if (video->ula.teletext_mode)
        render_teletext_frame(video, NULL, 0);
    else
        render_bitmap_frame(video);

    video->frames_rendered++;

    if (video->frame_cb)
        video->frame_cb(video->frame_ctx);
}

bool bbc_video_render_mode7_packed(bbc_video_t *video,
                                   uint8_t *pixels, uint32_t stride)
{
    const uint32_t packed_width = SAA5050_COLS * SAA5050_PIXELS_PER_CHAR / 2;
    if (!video || !pixels || !video->system_ram || !video->ula.teletext_mode ||
        stride < packed_width) {
        VID_LOGW("render_mode7_packed: invalid output or video state");
        return false;
    }

    render_teletext_frame(video, pixels, stride);
    video->frames_rendered++;
    if (video->frame_cb)
        video->frame_cb(video->frame_ctx);
    return true;
}

bool bbc_video_render_bitmap_packed(bbc_video_t *video,
                                    uint8_t *pixels, uint32_t stride)
{
    if (!video || !pixels || !video->system_ram || video->ula.teletext_mode ||
        stride < BBC_BITMAP_PACKED_STRIDE) {
        VID_LOGW("render_bitmap_packed: invalid output or video state");
        return false;
    }

    uint8_t row[BBC_BITMAP_PACKED_WIDTH];
    for (int y = 0; y < BBC_BITMAP_PACKED_HEIGHT; ++y) {
        render_bitmap_row(video, y, BBC_BITMAP_PACKED_HEIGHT,
                          row, BBC_BITMAP_PACKED_WIDTH);
        uint8_t *dst = pixels + (size_t)y * stride;
        for (int x = 0; x < BBC_BITMAP_PACKED_WIDTH; x += 8) {
            uint32_t packed = 0;
            for (int i = 0; i < 8; ++i)
                packed = (packed << 3) | row[x + i];
            *dst++ = (uint8_t)(packed >> 16);
            *dst++ = (uint8_t)(packed >> 8);
            *dst++ = (uint8_t)packed;
        }
    }
    video->frames_rendered++;
    if (video->frame_cb)
        video->frame_cb(video->frame_ctx);
    return true;
}

void bbc_video_toggle_flash(bbc_video_t *video)
{
    bbc_video_ula_toggle_flash(&video->ula);
    saa5050_toggle_flash(&video->teletext);
}

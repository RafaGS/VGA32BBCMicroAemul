/*
 * bbc_tape.c — BBC Micro tape (UEF) emulation
 *
 * UEF format reference: http://electrem.emuunlim.com/UEFSpecs.htm
 *
 * We parse supported UEF chunks at load time into a flat list of tape blocks.
 *
 * The ACIA (MC6850-compatible Serial ULA) delivers bytes to the MOS.
 * Baud rate: 1200 baud = 1 byte per ~16667 cycles at 2 MHz.
 * We use a simple cycle counter to pace delivery.
 *
 * ACIA register map (BBC Micro):
 *   $FE08 write = control   $FE08 read = status
 *   $FE09 read  = RX data   $FE09 write = TX data (ignored)
 *
 * Status register bits:
 *   bit 0  RDRF  Receive Data Register Full
 *   bit 1  TDRE  Transmit Data Register Empty (always 1)
 *   bit 2  DCD   Data Carrier Detect (0=carrier/motor on, 1=no carrier)
 *   bit 3  CTS   Clear To Send (always 0)
 *   bit 7  IRQ   (RDRF & rx_irq_en) or (DCD changed)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include "bbc_tape.h"

#ifndef ESP_PLATFORM
#  include <zlib.h>   /* for gzip decompression on host */
#else
#  include "esp32/rom/miniz.h"
#endif

#ifdef ESP_PLATFORM
#  include "esp_log.h"
#  define TAPE_LOGI(fmt, ...) ESP_LOGI("bbc_tape", fmt, ##__VA_ARGS__)
#  define TAPE_LOGD(fmt, ...) ESP_LOGD("bbc_tape", fmt, ##__VA_ARGS__)
#  define TAPE_LOGW(fmt, ...) printf("[tape] WARN: " fmt "\n", ##__VA_ARGS__)
#else
#  include <stdio.h>
#  define TAPE_LOGI(fmt, ...) fprintf(stderr, "[tape] " fmt "\n", ##__VA_ARGS__)
#  define TAPE_LOGD(fmt, ...) ((void)0)
#  define TAPE_LOGW(fmt, ...) fprintf(stderr, "[tape] WARN: " fmt "\n", ##__VA_ARGS__)
#endif

/* 1200 baud, 8N1: 10 bits per byte at the 2 MHz emulated CPU clock. */
#define CYCLES_PER_BYTE  16667

/* -------------------------------------------------------------------------
 * UEF helpers — little-endian reads from raw buffer
 * ------------------------------------------------------------------------- */
static uint16_t read_u16le(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t read_u32le(const uint8_t *p) {
    return (uint32_t)(p[0] | ((uint32_t)p[1]<<8) |
                      ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24));
}

static uint32_t uef_crc32(const uint8_t *data, size_t size)
{
#ifdef ESP_PLATFORM
    return (uint32_t)mz_crc32(0, data, size);
#else
    return (uint32_t)crc32(0, data, (uInt)size);
#endif
}

static size_t uef_inflate_raw(uint8_t *out, size_t out_size,
                              const uint8_t *data, size_t size)
{
#ifdef ESP_PLATFORM
    tinfl_decompressor *state = (tinfl_decompressor *)malloc(sizeof(*state));
    if (!state) {
        TAPE_LOGW("cannot allocate gzip decompressor state");
        return (size_t)-1;
    }
    tinfl_init(state);
    size_t input_size = size, output_size = out_size;
    tinfl_status status = tinfl_decompress(state, data, &input_size, out, out,
                                           &output_size,
                                           TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    free(state);
    return status == TINFL_STATUS_DONE ? output_size : (size_t)-1;
#else
    z_stream zs;
    memset(&zs, 0, sizeof(zs));
    zs.next_in = (Bytef *)data;
    zs.avail_in = (uInt)size;
    zs.next_out = out;
    zs.avail_out = (uInt)out_size;
    if (inflateInit2(&zs, -15) != Z_OK) return (size_t)-1;
    int result = inflate(&zs, Z_FINISH);
    size_t written = zs.total_out;
    inflateEnd(&zs);
    return result == Z_STREAM_END ? written : (size_t)-1;
#endif
}

static uint8_t *uef_decompress_gzip(const uint8_t *raw, size_t size,
                                    size_t *out_size)
{
    if (size < 18 || raw[2] != 8 || (raw[3] & 0xE0) != 0) {
        TAPE_LOGW("invalid gzip header");
        return NULL;
    }
    size_t pos = 10, end = size - 8;
    uint8_t flags = raw[3];
    if (flags & 4) {
        if (end - pos < 2) return NULL;
        size_t extra = read_u16le(raw + pos);
        pos += 2;
        if (extra > end - pos) return NULL;
        pos += extra;
    }
    for (uint8_t bit = 8; bit <= 16; bit <<= 1) {
        if (flags & bit) {
            while (pos < end && raw[pos] != 0) ++pos;
            if (pos == end) return NULL;
            ++pos;
        }
    }
    if (flags & 2) {
        if (end - pos < 2 ||
            (uef_crc32(raw, pos) & 0xFFFFu) != read_u16le(raw + pos))
            return NULL;
        pos += 2;
    }
    size_t expected = read_u32le(raw + end + 4);
    if (expected < 12 || pos >= end || size > UINT_MAX) {
        TAPE_LOGW("invalid gzip size or payload");
        return NULL;
    }
    uint8_t *out = (uint8_t *)malloc(expected);
    if (!out) {
        TAPE_LOGW("cannot allocate %zu bytes for gzip UEF", expected);
        return NULL;
    }
    size_t written = uef_inflate_raw(out, expected, raw + pos, end - pos);
    if (written != expected || uef_crc32(out, written) != read_u32le(raw + end)) {
        TAPE_LOGW("gzip decompression or CRC failed (expected=%zu, decoded=%zu)",
                  expected, written);
        free(out);
        return NULL;
    }
    *out_size = written;
    return out;
}
/* -------------------------------------------------------------------------
 * bbc_tape_init
 * ------------------------------------------------------------------------- */
void bbc_tape_init(bbc_tape_t *tape)
{
    memset(tape, 0, sizeof(*tape));
}

/* -------------------------------------------------------------------------
 * bbc_tape_free
 * ------------------------------------------------------------------------- */
void bbc_tape_free(bbc_tape_t *tape)
{
    if (tape->uef_data) { free(tape->uef_data); tape->uef_data = NULL; }
    if (tape->blocks)   { free(tape->blocks);   tape->blocks   = NULL; }
    tape->n_blocks = 0;
}

/* -------------------------------------------------------------------------
 * bbc_tape_load_uef
 *
 * UEF file structure:
 *   10 bytes header: "UEF File!\0" + minor_ver + major_ver
 *   Then chunks:
 *     2 bytes chunk ID (LE)
 *     4 bytes chunk length (LE)
 *     N bytes chunk data
 *
 * Chunk 0x0100 = tape data block. The chunk data IS the raw tape block
 * bytes (sync byte 0x2A + BBC tape block header + data + CRC).
 * We store each 0x0100 chunk as one bbc_tape_block_t.
 * ------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------
 * uef_load_raw — load file into memory, decompressing gzip if needed.
 * Returns malloc'd buffer (caller must free) and sets *out_size.
 * Returns NULL on error.
 * ------------------------------------------------------------------------- */
static uint8_t *uef_load_raw(const char *path, size_t *out_size)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        TAPE_LOGW("cannot open %s (errno=%d)", path, errno);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 12) {
        TAPE_LOGW("file too small or cannot determine size: %s (%ld)", path, sz);
        fclose(f); return NULL;
    }

    uint8_t *raw = (uint8_t *)malloc((size_t)sz);
    if (!raw) {
        TAPE_LOGW("cannot allocate %ld bytes for %s", sz, path);
        fclose(f); return NULL;
    }
    if ((long)fread(raw, 1, (size_t)sz, f) != sz) {
        TAPE_LOGW("cannot read complete file %s (errno=%d)", path, errno);
        free(raw); fclose(f); return NULL;
    }
    fclose(f);

    /* Check for gzip magic: 1F 8B */
    if (raw[0] == 0x1F && raw[1] == 0x8B) {
        uint8_t *out = uef_decompress_gzip(raw, (size_t)sz, out_size);
        free(raw);
        if (!out) TAPE_LOGW("invalid or unsupported gzip UEF");
        return out;
    }

    *out_size = (size_t)sz;
    return raw;
}

int bbc_tape_load_uef(bbc_tape_t *tape, const char *path)
{
    bbc_tape_free(tape);

    size_t uef_size = 0;
    uint8_t *uef_data = uef_load_raw(path, &uef_size);
    if (!uef_data) { TAPE_LOGW("cannot load %s", path); return -1; }

    /* Verify UEF magic */
    if (uef_size < 12 || memcmp(uef_data, "UEF File!", 9) != 0) {
        TAPE_LOGW("bad UEF magic in %s", path);
        free(uef_data); return -1;
    }

    tape->uef_data = uef_data;
    tape->uef_size = uef_size;

    /* Count data and synthetic carrier blocks before allocating them. */
    uint16_t n = 0;
    size_t pos = 12;
    while (pos < tape->uef_size) {
        if (tape->uef_size - pos < 6) {
            TAPE_LOGW("truncated UEF chunk header in %s", path);
            bbc_tape_free(tape); return -1;
        }
        uint16_t chunk_id  = read_u16le(tape->uef_data + pos);
        uint32_t chunk_len = read_u32le(tape->uef_data + pos + 2);
        pos += 6;
        if (chunk_len > tape->uef_size - pos) {
            TAPE_LOGW("truncated UEF chunk in %s", path);
            bbc_tape_free(tape); return -1;
        }
        if ((chunk_id == UEF_CHUNK_DATA && chunk_len > BBC_TAPE_BLOCK_MAX) ||
            (chunk_id == UEF_CHUNK_FORMAT_DATA && chunk_len > BBC_TAPE_BLOCK_MAX + 3)) {
            TAPE_LOGW("UEF data chunk too large in %s", path);
            bbc_tape_free(tape); return -1;
        }
        if (chunk_id == UEF_CHUNK_FORMAT_DATA &&
            (chunk_len < 3 || tape->uef_data[pos] != 8 ||
             tape->uef_data[pos + 1] != 'N' || tape->uef_data[pos + 2] != 1)) {
            TAPE_LOGW("unsupported UEF data framing in %s", path);
            bbc_tape_free(tape); return -1;
        }
        if (chunk_id == UEF_CHUNK_CARRIER_DUMMY && chunk_len < 4) {
            TAPE_LOGW("truncated UEF carrier in %s", path);
            bbc_tape_free(tape); return -1;
        }
        unsigned add = 0;
        if ((chunk_id == UEF_CHUNK_DATA && chunk_len > 0) ||
            (chunk_id == UEF_CHUNK_FORMAT_DATA && chunk_len > 3))
            add = 1;
        else if (chunk_id == UEF_CHUNK_CARRIER && chunk_len >= 2)
            add = read_u16le(tape->uef_data + pos) >= 20;
        else if (chunk_id == UEF_CHUNK_CARRIER_DUMMY)
            add = 1 + (read_u16le(tape->uef_data + pos) >= 20) +
                      (read_u16le(tape->uef_data + pos + 2) >= 20);
        if (add) {
            if (add > UINT16_MAX - n) {
                TAPE_LOGW("too many UEF blocks in %s", path);
                bbc_tape_free(tape); return -1;
            }
            n += add;
        }
        pos += chunk_len;
    }

    if (n == 0) {
        TAPE_LOGW("no data chunks in %s", path);
        bbc_tape_free(tape); return -1;
    }

    tape->blocks = (bbc_tape_block_t *)calloc(n, sizeof(bbc_tape_block_t));
    if (!tape->blocks) {
        TAPE_LOGW("cannot allocate %u UEF blocks", n);
        bbc_tape_free(tape); return -1;
    }
    tape->n_blocks = n;

    /* Second pass: fill blocks */
    uint16_t bi = 0;
    pos = 12;
    while (pos + 6 <= tape->uef_size && bi < n) {
        uint16_t chunk_id  = read_u16le(tape->uef_data + pos);
        uint32_t chunk_len = read_u32le(tape->uef_data + pos + 2);
        pos += 6;
        if ((chunk_id == UEF_CHUNK_DATA && chunk_len > 0) ||
            (chunk_id == UEF_CHUNK_FORMAT_DATA && chunk_len > 3)) {
            size_t prefix = chunk_id == UEF_CHUNK_FORMAT_DATA ? 3 : 0;
            uint16_t blen = (uint16_t)(chunk_len - prefix);
            memcpy(tape->blocks[bi].data, tape->uef_data + pos + prefix, blen);
            tape->blocks[bi].len = blen;
            bi++;
        } else if (chunk_id == UEF_CHUNK_CARRIER_DUMMY) {
            uint16_t before = read_u16le(tape->uef_data + pos) / 20;
            uint16_t after = read_u16le(tape->uef_data + pos + 2) / 20;
            if (before) {
                if (before > BBC_TAPE_BLOCK_MAX) before = BBC_TAPE_BLOCK_MAX;
                memset(tape->blocks[bi].data, 0xDC, before);
                tape->blocks[bi].len = before;
                tape->blocks[bi++].is_carrier = true;
            }
            tape->blocks[bi].data[0] = 0xAA;
            tape->blocks[bi++].len = 1;
            if (after) {
                if (after > BBC_TAPE_BLOCK_MAX) after = BBC_TAPE_BLOCK_MAX;
                memset(tape->blocks[bi].data, 0xDC, after);
                tape->blocks[bi].len = after;
                tape->blocks[bi++].is_carrier = true;
            }
        } else if (chunk_id == UEF_CHUNK_CARRIER && chunk_len >= 2) {
            uint16_t cycles = read_u16le(tape->uef_data + pos);
            uint16_t nbytes = (uint16_t)(cycles / 20);
            if (nbytes >= 1) {
                if (nbytes > BBC_TAPE_BLOCK_MAX) nbytes = BBC_TAPE_BLOCK_MAX;
                memset(tape->blocks[bi].data, 0xDC, nbytes);
                tape->blocks[bi].len = nbytes;
                tape->blocks[bi].is_carrier = true;
                TAPE_LOGI("carrier tone: %u cycles -> %u bytes $DC", cycles, nbytes);
                bi++;
            }
        }
        if (pos + chunk_len > tape->uef_size) break;
        pos += chunk_len;
    }

    TAPE_LOGI("loaded %s: %d tape blocks (data+carrier)", path, n);
    return 0;
}

/* -------------------------------------------------------------------------
 * bbc_tape_set_irq_cb
 * ------------------------------------------------------------------------- */
void bbc_tape_set_irq_cb(bbc_tape_t *tape,
                          void (*cb)(void *ctx, bool state), void *ctx)
{
    tape->irq_cb  = cb;
    tape->irq_ctx = ctx;
}

/* -------------------------------------------------------------------------
 * bbc_tape_set_motor
 * ------------------------------------------------------------------------- */
void bbc_tape_set_motor(bbc_tape_t *tape, bool on)
{
    if (tape->motor_on == on) return;
    tape->motor_on = on;
    TAPE_LOGI("motor %s", on ? "ON" : "OFF");
    if (on) {
        /* Give the MOS ~200ms to configure the ACIA before first byte arrives
         * on the first motor-on.  Subsequent motor-on (motor toggle) uses a
         * shorter re-engage delay (one byte period) so the stream resumes
         * promptly without discarding already-latched bytes.
         * Skipped entirely in test mode (no_motor_delay). */
        if (!tape->running) {
            tape->cycle_acc = tape->no_motor_delay ? 0 : -400000;
        } else {
            /* Already ran before — just a brief re-engage delay */
            tape->cycle_acc = tape->no_motor_delay ? 0 : -CYCLES_PER_BYTE;
        }
        tape->running = true;
        /* Do NOT reset rx_full here: a byte latched while motor was briefly
         * off should still be available to the MOS on motor-on. */
    }
}

/* -------------------------------------------------------------------------
 * ACIA status byte — BBC Micro MC6850 mapping:
 *   bit 0 = RDRF  Receive Data Register Full
 *   bit 1 = TDRE  Transmit Data Register Empty (always 1 — no TX emulated)
 *   bit 2 = DCD   Data Carrier Detect
 *   bit 3 = CTS   Clear To Send (always 0 — unused in BBC cassette path)
 *   bit 4 = FE    Framing Error (always 0)
 *   bit 5 = OVRN  Overrun (always 0)
 *   bit 6 = PE    Parity Error (always 0)
 *   bit 7 = IRQ   = rx_ie & (rx_full | ovr)
 *
 * DCD polarity (BBC Micro specific):
 *   In the BBC Micro cassette circuit, /DCD on the ACIA is driven HIGH
 *   when the cassette motor is running and carrier is present.  This is
 *   non-standard: /DCD HIGH = DCD status bit = 1 = carrier detected.
 *   The MOS uses this at $F5B7 (BCC $F61D): after 3×LSR of status,
 *   carry = original DCD bit.  BCC exits when carry=0 (no carrier).
 *   So DCD=1 (motor on) is required for $C2 to advance from 1→2.
 *
 *   Summary:  motor ON  → DCD=1 (carrier present, /DCD pin HIGH)
 *             motor OFF → DCD=0 (no carrier,   /DCD pin LOW)
 * ------------------------------------------------------------------------- */
static uint8_t acia_status(const bbc_tape_t *tape)
{
    uint8_t s = 0x02;                 /* TDRE always 1 */
    if (tape->rx_full)  s |= 0x01;   /* RDRF */
    /* DCD (bit 2) — BBC Micro cassette-specific:
     *   The cassette carrier-detect circuit drives /DCD HIGH during the
     *   carrier tone (leader), giving DCD status bit = 1.
     *   During actual data bytes (after the leader), /DCD goes LOW,
     *   giving DCD status bit = 0.
     *   The MOS uses this at $F5B7/BCC and $F5C0/BCS to distinguish:
     *     carrier bytes (DCD=1) → C=1 after 3xLSR → advances C2 1→2
     *     data bytes    (DCD=0) → C=0 after 3xLSR → C2 2→3 via CMP $2A */
    /* DCD drops when tape ends — no more carrier after last block */
    bool tape_has_data = tape->blocks && (tape->cur_block < tape->n_blocks || tape->rx_full);
    if (tape->motor_on && tape->rx_is_carrier && tape_has_data) s |= 0x04;
    /* IRQ = rx_ie & rx_full */
    if (tape->irq_enabled && tape->rx_full) s |= 0x80;
    return s;
}

/* -------------------------------------------------------------------------
 * bbc_tape_read
 *   reg=0 → status register ($FE08)
 *   reg=1 → receive data register ($FE09)
 * ------------------------------------------------------------------------- */
uint8_t bbc_tape_read(bbc_tape_t *tape, uint8_t reg)
{
    if (reg == 0) {
        uint8_t s = acia_status(tape);
        TAPE_LOGD("ACIA status=%02X", s);
        return s;
    } else {
        /* Reading RDR clears RDRF and deasserts IRQ */
        uint8_t d = tape->rx_data;
        tape->rx_full = false;
        /* Update IRQ line */
        if (tape->irq_cb) tape->irq_cb(tape->irq_ctx, false);
        TAPE_LOGD("RX %02X", d);
        return d;
    }
}

/* -------------------------------------------------------------------------
 * bbc_tape_write
 *   reg=0 → control register ($FE08)
 *   reg=1 → TX data ($FE09, ignored for tape load)
 *
 * MC6850 control register (from Verilog):
 *   bits 1:0 = clk_mult / master reset (11 = master reset)
 *   bit  2   = parity_odd
 *   bits 4:3 = word select
 *   bits 6:5 = TX control
 *   bit  7   = rx_ie (RX interrupt enable)
 * ------------------------------------------------------------------------- */
void bbc_tape_write(bbc_tape_t *tape, uint8_t reg, uint8_t val)
{
    if (reg == 0) {
        tape->acia_control = val;
        bool old_irq_en = tape->irq_enabled;
        tape->irq_enabled = (val & 0x80) ? true : false;

        /* Master reset: bits 1:0 = 11 */
        if ((val & 0x03) == 0x03) {
            tape->rx_full = false;
            if (tape->irq_cb) tape->irq_cb(tape->irq_ctx, false);
            TAPE_LOGD("ACIA master reset");
        } else if (!old_irq_en && tape->irq_enabled && tape->rx_full) {
            /* IRQ just enabled and byte already waiting — assert IRQ now */
            if (tape->irq_cb) tape->irq_cb(tape->irq_ctx, true);
        }
        TAPE_LOGD("ACIA ctrl=%02X rx_ie=%d", val, tape->irq_enabled);
    }
    /* TX data writes ignored (we only emulate RX) */
}

/* -------------------------------------------------------------------------
 * bbc_tape_tick
 *
 * Called every bbc_machine_step with the number of CPU cycles elapsed.
 * When the motor is on and a block is available, delivers bytes at
 * 1200 baud pace.
 * ------------------------------------------------------------------------- */
void bbc_tape_tick(bbc_tape_t *tape, int cycles)
{
    /* Only deliver bytes when the motor is running.
     * We require both running (has been on at least once) and motor_on.
     * The DCD bit in status reflects motor_on; the MOS uses DCD for error detection. */
    if (!tape->running || !tape->motor_on) return;
    if (!tape->blocks || tape->cur_block >= tape->n_blocks) return;
    if (tape->rx_full) return;  /* MOS hasn't read the previous byte yet */

    tape->cycle_acc += cycles;
    if (tape->cycle_acc < CYCLES_PER_BYTE) return;
    tape->cycle_acc -= CYCLES_PER_BYTE;

    /* Deliver next byte */
    bbc_tape_block_t *blk = &tape->blocks[tape->cur_block];
    tape->rx_data       = blk->data[tape->cur_pos++];
    tape->rx_full       = true;
    tape->rx_is_carrier = blk->is_carrier;

    if (tape->cur_pos == 1)
        TAPE_LOGD("delivering block %d (%d bytes)", tape->cur_block, blk->len);
    TAPE_LOGD("deliver block=%d pos=%d byte=%02X",
              tape->cur_block, tape->cur_pos - 1, tape->rx_data);

    /* Advance to next block if done */
    if (tape->cur_pos >= blk->len) {
        tape->cur_pos = 0;
        tape->cur_block++;
        if (tape->cur_block >= tape->n_blocks)
            TAPE_LOGI("tape end");
    }

    /* Assert IRQ if enabled */
    if (tape->irq_enabled && tape->irq_cb)
        tape->irq_cb(tape->irq_ctx, true);
}

/* -------------------------------------------------------------------------
 * bbc_tape_load_buffer
 *
 * Load raw bytes as a single tape block (for unit tests).
 * Sets no_motor_delay so bytes arrive immediately when motor is turned on.
 * ------------------------------------------------------------------------- */
int bbc_tape_load_buffer(bbc_tape_t *tape, const uint8_t *buf, size_t len)
{
    if (!buf || len == 0 || len > BBC_TAPE_BLOCK_MAX) return -1;

    bbc_tape_free(tape);

    /* Reset all runtime state so tests start clean */
    tape->motor_on      = false;
    tape->running       = false;
    tape->cycle_acc     = 0;
    tape->rx_full       = false;
    tape->rx_is_carrier = false;
    tape->rx_data       = 0;

    tape->blocks = (bbc_tape_block_t *)calloc(1, sizeof(bbc_tape_block_t));
    if (!tape->blocks) return -1;

    memcpy(tape->blocks[0].data, buf, len);
    tape->blocks[0].len = (uint16_t)len;
    tape->n_blocks       = 1;
    tape->cur_block      = 0;
    tape->cur_pos        = 0;
    tape->no_motor_delay = true;   /* unit test: no 200ms startup delay */
    return 0;
}

/* -------------------------------------------------------------------------
 * bbc_tape_append_buffer
 *
 * Append a raw byte buffer as an additional tape block.
 * Must be called after bbc_tape_load_buffer (or another append).
 * ------------------------------------------------------------------------- */
int bbc_tape_append_buffer(bbc_tape_t *tape, const uint8_t *buf, size_t len)
{
    if (!buf || len == 0 || len > BBC_TAPE_BLOCK_MAX) return -1;
    if (!tape->blocks) return -1;

    uint16_t n = tape->n_blocks;
    bbc_tape_block_t *newblocks = (bbc_tape_block_t *)realloc(
        tape->blocks, (size_t)(n + 1) * sizeof(bbc_tape_block_t));
    if (!newblocks) return -1;

    tape->blocks = newblocks;
    memset(&tape->blocks[n], 0, sizeof(bbc_tape_block_t));
    memcpy(tape->blocks[n].data, buf, len);
    tape->blocks[n].len = (uint16_t)len;
    tape->n_blocks       = (uint16_t)(n + 1);
    return 0;
}

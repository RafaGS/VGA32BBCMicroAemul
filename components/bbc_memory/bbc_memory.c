/*
 * bbc_memory.c — BBC Micro Model A memory map
 *
 * Model A memory map:
 *   &0000–&7FFF  16 KiB physical RAM mirrored in the CPU address window
 *   &8000–&BFFF  fixed BASIC language ROM (IC52)
 *   &C000–&FBFF  OS ROM
 *   &FC00–&FEFF  I/O (Sheila); unmapped reads return &FF
 *   &FF00–&FFFF  OS ROM, including reset/interrupt vectors
 *
 * The Model A has no ROMSEL IC76. BASIC is therefore a fixed ROM, not a
 * pageable sideways ROM. Writes to absent devices and ROM are ignored.
 *
 * Licence: zlib
 * Copyright (c) 2026 esp-beep project
 */

#include "bbc_memory.h"
#include <stdlib.h>
#include <string.h>
#ifdef ESP_PLATFORM
#  include "esp_heap_caps.h"
#endif

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */
#define RAM_SIZE          BBC_MEMORY_RAM_SIZE
#define ROM_OS_ADDR       0xC000u
#define ROM_OS_SIZE       16384u
#define IO_START          0xFC00u  /* first I/O address */
#define IO_END            0xFEFFu
#define IO_SIZE           (IO_END - IO_START + 1u)

/* --------------------------------------------------------------------------
 * Internal struct
 * -------------------------------------------------------------------------- */
typedef struct {
    /* Allocated separately in internal SRAM on ESP32. */
    uint8_t *ram;

    /* Fixed BASIC language ROM (IC52), &8000–&BFFF. */
    uint8_t  rom_basic[ROM_OS_SIZE];
    bool     basic_loaded;

    /* OS ROM (&C000–&FFFF) */
    uint8_t  rom_os[ROM_OS_SIZE];
    bool     os_loaded;

    /* I/O callback table for &FC00–&FEFF. */
    uint8_t  (*read_callback [IO_SIZE])(uint16_t, void *);
    void     (*write_callback[IO_SIZE])(uint16_t, uint8_t, void *);
    void     *read_userdata [IO_SIZE];
    void     *write_userdata[IO_SIZE];
} BBCMemoryInternal;

/* The emulator supports one active machine. Keep its hot RAM pointer in
 * internal data RAM so CPU reads/writes do not fetch the pointer field from
 * the (large, potentially PSRAM-resident) BBCMemoryInternal object. */
static BBCMemoryInternal *s_active_memory = NULL;
static uint8_t *s_active_ram = NULL;

/* --------------------------------------------------------------------------
 * Create / destroy
 * -------------------------------------------------------------------------- */
BBCMemory *bbc_memory_create(void) {
    BBCMemoryInternal *mem = (BBCMemoryInternal *)calloc(1, sizeof(BBCMemoryInternal));
    if (!mem) return NULL;

#ifdef ESP_PLATFORM
    mem->ram = (uint8_t *)heap_caps_malloc(
        RAM_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
    mem->ram = (uint8_t *)malloc(RAM_SIZE);
#endif
    if (!mem->ram) {
        free(mem);
        return NULL;
    }

    memset(mem->ram, 0x00, RAM_SIZE);
    memset(mem->rom_basic,   0xFF, ROM_OS_SIZE);
    memset(mem->rom_os,       0xFF, ROM_OS_SIZE);

    s_active_memory = mem;
    s_active_ram = mem->ram;
    return (BBCMemory *)mem;
}

void bbc_memory_destroy(BBCMemory *mem) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    if (!m) return;

    if (s_active_memory == m) {
        s_active_memory = NULL;
        s_active_ram = NULL;
    }
#ifdef ESP_PLATFORM
    heap_caps_free(m->ram);
#else
    free(m->ram);
#endif
    m->ram = NULL;
    free(m);
}

/* --------------------------------------------------------------------------
 * CPU-facing read
 * -------------------------------------------------------------------------- */
uint8_t bbc_memory_read(void *mem, uint16_t addr) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    if (!m) return 0xFF;

    /* Model A has 16 KiB physically installed. The absent upper bank mirrors
     * &0000–&3FFF at &4000–&7FFF on the CPU bus. MOS 1.20 relies on this alias
     * during reset-time RAM clearing/detection (e.g. $4001 aliases $0001).
     * The real allocation remains exactly 16 KiB. */
    if (addr < 0x8000u) {
        uint16_t ram_addr = addr & (RAM_SIZE - 1u);
        if (m == s_active_memory && s_active_ram)
            return s_active_ram[ram_addr];
        return m->ram[ram_addr];
    }

    /* Fixed BASIC language ROM: &8000–&BFFF */
    if (addr < 0xC000u) {
        return m->basic_loaded ? m->rom_basic[addr - 0x8000u] : 0xFF;
    }

    /* OS ROM: &C000–&FBFF */
    if (addr < IO_START)
        return m->os_loaded ? m->rom_os[addr - 0xC000u] : 0xFF;

    /* I/O: &FC00–&FEFF. Unmapped devices leave the bus open. */
    if (addr <= IO_END) {
        unsigned idx = addr - IO_START;
        if (m->read_callback[idx])
            return m->read_callback[idx](addr, m->read_userdata[idx]);
        return 0xFF;
    }

    /* OS ROM top page: &FF00–&FFFF */
    return m->os_loaded ? m->rom_os[addr - ROM_OS_ADDR] : 0xFF;
}

/* --------------------------------------------------------------------------
 * CPU-facing write
 * -------------------------------------------------------------------------- */
void bbc_memory_write(void *mem, uint16_t addr, uint8_t value) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    if (!m) return;

    /* Mirror the upper address range onto the single physical 16 KiB bank.
     * This reproduces the aliasing used by MOS to detect a Model A at reset. */
    if (addr < 0x8000u) {
        uint16_t ram_addr = addr & (RAM_SIZE - 1u);
        if (m == s_active_memory && s_active_ram)
            s_active_ram[ram_addr] = value;
        else
            m->ram[ram_addr] = value;
        return;
    }

    /* BASIC and OS ROM are read-only. */
    if (addr < IO_START)
        return;

    /* I/O: &FC00–&FEFF. Writes to ROM at &FF00–&FFFF are ignored. */
    if (addr <= IO_END) {
        unsigned idx = addr - IO_START;
        if (m->write_callback[idx])
            m->write_callback[idx](addr, value, m->write_userdata[idx]);
    }
}

/* --------------------------------------------------------------------------
 * ROM loading: BASIC at &8000 or OS at &C000.
 * -------------------------------------------------------------------------- */
void bbc_memory_load_rom(BBCMemory *mem, const uint8_t *romData,
                          uint32_t romSize, uint16_t startAddr) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    if (!m || !romData || romSize == 0) return;

    if (startAddr == 0x8000u && romSize <= ROM_OS_SIZE) {
        memset(m->rom_basic, 0xFF, ROM_OS_SIZE);
        memcpy(m->rom_basic, romData, romSize);
        m->basic_loaded = true;
    } else if (startAddr == 0xC000u && romSize <= ROM_OS_SIZE) {
        memset(m->rom_os, 0xFF, ROM_OS_SIZE);
        memcpy(m->rom_os, romData, romSize);
        m->os_loaded = true;
    }
}

/* --------------------------------------------------------------------------
 * I/O callback registration
 * -------------------------------------------------------------------------- */
void bbc_memory_set_read_callback(BBCMemory *mem, uint16_t addr,
    uint8_t (*callback)(uint16_t, void *), void *userData) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    if (!m || addr < IO_START || addr > IO_END) return;
    unsigned idx = addr - IO_START;
    if (idx >= IO_SIZE) return;
    m->read_callback[idx]  = callback;
    m->read_userdata[idx]  = userData;
}

void bbc_memory_set_write_callback(BBCMemory *mem, uint16_t addr,
    void (*callback)(uint16_t, uint8_t, void *), void *userData) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    if (!m || addr < IO_START || addr > IO_END) return;
    unsigned idx = addr - IO_START;
    if (idx >= IO_SIZE) return;
    m->write_callback[idx] = callback;
    m->write_userdata[idx] = userData;
}

void bbc_memory_set_range_callbacks(BBCMemory *mem,
    uint16_t base, uint16_t len,
    uint8_t (*rd)(uint16_t, void *),
    void    (*wr)(uint16_t, uint8_t, void *),
    void    *userData) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    uint32_t end = (uint32_t)base + len;
    if (!m || base < IO_START || end > IO_END + 1u) return;
    for (uint32_t a = base; a < end; a++) {
        unsigned idx = a - IO_START;
        m->read_callback [idx] = rd;
        m->write_callback[idx] = wr;
        m->read_userdata [idx] = userData;
        m->write_userdata[idx] = userData;
    }
}

/* --------------------------------------------------------------------------
 * Direct RAM access (for video subsystem DMA)
 * -------------------------------------------------------------------------- */
uint8_t *bbc_memory_get_ram(BBCMemory *mem) {
    BBCMemoryInternal *m = (BBCMemoryInternal *)mem;
    return m ? m->ram : NULL;
}

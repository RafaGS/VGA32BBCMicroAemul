#ifndef BBC_MEMORY_H
#define BBC_MEMORY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BBCMemory BBCMemory;

/* Standard BBC Micro Model A has one 16 KiB RAM bank. */
#define BBC_MEMORY_RAM_SIZE 16384u

/* Create / destroy (heap-allocated) */
BBCMemory *bbc_memory_create(void);
void       bbc_memory_destroy(BBCMemory *mem);

/* CPU-facing read/write — first arg is void* to match BBCMemoryRead/Write */
uint8_t    bbc_memory_read (void *mem, uint16_t addr);
void       bbc_memory_write(void *mem, uint16_t addr, uint8_t value);

/*
 * Load the fixed BASIC language ROM at &8000 or OS ROM at &C000.
 */
void bbc_memory_load_rom(BBCMemory *mem, const uint8_t *romData,
                          uint32_t romSize, uint16_t startAddr);

/*
 * I/O callbacks — address must be in &FC00–&FEFF (Sheila).
 * read_userdata and write_userdata are stored independently so a single
 * device object can be passed to both without conflict.
 */
void bbc_memory_set_read_callback (BBCMemory *mem, uint16_t addr,
    uint8_t (*callback)(uint16_t, void *), void *userData);
void bbc_memory_set_write_callback(BBCMemory *mem, uint16_t addr,
    void (*callback)(uint16_t, uint8_t, void *), void *userData);

/* Register the same rd/wr pair for a contiguous address range */
void bbc_memory_set_range_callbacks(BBCMemory *mem,
    uint16_t base, uint16_t len,
    uint8_t (*rd)(uint16_t, void *),
    void    (*wr)(uint16_t, uint8_t, void *),
    void    *userData);

/* Direct pointer to the 16 KB Model A RAM bank (for video access). */
uint8_t *bbc_memory_get_ram(BBCMemory *mem);

#ifdef __cplusplus
}
#endif

#endif /* BBC_MEMORY_H */

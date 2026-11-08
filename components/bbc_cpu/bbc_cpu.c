
#include "bbc_cpu.h"
#include "vrEmu6502.h"
#include <stdlib.h>
#include <stdio.h>

/*
 * vrEmu6502 uses global (non-context) memory callbacks.
 * We support a single BBCCPU instance at a time via a static pointer.
 * This is sufficient for single-machine emulation on ESP32.
 */
static BBCCPU *s_active_cpu = NULL;

static BBCMemoryRead  s_active_read_callback = NULL;
static BBCMemoryWrite s_active_write_callback = NULL;
static void          *s_active_user_data = NULL;

struct BBCCPU {
    VrEmu6502      *cpu;
    BBCMemoryRead   readCallback;
    BBCMemoryWrite  writeCallback;
    void           *userData;
};

/* vrEmu6502 global read/write callbacks */
static uint8_t cpu_mem_read(uint16_t addr, bool isDbg)
{
    (void)isDbg;

    BBCMemoryRead callback = s_active_read_callback;
    if (callback)
        return callback(s_active_user_data, addr);

    return 0xFF;
}

static void cpu_mem_write(uint16_t addr, uint8_t value)
{
    BBCMemoryWrite callback = s_active_write_callback;
    if (callback)
        callback(s_active_user_data, addr, value);
}

BBCCPU *bbc_cpu_create(BBCMemoryRead readCallback,
                       BBCMemoryWrite writeCallback,
                       void *userData)
{
    BBCCPU *bbc = (BBCCPU *)malloc(sizeof(BBCCPU));
    if (!bbc)
        return NULL;

    bbc->readCallback = readCallback;
    bbc->writeCallback = writeCallback;
    bbc->userData = userData;

    /*
     * Register callbacks before creating vrEmu6502:
     * initialization may perform memory accesses.
     */
    s_active_cpu = bbc;
    s_active_read_callback = readCallback;
    s_active_write_callback = writeCallback;
    s_active_user_data = userData;

    bbc->cpu = vrEmu6502New(CPU_6502,
                            cpu_mem_read,
                            cpu_mem_write);

    if (!bbc->cpu) {
        s_active_cpu = NULL;
        s_active_read_callback = NULL;
        s_active_write_callback = NULL;
        s_active_user_data = NULL;
        free(bbc);
        return NULL;
    }

    return bbc;
}

void bbc_cpu_destroy(BBCCPU *cpu)
{
    if (cpu) {
        if (cpu->cpu)
            vrEmu6502Destroy(cpu->cpu);

        if (s_active_cpu == cpu) {
            s_active_cpu = NULL;
            s_active_read_callback = NULL;
            s_active_write_callback = NULL;
            s_active_user_data = NULL;
        }

        free(cpu);
    }
}

void bbc_cpu_reset(BBCCPU *cpu)
{
    if (cpu && cpu->cpu) {
        s_active_cpu = cpu;
        s_active_read_callback = cpu->readCallback;
        s_active_write_callback = cpu->writeCallback;
        s_active_user_data = cpu->userData;

        vrEmu6502Reset(cpu->cpu);
    }
}

/*
 * vrEmu6502InstCycle() executes one full instruction and returns the
 * number of clock cycles it consumed.
 */
int bbc_cpu_step(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        return (int)vrEmu6502InstCycle(cpu->cpu);

    return 0;
}

void bbc_cpu_nmi(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        *vrEmu6502Nmi(cpu->cpu) = IntRequested;
}

void bbc_cpu_irq(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        *vrEmu6502Int(cpu->cpu) = IntRequested;
}

void bbc_cpu_clear_irq(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        *vrEmu6502Int(cpu->cpu) = IntCleared;
}

uint8_t bbc_cpu_get_a(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        return vrEmu6502GetAcc(cpu->cpu);

    return 0;
}

uint8_t bbc_cpu_get_x(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        return vrEmu6502GetX(cpu->cpu);

    return 0;
}

uint8_t bbc_cpu_get_y(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        return vrEmu6502GetY(cpu->cpu);

    return 0;
}

uint8_t bbc_cpu_get_p(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        return vrEmu6502GetStatus(cpu->cpu);

    return 0;
}

uint8_t bbc_cpu_get_sp(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        return vrEmu6502GetStackPointer(cpu->cpu);

    return 0;
}

uint16_t bbc_cpu_get_pc(BBCCPU *cpu)
{
    if (cpu && cpu->cpu)
        return vrEmu6502GetPC(cpu->cpu);

    return 0;
}


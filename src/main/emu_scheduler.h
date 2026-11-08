#ifndef EMU_SCHEDULER_H
#define EMU_SCHEDULER_H

#include <stdint.h>

enum {
    EMU_CYCLES_PER_US = 2,
    EMU_CREDIT_LIMIT = 20000,
    EMU_BATCH_LIMIT = 2000,
    EMU_YIELD_INTERVAL_US = 10000
};

typedef struct {
    uint32_t last_us;
    uint32_t last_yield_us;
    int32_t credit;
} emu_scheduler_t;

static inline void emu_scheduler_init(emu_scheduler_t *s, uint32_t now)
{
    s->last_us = now;
    s->last_yield_us = now;
    s->credit = 0;
}

static inline int32_t emu_scheduler_budget(emu_scheduler_t *s, uint32_t now)
{
    uint32_t elapsed = now - s->last_us;
    s->last_us = now;
    if (elapsed > EMU_CREDIT_LIMIT / EMU_CYCLES_PER_US)
        elapsed = EMU_CREDIT_LIMIT / EMU_CYCLES_PER_US;
    s->credit += (int32_t)(elapsed * EMU_CYCLES_PER_US);
    if (s->credit > EMU_CREDIT_LIMIT)
        s->credit = EMU_CREDIT_LIMIT;
    if (s->credit <= 0)
        return 0;
    return s->credit > EMU_BATCH_LIMIT ? EMU_BATCH_LIMIT : s->credit;
}

static inline void emu_scheduler_consume(emu_scheduler_t *s, int32_t cycles)
{
    s->credit -= cycles;
}

static inline int emu_scheduler_should_yield(const emu_scheduler_t *s, uint32_t now)
{
    return s->credit <= 0 ||
           (uint32_t)(now - s->last_yield_us) >= EMU_YIELD_INTERVAL_US;
}

static inline void emu_scheduler_yielded(emu_scheduler_t *s, uint32_t now)
{
    s->last_yield_us = now;
}

#endif

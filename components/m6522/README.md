# m6522 — MOS 6522 VIA component for ESP32/ESP-IDF

Generic MOS 6522 VIA emulation plus the BBC Micro-specific System VIA layer,
adapted from [floooh/chips](https://github.com/floooh/chips) (zlib) and
[B-em](https://github.com/stardot/b-em) (GPL-2.0).

## Files

| File | Description |
|---|---|
| `include/m6522.h` | Generic 6522 API — struct, register constants, callbacks |
| `m6522.c` | Generic 6522 implementation (timers, ports, edge detection, IRQ) |
| `include/bbc_sysvia.h` | System VIA BBC-specific layer |
| `bbc_sysvia.c` | Model A IC32 latch, keyboard, sound, VSYNC |

## BBC Micro VIA register map

### System VIA (IC3) — &FE40–&FE4F

| Offset | Read | Write |
|--------|------|-------|
| &FE40 | IRB / input reg B | ORB (→ IC32 latch via PB0-PB3) |
| &FE41 | IRA / slow data bus | ORA (→ keyboard col / SN76489) |
| &FE42 | DDRB | DDRB |
| &FE43 | DDRA | DDRA |
| &FE44 | T1CL (clears T1 IRQ) | T1CL latch |
| &FE45 | T1CH | T1CH (loads + starts timer) |
| &FE46 | T1LL | T1LL |
| &FE47 | T1LH | T1LH |
| &FE48 | T2CL (clears T2 IRQ) | T2CL latch |
| &FE49 | T2CH | T2CH (loads + starts timer) |
| &FE4A | SR | SR |
| &FE4B | ACR | ACR |
| &FE4C | PCR | PCR |
| &FE4D | IFR | IFR (write 1 to clear bits) |
| &FE4E | IER \| 0x80 | IER (bit 7: 1=set, 0=clear) |
| &FE4F | IRA (no handshake) | ORA (no handshake) |

The standard Model A profile has no User VIA. Its address range is unmapped and
reads return the open-bus value `&FF`.

## Addressable latch IC32 (System VIA Port B)

| Bit | Name | Function |
|-----|------|----------|
| 0 | SOUND_WE | SN76489 write enable (active LOW) |
| 1 | SPEECH_RD | Speech chip read (stub) |
| 2 | SPEECH_WR | Speech chip write (stub) |
| 3 | KB_AUTOSCAN | Keyboard auto-scan enable |
| 4 | VIDEO_C0 | C0 address-wrap control |
| 5 | VIDEO_C1 | C1 address-wrap control |
| 6 | CAPS_LED | Caps Lock LED (active LOW) |
| 7 | SHIFT_LED | Shift Lock LED (active LOW) |

## Key signal routing

```
6845 CRTC VSYNC  → bbc_sysvia_vsync()    → CA1 → IFR bit 1 (CA1) → IRQ
System VIA IRQ   → bbc_sysvia cb.irq()   → 6502 /IRQ
```

The Model A profile does not install a User VIA, ADC, joystick interface, or
WD1770, so those devices do not generate IRQ/NMI signals.

## Usage

```c
#include "bbc_sysvia.h"
static bbc_sysvia_t  sysvia;

/* --- System VIA setup --- */
static void my_sound_write(void *ctx, uint8_t data) {
    sn76489_write(data);
}
static uint8_t my_kbd_read(void *ctx, uint8_t col) {
    return keyboard_scan_column(col);
}
static void my_sysvia_irq(void *ctx, bool state) {
    cpu_set_irq(state);
}

bbc_sysvia_callbacks_t sv_cb = {
    .sound_write   = my_sound_write,
    .keyboard_read = my_kbd_read,
    .latch_changed = NULL,   /* or implement LED/screen updates */
    .irq           = my_sysvia_irq,
    .user_ctx      = NULL,
};
bbc_sysvia_init(&sysvia, &sv_cb);

/* --- Tick once per 1 MHz BBC clock cycle --- */
void bbc_tick(void) {
    bbc_sysvia_tick(&sysvia, 1);
}

/* --- VSYNC from 6845 --- */
void bbc_vsync(bool state) {
    bbc_sysvia_vsync(&sysvia, state);
}

/* --- Memory map reads/writes --- */
uint8_t bbc_read_sysvia(uint16_t addr) {
    return bbc_sysvia_read(&sysvia, addr & 0x0F);
}
void bbc_write_sysvia(uint16_t addr, uint8_t val) {
    bbc_sysvia_write(&sysvia, addr & 0x0F, val);
}
```

## References

- MOS 6522 datasheet: https://www.westerndesigncenter.com/wdc/documentation/w65c22.pdf
- BBC Micro Advanced User Guide, Chapter 23 (System VIA)
- floooh/chips: https://github.com/floooh/chips/blob/master/chips/m6522.h
- B-em sysvia.c: https://github.com/stardot/b-em/blob/master/src/sysvia.c
- Timer behaviour: http://forum.6502.org/viewtopic.php?f=4&t=2901

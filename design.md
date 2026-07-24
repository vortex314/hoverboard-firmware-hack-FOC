# Limero Protocol Integration Design

## Overview

This document describes how the Limero serial protocol (`CONTROL_LIMERO` / `FEEDBACK_LIMERO`)
integrates into the hoverboard firmware's motor control pipeline. It documents the architecture,
the changes made, and the reasoning behind the integration approach.

---

## Input Data Pipeline

The motor control firmware uses a layered input processing pipeline:

```
                    ┌─────────────────┐
                    │  Hardware IRQ    │  DMA1_Channel1 (ADC ~16 kHz)
                    │  bldc.c          │  ← actual motor PWM application
                    └────────┬────────┘
                             │ pwml, pwmr (volatile int)
                             │
┌──────────────────────────────────────────────────────────────┐
│  main() loop (~5 ms cycle)                                   │
│                                                              │
│  readCommand()                                               │
│    ├── readInputRaw()       populate input1[inIdx].raw       │
│    │                        populate input2[inIdx].raw       │
│    ├── calcInputCmd()       raw → cmd (typ/min/mid/max)      │
│    └── handleTimeout()      input-loss detection             │
│                                                              │
│  rateLimiter → low-pass filter → mixer → pwml/pwmr           │
│                                                              │
│  safety checks: motor enable, error codes, temp, battery     │
└──────────────────────────────────────────────────────────────┘
```

Each control variant (ADC, Serial, PPM, PWM, Nunchuk, iBUS, Limero) provides its
own `readInputRaw()` case that populates `input1[inIdx].raw` and `input2[inIdx].raw`.
The pipeline then applies scaling, filtering, and rate limiting uniformly.

---

## Limero Architecture

### What VARIANT_USART + FEEDBACK_LIMERO activate

| Component         | Config key                | Effect                                  |
|-------------------|---------------------------|-----------------------------------------|
| USART2 init       | `CONTROL_LIMERO`          | GPIO PA2/PA3, DMA Ch6(RX)+Ch7(TX), IRQs |
| RX data path      | `CONTROL_LIMERO`          | `usart2_rx_check()` → `handle_rxd()`    |
| TX feedback path  | `FEEDBACK_LIMERO`         | `get_txd()` → `HAL_UART_Transmit_DMA()` |
| Input pipeline    | `PRI_INPUT1`/`PRI_INPUT2` | type=3 (auto-detect→type-2 mid-resting) |

### RX data flow

```
USART2 RX DMA (circular, 64 bytes)
    │
    ▼ (IDLE line interrupt)
usart2_rx_check()           [util.c, called from USART2_IRQHandler]
    │
    ▼ (CONTROL_LIMERO path)
handle_rxd(buffer, size)    [external, provided by Limero protocol library]
    │
    ▼ (parses CBOR HoverboardRequest)
limero_steer / limero_speed [volatile globals, written by handle_rxd]
    │
    ▼ (main loop)
readInputRaw()              [reads globals → input1[0].raw / input2[0].raw]
    │
    ▼
calcInputCmd()              [type-2 passthrough: raw → cmd 1:1]
```

### TX data flow

```
main loop (every 40 cycles ≈ 200ms)
    │
    ▼
get_txd(&txd)               [external, produces CBOR-encoded telemetry]
    │
    ▼
HAL_UART_Transmit_DMA()     [DMA1_Channel7, non-blocking]
```

---

## Changes Made

### 1. Fixed missing interrupt handlers (`Src/stm32f1xx_it.c`)

**Problem:** The ISR functions `DMA1_Channel6_IRQHandler`, `DMA1_Channel7_IRQHandler`,
and `USART2_IRQHandler` were guarded by `#if defined(DEBUG_SERIAL_USART2) || ...`
conditions that did not include `CONTROL_LIMERO` or `FEEDBACK_LIMERO`.

The initialization code (`setup.c`, `util.c`) enabled these interrupts when
`CONTROL_LIMERO` was defined, but the handlers were compiled out. Any received
data or DMA completion would vector to `Default_Handler` (infinite loop).

**Fix:** Added `|| defined(CONTROL_LIMERO) || defined(FEEDBACK_LIMERO)` to the
three guard conditions at lines 279, 294, and 336.

### 2. Added Limero-to-pipeline shared variables (`Src/util.c`)

Three volatile globals bridge the external `handle_rxd()` (which runs in ISR
context via `usart2_rx_check`) and the main-loop `readInputRaw()`:

```c
volatile int16_t limero_steer       = 0;   // [-1000, 1000] steering
volatile int16_t limero_speed       = 0;   // [-1000, 1000] speed/throttle
volatile uint8_t  limero_data_fresh = 0;   // set by handle_rxd, cleared by readInputRaw
```

`volatile` is required because these are written in ISR context and read in
main-loop context — the compiler must not cache or reorder accesses.

### 3. Added CONTROL_LIMERO case in `readInputRaw()` (`Src/util.c`)

Reads the shared variables and resets the serial timeout when fresh data is
detected:

```c
#ifdef CONTROL_LIMERO
    input1[0].raw = limero_steer;
    input2[0].raw = limero_speed;
    if (limero_data_fresh) {
        timeoutCntSerial_L = 0;
        timeoutFlgSerial_L = 0;
        limero_data_fresh = 0;
    }
#endif
```

Uses array index `[0]` (primary input slot) since `inIdx` is always 0 for
single-input configurations. This mirrors how `VARIANT_TRANSPOTTER` directly
writes to fixed array positions.

### 4. Propagated serial timeout for CONTROL_LIMERO (`Src/util.c`)

The `timeoutFlgSerial` global was not being set for the Limero path, meaning
no warning beeps would sound on data loss. Added `|| defined(CONTROL_LIMERO)`
to the existing propagation condition at line 980-982.

---

## Integration Contract for External `handle_rxd()`

The external Limero protocol handler must:

1. **Declare** the shared variables:
   ```c
   extern volatile int16_t limero_steer;
   extern volatile int16_t limero_speed;
   extern volatile uint8_t  limero_data_fresh;
   ```

2. **On valid HoverboardRequest:**
   ```c
   limero_steer = request.steer;    // map to [-1000, 1000]
   limero_speed = request.speed;    // map to [-1000, 1000]
   limero_data_fresh = 1;
   ```

3. **On invalid data or timeout:** do nothing — the firmware handles timeout
   via `handleTimeout()` which increments `timeoutCntSerial_L` each main loop
   iteration (~5ms). After `SERIAL_TIMEOUT` (160 = ~0.8s) of no fresh data,
   `timeoutFlgSerial` triggers a 3-beep warning.

---

## Why Not Write Directly to pwml/pwmr

Direct writes to `pwml`/`pwmr` bypass:
- **Motor enable gate:** `enable == 0` blocks PWM output. Writing directly
  could drive motors before the safety gate opens.
- **Rate limiter:** prevents sudden acceleration spikes.
- **Low-pass filter:** smooths jerky serial commands.
- **Timeout detection:** no warning on serial link loss.
- **Input calibration:** min/mid/max scaling.

Writing through the pipeline gives all these protections for free.

---

## File Index

| File                  | Changes                          |
|-----------------------|----------------------------------|
| `Src/stm32f1xx_it.c`  | 3 guard conditions extended      |
| `Src/util.c`          | Shared vars, readInputRaw case,  |
|                       | timeout propagation              |

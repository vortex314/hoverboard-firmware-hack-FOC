# Hoverboard FOC Firmware - Design and Control Execution

## Overview

This document describes the architecture and control execution of the Hoverboard FOC (Field Oriented Control) firmware. The firmware implements advanced motor control for dual hoverboard motors running on an STM32F1xx microcontroller, featuring three distinct control methods: Commutation, Sinusoidal, and Field Oriented Control (FOC).

**Target Hardware:** STM32F103RCT6 / GD32F103RCT6
**Control Frequency:** 16 kHz (PWM frequency)
**Main Loop Frequency:** ~200 Hz (5ms delay per iteration)

---

## System Architecture

### High-Level System Flow

```
┌─────────────────────────────────────────────────────────┐
│                  System Initialization                   │
│  - Clock Configuration                                  │
│  - GPIO, Timer, ADC Setup                               │
│  - BLDC Controller Initialization                       │
│  - Input System Initialization                          │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│                  Main Loop (≈200 Hz)                     │
│  1. Read Input Commands                                 │
│  2. Calculate Average Speed                             │
│  3. Apply Filters & Rate Limiters                       │
│  4. Execute Motor Mixing                                │
│  5. Set PWM Output                                      │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│          Interrupt-Driven Control (16 kHz)              │
│  - DMA/ADC Interrupt Handler                            │
│  - Current Measurement & Protection                     │
│  - FOC Algorithm Execution                              │
│  - PWM Generation                                       │
└─────────────────────────────────────────────────────────┘
```

---

## Core Components

### 1. Main Loop (`main.c`)

The main loop runs at approximately 200 Hz (5ms interval) and orchestrates the overall control flow.

#### Main Loop Flow:

```c
while (1) {
    // Execute every 16 * DELAY_IN_MAIN_LOOP = 80ms / 16 = 5ms
    
    if (buzzerTimer - buzzerTimer_prev > 16 * DELAY_IN_MAIN_LOOP) {
        // 1. Read user input commands
        readCommand();          // Read from ADC, UART, PPM, PWM, etc.
        calcAvgSpeed();         // Calculate measured motor speed
        
        // 2. Motor enable logic (safety check)
        if (enable == 0 && input commands are small) {
            enable = 1;         // Enable motors only if inputs are near zero
        }
        
        // 3. Input processing (variant-specific logic)
        // - Hovercar brake/throttle handling
        // - Electric brake application
        // - Speed blending
        
        // 4. Signal filtering & rate limiting
        rateLimiter16(input1, rate, &steerRateFixdt);
        rateLimiter16(input2, rate, &speedRateFixdt);
        filtLowPass32(...);     // Apply low-pass filter
        
        // 5. Motor command mixing
        mixerFcn(speed, steer, &cmdR, &cmdL);
        
        // 6. Set motor PWM outputs
        pwml = cmdL;
        pwmr = cmdR;
    }
}
```

**Key Control Variables:**
- `speed`: Forward/backward command (-1000 to 1000)
- `steer`: Left/right steering command (-1000 to 1000)
- `cmdL`, `cmdR`: Final motor commands to motors
- `pwml`, `pwmr`: PWM output signals
- `enable`: Motor enable flag (0 = off, 1 = on)

---

### 2. Interrupt-Driven Control (`bldc.c`)

The DMA interrupt runs at 16 kHz (every 62.5 µs) and handles real-time motor control.

#### DMA Interrupt Handler (`DMA1_Channel1_IRQHandler`):

```
DMA Interrupt (16 kHz)
├─ ADC Offset Calibration (first 2000 cycles)
├─ Battery Voltage Filtering
├─ Read Motor Currents
│  ├─ Left motor: Phase A, Phase B, DC link
│  └─ Right motor: Phase B, Phase C, DC link
├─ Current Protection (Level 2 - Current Chopping)
│  └─ Disable PWM if current exceeds I_DC_MAX
├─ Buzzer Control
├─ BLDC_Controller Execution (FOC Algorithm)
│  ├─ Read Hall Sensor Inputs
│  ├─ Set Motor Enable & Control Mode
│  ├─ Set PWM Duty Cycles
│  └─ Write PWM Values to Timer Registers
└─ Process Outputs & Error Codes
```

**Critical Timing Notes:**
- All operations must complete within 62.5 µs
- No floating-point operations (all fixed-point)
- Hall sensors read directly from GPIO registers for low latency
- PWM updates occur directly via timer CCR (capture-compare registers)

---

### 3. BLDC Controller (`BLDC_controller.c`)

The BLDC controller is auto-generated from a Simulink model and implements the Field Oriented Control (FOC) algorithm.

#### Control Modes:

1. **VOLTAGE MODE (VLT_MODE)** - Direct voltage control
   - Applies constant voltage to motors
   - Fast response for robotics applications
   - No closed-loop feedback

2. **SPEED MODE (SPD_MODE)** - Closed-loop speed control
   - Maintains target RPM despite load changes
   - PI controller rejecting disturbances
   - Best for constant speed applications

3. **TORQUE MODE (TRQ_MODE)** - Current-based torque control
   - Enables freewheeling when torque = 0
   - Best for applications with human operator
   - Motor efficiency: 100% → Efficiency varies with load

#### FOC Algorithm Features:

- **Park Transform:** Converts 3-phase currents to D-Q (direct-quadrature) components
- **PI Controllers:** Separate controllers for D-Q current loops
- **Inverse Park Transform:** Converts D-Q voltages back to 3-phase signals
- **Space Vector PWM (SVPWM):** Generates optimal PWM signals for 3-phase motor
- **Field Weakening:** Extends maximum speed range above rated frequency
- **Phase Advance:** Phase angle adjustment for speed optimization

#### Fixed-Point Arithmetic:

All calculations use fixed-point format (e.g., fixdt(0,16,15)) for efficiency:
- Avoids floating-point hardware (not available in STM32F1)
- Reduces computation time
- Ensures deterministic behavior
- Scale factor specified in BLDC_controller_data.c

**Example:** Current in Amps = ADC_value × 50 (A2BIT_CONV factor)

---

## Input Processing System

### Supported Input Variants:

| Variant | Input Method | Usage | Default Pins |
|---------|--------------|-------|--------------|
| VARIANT_ADC | 2 Potentiometers | Robotics/Testing | PA0, PC3 (USART2 cable) |
| VARIANT_USART | Serial Commands | Computer control | USART3 |
| VARIANT_PPM | RC Sum Signal | RC Remote | PA3 (USART2) or PB10 (USART3) |
| VARIANT_PWM | RC 2-Channel PWM | RC Remote | PA2/PA3 (USART2) or PB10/PB11 (USART3) |
| VARIANT_IBUS | Flysky iBUS | RC Remote | USART2 |
| VARIANT_NUNCHUK | Wii Nunchuk I2C | Hand Control | I2C (PB10/PB11) |
| VARIANT_HOVERCAR | Dual Pedals + Buttons | Hovercar/Wheelchair | ADC inputs |
| VARIANT_TRANSPOTTER | Distance + Steering | Autonomous following | ADC + Distance sensor |

### Input Reading Flow:

```c
readCommand() {
    switch(inIdx) {
        case CONTROL_ADC:           // Read from ADC
            input[inIdx].cmd = (adc_buffer.val - min) / (max - min) * 1000;
            
        case CONTROL_SERIAL:        // Read from UART
            input[inIdx].cmd = received_serial_data;
            
        case CONTROL_PPM:           // Read from PPM
            input[inIdx].cmd = ppm_captured_value[channel];
            
        case CONTROL_PWM:           // Read from PWM
            input[inIdx].cmd = pwm_captured_ch_value;
    }
}
```

**Input Timeout Protection:**
- If no input received for >20 cycles: `timeoutFlgGen = 1`
- Motors automatically disabled on timeout
- Safety feature prevents runaway

---

## Motor Command Mixing

### Standard Mixer Function:

The mixer combines speed and steering commands into individual motor commands:

```c
mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL);

// Typical equations:
// cmdL = speed - steer
// cmdR = speed + steer
```

This produces:
- **Forward:** Both motors forward equally
- **Backward:** Both motors backward equally
- **Left Turn:** Right motor faster than left
- **Right Turn:** Left motor faster than right

### Tank Steering (Alternative):

```c
#ifdef TANK_STEERING
    cmdL = steer;    // Left stick controls left motor
    cmdR = speed;    // Right stick controls right motor
#endif
```

---

## Signal Processing Chain

### 1. Rate Limiter

Limits the rate of change of input commands to prevent sudden jerks:

```c
rateLimiter16(input_cmd, rate, &output_fixdt);
// Parameters:
// - rate: RATE = 20 (adjustable, units per cycle)
// - output_fixdt: Fixed-point output
```

### 2. Low-Pass Filter

Smooths command signals to reduce noise:

```c
filtLowPass32(input, FILTER_COEFFICIENT, &output_fixdt);
// FILTER_COEFFICIENT typically 0.05 (5% new value, 95% old)
// Implemented as: output = 0.95 * previous + 0.05 * input
```

### 3. Conversion to Integer

```c
steer = (int16_t)(steerFixdt >> 16);  // Extract integer part
speed = (int16_t)(speedFixdt >> 16);
```

---

## Motor Control Flow Diagram

```
┌──────────────────────────────────────────────┐
│ Main Loop: Read Input & Process Commands    │
│ Frequency: ~200 Hz (5ms)                     │
└────────────────┬─────────────────────────────┘
                 │
                 ├─→ readCommand()          // ADC, UART, PPM, PWM, etc.
                 ├─→ calcAvgSpeed()         // Measure motor RPM
                 ├─→ Rate Limiter           // Limit acceleration
                 ├─→ Low-Pass Filter        // Noise reduction
                 ├─→ Motor Mixer            // speed + steer → cmdL, cmdR
                 └─→ Set pwml, pwmr         // Store commands
                 
┌──────────────────────────────────────────────┐
│ DMA Interrupt: FOC Motor Control             │
│ Frequency: 16 kHz (62.5 µs)                 │
└────────────────┬─────────────────────────────┘
                 │
                 ├─→ Read ADC (Currents)
                 ├─→ Read Hall Sensors
                 ├─→ FOC Algorithm
                 │   ├─ Clarke Transform (3-phase → α-β)
                 │   ├─ Park Transform (α-β → D-Q)
                 │   ├─ PI Controllers (Id, Iq)
                 │   ├─ Inverse Park (D-Q → α-β)
                 │   ├─ Inverse Clarke (α-β → 3-phase)
                 │   └─ SVPWM (Generate PWM)
                 ├─→ Current Protection
                 └─→ Update PWM Output
```

---

## Current Measurement and Protection

### Current Measurement:

**Dual-Layer Protection:**
1. **Level 1 (Software):** PI controller limits (I_MOT_MAX per motor)
2. **Level 2 (Hardware):** Absolute current limit (I_DC_MAX)

```c
// Level 1 - PI Controller Saturation
rtP_Left.i_max = I_MOT_MAX * A2BIT_CONV;   // Soft limit

// Level 2 - Current Chopping
if(ABS(curL_DC) > curDC_max || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;  // Disable PWM
} else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;   // Enable PWM
}
```

### Measured Currents:

| Motor | Phase A | Phase B | Phase C | DC Link |
|-------|---------|---------|---------|---------|
| Left | rlA | rlB | (calculated) | dcl |
| Right | (calculated) | rrB | rrC | dcr |

**Current Calculation:**
```c
curL_phaA = offsetrlA - adc_buffer.rlA;
curL_phaB = offsetrlB - adc_buffer.rlB;
curL_DC   = offsetdcl - adc_buffer.dcl;
```

---

## Timing Architecture

### Time Scales:

```
16 kHz  ├─ DMA Interrupt (FOC algorithm, PWM update)
        │  └─ Executes every 62.5 µs
        │
~1 kHz  ├─ Battery voltage filtering (every 1000 DMA cycles)
        │
200 Hz  ├─ Main loop iteration
        │  └─ Executes every 5 ms (DELAY_IN_MAIN_LOOP)
        │
~50 Hz  ├─ Buzzer control (via pattern modulation)
        │
Varies  └─ Input sampling (depends on input method)
           ├─ ADC: Continuous
           ├─ UART: On reception
           ├─ PPM: Rising edge triggered
           └─ PWM: Rising/falling edge triggered
```

### Critical Real-Time Constraints:

1. **DMA Interrupt (62.5 µs deadline):**
   - Cannot perform I/O operations (Serial, I2C)
   - No function calls to slow operations
   - Only fast arithmetic and GPIO register access

2. **Main Loop (5 ms deadline):**
   - Can perform moderate I/O operations
   - Serial communication acceptable
   - Input reading from shared buffers

---

## Hall Sensor Decoding

### Hall Sensor Inputs:

3 Hall sensors per motor (U, V, W phases) provide 6 possible states:
- Each state indicates a specific rotor position sector
- States repeat every 60° of electrical rotation
- Used to commutate current in correct motor phase

```c
// Left Motor Hall Sensors
uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

// Hall state = (U << 2) | (V << 1) | W
// Valid states: 1, 2, 3, 4, 5, 6 (0 and 7 are invalid)
```

### Rotor Position Estimation:

The FOC algorithm uses Hall sensors to:
1. Determine exact rotor position (6 sectors)
2. Properly orient the D-Q reference frame
3. Apply correct phase currents via Park Transform

---

## Motor Enable Logic (Safety)

### Enable Sequence:

```c
if (enable == 0 &&                          // Motors currently disabled
    !rtY_Left.z_errCode &&                  // No left motor error
    !rtY_Right.z_errCode &&                 // No right motor error
    ABS(input1[inIdx].cmd) < 50 &&          // Input 1 near zero
    ABS(input2[inIdx].cmd) < 50) {          // Input 2 near zero
    
    // Only enable if user provides small input (safety verification)
    beepShort(6);           // Beep 2x indicating enable
    beepShort(4);
    steerFixdt = 0;         // Reset filter states
    speedFixdt = 0;
    enable = 1;             // Enable motors
}
```

### Disable Sequence:

```c
// Automatic disable if:
if (enable == 1 && 
    (rtY_Left.z_errCode || rtY_Right.z_errCode ||    // Error detected
     timeoutFlgGen || timeoutFlgSerial)) {            // Input timeout
    enable = 0;             // Disable motors
    pwml = 0;               // Clear commands
    pwmr = 0;
}
```

---

## Error Handling

### Error Codes (from rtY_Left.z_errCode, rtY_Right.z_errCode):

| Code | Meaning | Action |
|------|---------|--------|
| 0 | No error | Normal operation |
| 1 | Hall sensor error | Motors disabled |
| 2 | Overcurrent | Current chopping active |
| 3 | Overtemperature | Motors disabled |
| 4+ | Other errors | Motors disabled |

### Error Detection:

```c
// Verify error codes before enabling or operating
enableFin = enable && !rtY_Left.z_errCode && !rtY_Right.z_errCode;
```

---

## PWM Generation and Output

### PWM Timer Configuration:

- **Frequency:** 16 kHz (PWM_FREQ)
- **Resolution:** 2000 counts per period (64MHz / 2 / 16kHz)
- **Dead-Time:** 48 ticks (1.5 µs deadtime for gate drivers)

### Timer Hierarchy:

```
TIM1 (Right Motor)
├─ CC1 (Phase U High)
├─ CC2 (Phase V High)
└─ CC3 (Phase W High)

TIM8 (Left Motor) - Gated by TIM1
├─ CC1 (Phase U High)
├─ CC2 (Phase V High)
└─ CC3 (Phase W High)
```

### PWM Output:

```c
// Three-phase PWM (one example sector):
// Phase U: 80% duty (1600/2000)
// Phase V: 40% duty (800/2000)
// Phase W: 20% duty (400/2000)

LEFT_TIM->CCR1 = 1600;   // Update phase U PWM
LEFT_TIM->CCR2 = 800;    // Update phase V PWM
LEFT_TIM->CCR3 = 400;    // Update phase W PWM
```

### PWM Margin for FOC:

```c
// Adjust PWM margin for proper current measurement timing
if (rtP_Left.z_ctrlTypSel == FOC_CTRL) {
    pwm_margin = 110;   // Reserve time for ADC conversion
} else {
    pwm_margin = 0;     // No margin needed for commutation
}
```

---

## Battery and Temperature Monitoring

### Battery Voltage Filtering:

```c
// Every 1 ms (during 1000th DMA cycle)
filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
batVoltage = (int16_t)(batVoltageFixdt >> 16);
```

**Filter Coefficient:** BAT_FILT_COEF = 655 (0.01 in fixed-point)
- Very slow filter (tau ≈ 100 ms)
- Removes voltage ripple from PWM switching

### Battery Protection Levels:

```
Level 5 (Green):   3.90 V/cell - Normal operation
Level 4 (Yellow):  3.80 V/cell - No action
Level 3 (Blink):   3.70 V/cell - Visual warning
Level 2 (Red):     3.60 V/cell - Gentle beep, user should stop
Level 1 (Blink):   3.50 V/cell - Fast beep, battery critical
Dead (Off):        3.37 V/cell - Forced poweroff (auto-recovery)
```

### Temperature Monitoring:

- Internal STM32 temperature sensor
- Calibration required for accuracy (highly temperature dependent)
- Can trigger motor disable if overheating detected

---

## Advanced Features

### Field Weakening:

Extends motor speed range beyond rated frequency:

```c
// Linear interpolation based on speed
if (speed > FIELD_WEAK_LO) {
    field_weak = (speed - FIELD_WEAK_LO) / (FIELD_WEAK_HI - FIELD_WEAK_LO);
    field_weak = MIN(field_weak, FIELD_WEAK_MAX);
}
```

**Caution:** ⚠️ Increases power consumption and motor heating

### Standstill Hold:

Maintains motor position when commanded to stop:

```c
#ifdef STANDSTILL_HOLD_ENABLE
standstillHold();   // Applies holding current when speed ≈ 0
#endif
```

### Cruise Control (HOVERCAR Variant):

Maintains constant speed autonomously:

```c
cruiseControl(rtP_Left.b_cruiseCtrlEna);  // Enable/disable
// Once engaged at constant speed, maintains that speed
```

### Electric Brake:

Converts kinetic energy into magnetic braking:

```c
#ifdef ELECTRIC_BRAKE_ENABLE
electricBrake(speedBlend, MultipleTapBrake.b_multipleTap);
// Braking proportional to speed for smooth deceleration
#endif
```

---

## Control Mode Selection

### Selecting Control Type:

In `config.h`:
```c
#define CTRL_TYP_SEL    FOC_CTRL    // FOC_CTRL, SIN_CTRL, or COM_CTRL
```

### Selecting Control Mode (FOC only):

In `main.c`:
```c
extern uint8_t ctrlModReq;

// Set to one of:
// - VLT_MODE (Voltage mode)
// - SPD_MODE (Speed mode)
// - TRQ_MODE (Torque mode)

rtU_Left.z_ctrlModReq = ctrlModReq;
rtU_Right.z_ctrlModReq = ctrlModReq;
```

**Note:** Control mode can be switched at runtime via serial commands or configuration.

---

## Data Flow Summary

```
Physical Inputs (ADC, UART, PPM, PWM, I2C)
    ↓
Input Reading System (readCommand)
    ↓
Input Limitations & Scaling
    ↓
Main Loop (≈200 Hz)
├─ Rate Limiting
├─ Low-Pass Filtering
└─ Motor Mixing
    ↓
pwml, pwmr Variables (stored in RAM)
    ↓
DMA Interrupt (16 kHz)
├─ Read Current Measurements
├─ Read Hall Sensors
├─ Execute FOC Algorithm
│  └─ Generate Phase Voltages
├─ Check Current Limits
└─ Update PWM CCR Registers
    ↓
Motor Drivers (receive PWM signals)
    ↓
3-Phase Motor Windings
    ↓
Motor Rotation
    ↓
Back-EMF detected by Hall sensors (closes the loop)
```

---

## Configuration Parameters

### Key Configuration Parameters (config.h):

```c
// Control Settings
#define CTRL_TYP_SEL        FOC_CTRL      // Control type
#define CTRL_MOD_REQ        TRQ_MODE      // Control mode

// Motor Parameters
#define I_DC_MAX            41            // Max DC current (A)
#define N_MOT_MAX           1000          // Max RPM

// Filter & Rate Limiting
#define RATE                20            // Rate limit (units/cycle)
#define FILTER              0.05          // Low-pass filter coefficient
#define DELAY_IN_MAIN_LOOP  5             // Main loop delay (ms)

// Input Configuration
#define PWM_FREQ            16000         // PWM frequency (Hz)
#define TIMEOUT             20            // Input timeout cycles
#define A2BIT_CONV          50            // ADC to current conversion

// Battery
#define BAT_CELLS           10            // Battery cells
#define BAT_CALIB_ADC       1492          // Calibration value
#define BAT_CALIB_REAL_VOLTAGE 3970       // Real voltage (×100)

// Features
#define FIELD_WEAK_ENA      0             // Enable field weakening
#define STANDSTILL_HOLD_ENABLE 0          // Enable standstill hold
#define ELECTRIC_BRAKE_ENABLE 0           // Enable electric brake
```

---

## Performance Characteristics

| Metric | Value | Note |
|--------|-------|------|
| Control Loop Frequency | 16 kHz | DMA interrupt |
| Main Loop Frequency | ~200 Hz | 5 ms delay |
| Control Latency | <1 ms | Input to PWM update |
| Current Measurement Rate | 16 kHz | Continuous sampling |
| Temperature Measurement | ~1 Hz | Slow filter |
| Battery Voltage Filter | ~100 ms | Very smooth |
| Hall Sensor Latency | <1 µs | Direct register read |
| FOC Algorithm Time | <50 µs | Typically 30-40 µs |
| Total DMA Interrupt Time | <60 µs | Must stay under 62.5 µs deadline |

---

## Conclusion

The Hoverboard FOC firmware implements a sophisticated real-time control system with:

1. **Deterministic Timing:** Dual-frequency architecture (16 kHz + 200 Hz) separates real-time control from user input processing
2. **Advanced Motor Control:** FOC algorithm provides smooth, efficient motor operation
3. **Safety:** Multiple protection layers (timeouts, current limits, error codes)
4. **Flexibility:** Multiple input variants and control modes via configuration
5. **Efficiency:** Fixed-point arithmetic optimized for embedded hardware

The control execution flow ensures responsive motor control while maintaining system stability through careful timing management and hierarchical interrupt prioritization.

---

**Document Version:** 1.0
**Last Updated:** August 3, 2026
**Author:** Documentation Generation System

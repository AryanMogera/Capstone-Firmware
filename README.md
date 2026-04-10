# Firmware Skeleton

STM32 firmware for a small 4-series-group battery management prototype on the
NUCLEO-F446RE / STM32F446RETx target.

The application samples pack voltages, thermistors, current, and charger
detection through ADC1 with DMA, runs a simple BMS state machine, controls
charge/discharge enable outputs, and streams telemetry over USART2.

This is development firmware. Validate limits, sensing hardware, output drivers,
and fault handling before connecting real cells, chargers, or loads.

## Target

- Board: NUCLEO-F446RE
- MCU: STM32F446RETx
- Toolchain: STM32CubeIDE managed build / ARM GCC
- Linker script: `STM32F446RETX_FLASH.ld`
- UART: USART2 on PA2/PA3 at 115200 baud

## Project Layout

- `Core/Src/main.c` - application loop, measurement conversion, telemetry, UART commands
- `Core/Src/adc_dma.c` - register-level ADC1 + DMA2 Stream0 sampling setup
- `Core/Src/bms_fsm.c` - BMS states, threshold checks, fault handling
- `Core/Src/bms_hw.c` - charge/discharge GPIO outputs and user button input
- `Core/Src/uart_vcp.c` - USART2 TX/RX line interface
- `Core/Src/timebase.c` - 1 ms SysTick timebase
- `Core/Inc/bms_config.h` - default voltage, temperature, current, and timing limits
- `Drivers/` - STM32 HAL/CMSIS headers and source from Cube
- `Firmware_skeleton.ioc` - CubeMX/CubeIDE project metadata

## Hardware Signals

### ADC Sequence

`adc_dma.c` samples 14 channels in this order:

| Index | Signal | Pin | ADC channel |
| --- | --- | --- | --- |
| 0 | V1 lowest tap | PB0 | ADC12_IN8 |
| 1 | V2 tap | PA4 | ADC12_IN4 |
| 2 | V3 tap | PA1 | ADC123_IN1 |
| 3 | V4 highest tap | PA0 | ADC123_IN0 |
| 4 | T1 | PA6 | ADC12_IN6 |
| 5 | T2 | PB1 | ADC12_IN9 |
| 6 | T3 | PC5 | ADC12_IN15 |
| 7 | T4 | PC4 | ADC12_IN14 |
| 8 | T5 | PC3 | ADC123_IN13 |
| 9 | T6 | PC2 | ADC123_IN12 |
| 10 | T7 | PC0 | ADC123_IN10 |
| 11 | T8 | PC1 | ADC123_IN11 |
| 12 | I1 current sense | PA5 | ADC12_IN5 |
| 13 | Charger detect | PA7 | ADC12_IN7 |

### GPIO

| Signal | Pin | Direction | Purpose |
| --- | --- | --- | --- |
| CHG enable | PC6 | Output | Charge path enable |
| DSG enable | PC7 | Output | Discharge path enable |
| USER button | PC13 | Input pull-up | 5 s long press toggles deep-discharge mode |
| USART2 TX | PA2 | AF7 | Virtual COM TX |
| USART2 RX | PA3 | AF7 | Virtual COM RX |

## Runtime Behavior

At startup the firmware initializes the timebase, UART2, ADC/DMA, BMS GPIO, and
BMS context. The main loop then:

1. Handles newline-terminated UART commands.
2. Samples and filters measurements every `MEASURE_PERIOD_MS`.
3. Converts tap voltages into four per-group voltages.
4. Converts thermistor readings using the configured NTC model.
5. Updates charger detect with hysteresis.
6. Steps the BMS state machine.
7. Emits JSON telemetry every `TELEMETRY_MS`.

The BMS states are:

- `BMS_INIT`
- `BMS_STANDBY`
- `BMS_MEASURE`
- `BMS_CHARGE`
- `BMS_DISCHARGE`
- `BMS_DEEP_DISCHARGE`
- `BMS_FAULT`

Fault bits:

- `FAULT_OV` - over-voltage
- `FAULT_UV` - under-voltage
- `FAULT_OT` - over-temperature
- `FAULT_OC` - over-current
- `FAULT_SENSOR` - fallback/internal sensor fault

When a fault is present, both charge and discharge outputs are turned off.

## Default Limits

Defaults live in `Core/Inc/bms_config.h`:

| Setting | Default |
| --- | --- |
| Over-voltage | 4.5 V per group |
| Under-voltage | 1.30 V per group |
| Deep under-voltage | 0.00 V per group |
| Over-temperature | 60.0 C |
| Over-current | 6.0 A |
| Measurement period | 50 ms |
| Telemetry period | 50 ms |
| Charger detect on | 2.50 V at ADC pin |
| Charger detect off | 1.80 V at ADC pin |



## Build and Flash

This repository is set up as an STM32CubeIDE project.

1. Open STM32CubeIDE.
2. Import this directory as an existing STM32CubeIDE/Eclipse project.
3. Select the `Debug` or `Release` configuration.
4. Build the project.
5. Flash/debug using an ST-LINK connected to the NUCLEO-F446RE.

Generated build output goes under `Debug/` or `Release/` and is intentionally
ignored by Git.

## CubeMX Note

`Firmware_skeleton.ioc` is useful project metadata, but the current application
contains handwritten peripheral setup, especially ADC/DMA in `Core/Src/adc_dma.c`.
Review generated changes carefully before regenerating code from CubeMX.

## Git Hygiene

The repository should track source, linker scripts, Cube project metadata, and
hardware configuration. It should not track local editor settings, build output,
or machine-specific launch/debug files.

Ignored examples:

- `.vscode/`
- `.settings/`
- `.metadata/`
- `*.launch`
- `Debug/`
- `Release/`
- compiled artifacts such as `*.elf`, `*.hex`, `*.bin`, `*.map`, `*.o`, `*.d`

# Planb Discharge Module (2026.7)

GD32F303CCT6 firmware for an intelligent motor-drive discharge-control module. The module provides bus-voltage monitoring, staged PWM discharge control, thermal and safety supervision, CAN communication, FRAM-backed configuration and event logging, and UART diagnostics.

This is a standalone discharge-control submodule associated with the PlanbPro system. It is not the PlanbPro main-controller firmware.

## Project contents

- `Core/App/` — discharge logic, protection, monitoring, CAN protocol, and event logging
- `Core/Bsp/` — GD32F303 board support and peripheral drivers
- `Core/Test/` — module-level logic and BSP tests
- `Core/Lib/` — CMSIS and GD32F30x standard peripheral library files used by the project
- `Core/User/` — startup application and interrupt handlers
- `Proj/` — GNU Arm Embedded build configuration, startup assembly, and linker script

## Build

The project targets GD32F303CCT6 (Cortex-M4F) and uses the `arm-none-eabi-gcc` toolchain. Build configuration and source lists are in `Proj/Makefile`; the linker script is `Proj/GD32F30x_FLASH.ld`.

Build outputs are generated under `Proj/build/` and are excluded from version control. Hardware behavior and safety limits require validation on the target board before deployment.

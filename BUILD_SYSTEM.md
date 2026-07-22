# Build System Notes

## Purpose
This file is the focused reference for configuring, building, flashing, and debugging the firmware targets with minimal friction.

## Build Stack
- Build generator: Ninja (via CMake presets)
- Toolchain file: firmware/cmake/gcc-arm-none-eabi.cmake
- Language standards:
  - C11
  - C++20
- CPU flags:
  - -mcpu=cortex-m4
  - -mthumb
  - -mfpu=fpv4-sp-d16
  - -mfloat-abi=hard

## CMake Target Structure
Top-level firmware/CMakeLists.txt defines two subprojects:
- bootloader target from firmware/bootloader/CMakeLists.txt
- application target from firmware/core/CMakeLists.txt

Output conversion target:
- convert_hex (ALL)
  - bootloader.hex
  - application.hex
  - application.bin

Intentionally not active:
- combined image generation pipeline (requires Python merge script that is not part of current baseline)

## Source Composition
Bootloader target includes:
- bootloader/Src/main.cpp
- bootloader/Src/syscalls.c
- bootloader/Src/sysmem.c
- core/startup/startup_stm32g473qetx.s
- drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c

Application target includes:
- core/src/main.cpp
- core/startup/startup_stm32g473qetx.s
- drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c

## Linker Regions
Bootloader linker script:
- firmware/bootloader/bootloader.ld
- FLASH origin 0x08000000 length 32K
- RAM origin 0x20000000 length 128K

Application linker script:
- firmware/core/core.ld
- FLASH origin 0x08008000 length 224K
- RAM origin 0x20000000 length 128K

## VS Code Integration
Workspace file contains:
- build tasks for build/rebuild/clean
- CubeProgrammer flash task
- cortex-debug launch configuration

Debug launch loads both ELFs:
- bootloader ELF
- application ELF

Device config:
- STM32G473QE
- SWD
- ST-Link server type
- SVD: firmware/STM32G473.svd

## Recommended Daily Flow
1. Select CMake configure preset: Debug
2. Run build task: Build project
3. Flash:
   - either CubeProg task
   - or Debug launch (loads both ELFs)
4. Validate stop at main

## Common Failure Modes
- Wrong source directory in local settings
- Case-sensitive path mismatch in CMake subdirectories
- Accidental reintroduction of STM32F4 include/define/startup references
- Bootloader constants not matching linker partitions

## Rules For Future Build Changes
- Keep flash partition values synchronized across:
  - bootloader.ld
  - core.ld
  - bootloader constants in main.cpp
  - debug/flash task assumptions
- Prefer explicit source lists when startup/system files are involved
- Add optional pipelines (like combined image generation) only when scripts are committed and validated

## Optional Future Enhancements
- Re-add combined image generation with checked-in Python tooling
- Add dedicated CMake targets for flash package artifacts
- Add CI build matrix for Debug and Release presets
- Add static analysis task integration

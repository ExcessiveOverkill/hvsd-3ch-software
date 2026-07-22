# AGENTS

## Project Snapshot
This repository contains firmware for a 3-channel high-voltage servo drive based on STM32G473.

Current architecture is split into:
- Bootloader image (bank 1, first 32 KB)
- Application image (bank 1, remaining 224 KB)
- OTA staging area (bank 2, 256 KB)

Primary goal right now is a clean, stable bring-up baseline (build, flash, debug), not application functionality.

## Repository Map (High Signal)
- firmware/: Embedded project root (CMake + linker scripts + startup + sources)
- firmware/bootloader/: Bootloader target sources and linker script
- firmware/core/: Application target sources and linker script
- firmware/drivers/CMSIS/: CMSIS headers + STM32G4 system template source
- hvsd-3ch-software.code-workspace: VS Code tasks + launch config
- .vscode/settings.json: Local CMake source directory setting
- python/: Reserved for future tooling/scripts (currently not required for core build)

## Current Bring-Up State
- STM32F413 reference files were removed.
- Shared startup assembly is STM32G473 startup from core/startup.
- Bootloader update logic is intentionally stubbed for STM32G4 flash-porting work.
- Bootloader currently chainloads application by default.
- Build emits:
  - bootloader ELF
  - application ELF
  - bootloader HEX
  - application HEX
  - application BIN
- Combined firmware image generation is intentionally not active right now.

## Flash Layout (Authoritative)
- Bootloader: 0x08000000 - 0x08007FFF (32 KB)
- Application: 0x08008000 - 0x0803FFFF (224 KB)
- OTA Staging: 0x08040000 - 0x0807FFFF (256 KB)
- RAM: 0x20000000 - 0x2001FFFF (128 KB)

## Agent Rules For This Repo
- Keep edits minimal and deterministic.
- Prioritize build/debug correctness before adding firmware features.
- Prefer explicit source lists in CMake over broad globs for startup/system files.
- Do not reintroduce STM32F4 symbols, includes, startup files, or linker assumptions.
- Keep bootloader/app flash boundaries consistent with linker scripts and bootloader constants.

## Context-Efficient Workflow
When starting work, load only:
1. firmware/CMakeLists.txt
2. firmware/core/CMakeLists.txt
3. firmware/bootloader/CMakeLists.txt
4. firmware/core/core.ld
5. firmware/bootloader/bootloader.ld
6. hvsd-3ch-software.code-workspace
7. bootloader/Src/main.cpp (only if bootloader behavior is touched)

This avoids re-reading low-signal vendor directories.

## Known TODO Areas
- STM32G4 flash erase/program implementation in bootloader/Src/main.cpp
- CRC validation logic in bootloader
- Optional future combined image pipeline for field updates (if needed)
- Expand core application beyond minimal main loop

## Definition Of Good Baseline
- Configure succeeds with Debug preset
- Bootloader and app both compile and link
- Debug launch can load both ELFs
- ST-Link session reaches main reliably
- No stale F4 references remain in build-critical paths

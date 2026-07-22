
# https://github.com/MaJerle/stm32-cube-cmake-vscode?tab=readme-ov-file

cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
message("Build type: "              ${CMAKE_BUILD_TYPE})


project(drive_firmware)
enable_language(C CXX ASM)

#
# Core MCU flags, CPU, instruction set and FPU setup
#
set(CPU_PARAMETERS
    -mthumb

    # This depends on the specific STM32 MCU you are using
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Add subdirectories for bootloader and application
add_subdirectory(bootloader)
add_subdirectory(core)

# Define paths to the output binaries.
# Assume bootloader produces target "bootloader" and application produces target "application".
# The following variables refer to the ELF outputs (you can then use objcopy to convert them into binary format).
set(BOOTLOADER_ELF $<TARGET_FILE:bootloader>)
set(APPLICATION_ELF $<TARGET_FILE:application>)
# Create custom commands to convert each ELF to a HEX file.
# Using the standard objcopy tool which is typically part of the gcc arm toolchain.
add_custom_command(
  OUTPUT bootloader.hex
  COMMAND arm-none-eabi-objcopy -O ihex ${BOOTLOADER_ELF} bootloader.hex
  DEPENDS bootloader
  COMMENT "Converting bootloader ELF to HEX"
)

add_custom_command(
  OUTPUT application.hex
  COMMAND arm-none-eabi-objcopy -O ihex ${APPLICATION_ELF} application.hex
  DEPENDS application
  COMMENT "Converting application ELF to HEX"
)

# create the binary for field firmware updates
add_custom_command(
  OUTPUT application.bin
  COMMAND arm-none-eabi-objcopy -O binary ${APPLICATION_ELF} application.bin
  DEPENDS application
  COMMENT "Converting application ELF to BIN"
)

# Merge bootloader and application into combined firmware image
# Use Python from project .venv
set(PYTHON_VENV_BIN ${PROJ_PATH}/../python/.venv/Scripts)
set(PYTHON_EXECUTABLE ${PYTHON_VENV_BIN}/python.exe)
set(MERGE_SCRIPT ${PROJ_PATH}/../python/merge_hex.py)

add_custom_command(
  OUTPUT combined.hex
  COMMAND ${PYTHON_EXECUTABLE} ${MERGE_SCRIPT} bootloader.hex application.hex combined.hex
  DEPENDS bootloader.hex application.hex
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Merging bootloader and application into combined firmware image"
)

add_custom_target(convert_hex ALL DEPENDS bootloader.hex application.hex application.bin combined.hex)
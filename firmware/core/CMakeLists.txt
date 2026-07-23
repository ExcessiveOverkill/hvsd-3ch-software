
# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/core/core.ld)
set(EXECUTABLE                      application)

#
# List of source files to compile
#
set(sources_SRCS
    ${PROJ_PATH}/core/src/main.cpp
    ${PROJ_PATH}/core/src/board_hw.cpp
    ${PROJ_PATH}/core/src/device.cpp
    ${PROJ_PATH}/core/src/device_interrupts.cpp
    ${PROJ_PATH}/core/src/gpio_ll.cpp
    ${PROJ_PATH}/core/src/motor_channel.cpp
    ${PROJ_PATH}/core/src/mode.cpp
    ${PROJ_PATH}/core/src/fan.cpp
    ${PROJ_PATH}/core/src/communication.cpp
    ${PROJ_PATH}/generated/registers/reg_doc.cpp
    ${PROJ_PATH}/generated/registers/reg_shell.cpp
    ${PROJ_PATH}/generated/messaging/msg.cpp
    ${PROJ_PATH}/generated/messaging/msg_strings.cpp
    ${PROJ_PATH}/generated/registers/reg_storage.cpp
    ${PROJ_PATH}/generated/registers/reg_meta.cpp
    ${PROJ_PATH}/generated/registers/reg_verify.cpp
    ${PROJ_PATH}/generated/registers/reg_comm.cpp
    ${PROJ_PATH}/core/startup/startup_stm32g473qetx.s
    ${PROJ_PATH}/drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c
)

#
# Include directories
#
set(include_path_DIRS
    ${PROJ_PATH}/core/inc
    ${PROJ_PATH}/generated
    ${PROJ_PATH}/generated/registers
    ${PROJ_PATH}/generated/messaging
    ${PROJ_PATH}/drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${PROJ_PATH}/drivers/CMSIS/Include
)

#
# Symbols definition
#
set(symbols_SYMB
    "DEBUG"
    "STM32G473xx"
    "HSE_VALUE=16000000"
)

# Executable files
add_executable(${EXECUTABLE} ${sources_SRCS}
    src/leds.cpp
    inc/time.h
    src/time.cpp
    src/control_utils.cpp
    src/cordic.cpp)

# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${include_path_DIRS})

# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${symbols_SYMB})

set_source_files_properties(
    ${PROJ_PATH}/generated/messaging/msg.cpp
    ${PROJ_PATH}/generated/messaging/msg_strings.cpp
    PROPERTIES COMPILE_OPTIONS "-include;${PROJ_PATH}/core/inc/msg_time_override.h"
)

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    # Full debug configuration
    -O0 -g3 -ggdb
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    -T${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,--print-memory-usage
)
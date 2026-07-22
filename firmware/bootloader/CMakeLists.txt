# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/bootloader/bootloader.ld)
set(EXECUTABLE                      bootloader)

#
# List of source files to compile
#
set(sources_SRCS
    ${PROJ_PATH}/bootloader/Startup/reset_handler_stm32g473qetx.s
    ${PROJ_PATH}/bootloader/Src/main.cpp
    ${PROJ_PATH}/bootloader/Src/syscalls.c
    ${PROJ_PATH}/bootloader/Src/sysmem.c
    ${PROJ_PATH}/core/startup/startup_stm32g473qetx.s
    ${PROJ_PATH}/drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c
)

#
# Include directories
#
set(include_path_DIRS
    ${PROJ_PATH}/bootloader/Inc
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
add_executable(${EXECUTABLE} ${sources_SRCS})

# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${include_path_DIRS})

# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${symbols_SYMB})

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    # Full debug configuration
    -Og -g3 -ggdb
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
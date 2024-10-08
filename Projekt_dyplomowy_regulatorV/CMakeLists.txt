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

#
# Core project settings
#
project(Linear_motor)
enable_language(C CXX ASM)

#
# Core MCU flags, CPU, instruction set and FPU setup
# Needs to be set properly for your MCU
#
set(CPU_PARAMETERS
    -mthumb

    # This needs attention to properly set for used MCU
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/STM32L476RGTX_FLASH.ld) # Modified to own .ld file (linker)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

#
# List of source files to compile
#
set(sources_SRCS
    # Put here your source files, one in each line, relative to CMakeLists.txt file location

    #HAL Library
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c

    #Cube Generated
    ${PROJ_PATH}/Core/Src/stm32l4xx_hal_msp.c
    ${PROJ_PATH}/Core/Src/stm32l4xx_it.c
    ${PROJ_PATH}/Core/Src/system_stm32l4xx.c
    ${PROJ_PATH}/Core/Src/syscalls.c
    ${PROJ_PATH}/Core/Src/sysmem.c
    ${PROJ_PATH}/Core/Startup/startup_stm32l476rgtx.s
    ${PROJ_PATH}/Core/Src/main.c
    ${PROJ_PATH}/Core/Src/adc.c
    ${PROJ_PATH}/Core/Src/dma.c
    ${PROJ_PATH}/Core/Src/gpio.c
    ${PROJ_PATH}/Core/Src/i2c.c
    ${PROJ_PATH}/Core/Src/tim.c
    ${PROJ_PATH}/Core/Src/usart.c

    #User Code
    ${PROJ_PATH}/CircularBuffer/CircularBuffer.c
    ${PROJ_PATH}/DigitalFilters/DigitalFilters.c
    ${PROJ_PATH}/ds18b20/ds18b20.c
    ${PROJ_PATH}/HBridge/hbridge.c
    ${PROJ_PATH}/PID/PID.c
    ${PROJ_PATH}/Core/Src/wire.c
)

#
# Include directories
#
set(include_path_DIRS
    # Put here your include dirs, one in each line, relative to CMakeLists.txt file location
    ${PROJ_PATH}/Core/Inc
    ${PROJ_PATH}/Drivers/STM32L4xx_HAL_Driver/Inc
    ${PROJ_PATH}/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${PROJ_PATH}/Drivers/CMSIS/Include

    #User Code
    ${PROJ_PATH}/CircularBuffer
    ${PROJ_PATH}/DigitalFilters
    ${PROJ_PATH}/ds18b20
    ${PROJ_PATH}/HBridge
    ${PROJ_PATH}/PID
)

#
# Symbols definition
#
set(symbols_SYMB
    # Put here your symbols (preprocessor defines), one in each line
    # Encapsulate them with double quotes for safety purpose
    "DEBUG"
    "STM32L476xx"
    "USE_HAL_DRIVER"
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
    -u _printf_float                # STDIO float formatting support
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)
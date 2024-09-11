################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CircularBuffer.c \
../Core/Src/DigitalFilters.c \
../Core/Src/MAFilter.c \
../Core/Src/PID.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/ds18b20.c \
../Core/Src/gpio.c \
../Core/Src/hbridge.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/ring_buffer.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/wire.c 

OBJS += \
./Core/Src/CircularBuffer.o \
./Core/Src/DigitalFilters.o \
./Core/Src/MAFilter.o \
./Core/Src/PID.o \
./Core/Src/adc.o \
./Core/Src/dma.o \
./Core/Src/ds18b20.o \
./Core/Src/gpio.o \
./Core/Src/hbridge.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/ring_buffer.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/wire.o 

C_DEPS += \
./Core/Src/CircularBuffer.d \
./Core/Src/DigitalFilters.d \
./Core/Src/MAFilter.d \
./Core/Src/PID.d \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/ds18b20.d \
./Core/Src/gpio.d \
./Core/Src/hbridge.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/ring_buffer.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/wire.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CircularBuffer.cyclo ./Core/Src/CircularBuffer.d ./Core/Src/CircularBuffer.o ./Core/Src/CircularBuffer.su ./Core/Src/DigitalFilters.cyclo ./Core/Src/DigitalFilters.d ./Core/Src/DigitalFilters.o ./Core/Src/DigitalFilters.su ./Core/Src/MAFilter.cyclo ./Core/Src/MAFilter.d ./Core/Src/MAFilter.o ./Core/Src/MAFilter.su ./Core/Src/PID.cyclo ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/PID.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/ds18b20.cyclo ./Core/Src/ds18b20.d ./Core/Src/ds18b20.o ./Core/Src/ds18b20.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hbridge.cyclo ./Core/Src/hbridge.d ./Core/Src/hbridge.o ./Core/Src/hbridge.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ring_buffer.cyclo ./Core/Src/ring_buffer.d ./Core/Src/ring_buffer.o ./Core/Src/ring_buffer.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/wire.cyclo ./Core/Src/wire.d ./Core/Src/wire.o ./Core/Src/wire.su

.PHONY: clean-Core-2f-Src


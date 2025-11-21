################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bmi2.c \
../Core/Src/bmi270.c \
../Core/Src/bmi270_context.c \
../Core/Src/bmi270_dsd.c \
../Core/Src/bmi270_legacy.c \
../Core/Src/bmi270_maximum_fifo.c \
../Core/Src/bmi2_ois.c \
../Core/Src/gps.c \
../Core/Src/helpers.c \
../Core/Src/main.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/bmi2.o \
./Core/Src/bmi270.o \
./Core/Src/bmi270_context.o \
./Core/Src/bmi270_dsd.o \
./Core/Src/bmi270_legacy.o \
./Core/Src/bmi270_maximum_fifo.o \
./Core/Src/bmi2_ois.o \
./Core/Src/gps.o \
./Core/Src/helpers.o \
./Core/Src/main.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/bmi2.d \
./Core/Src/bmi270.d \
./Core/Src/bmi270_context.d \
./Core/Src/bmi270_dsd.d \
./Core/Src/bmi270_legacy.d \
./Core/Src/bmi270_maximum_fifo.d \
./Core/Src/bmi2_ois.d \
./Core/Src/gps.d \
./Core/Src/helpers.d \
./Core/Src/main.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bmi2.cyclo ./Core/Src/bmi2.d ./Core/Src/bmi2.o ./Core/Src/bmi2.su ./Core/Src/bmi270.cyclo ./Core/Src/bmi270.d ./Core/Src/bmi270.o ./Core/Src/bmi270.su ./Core/Src/bmi270_context.cyclo ./Core/Src/bmi270_context.d ./Core/Src/bmi270_context.o ./Core/Src/bmi270_context.su ./Core/Src/bmi270_dsd.cyclo ./Core/Src/bmi270_dsd.d ./Core/Src/bmi270_dsd.o ./Core/Src/bmi270_dsd.su ./Core/Src/bmi270_legacy.cyclo ./Core/Src/bmi270_legacy.d ./Core/Src/bmi270_legacy.o ./Core/Src/bmi270_legacy.su ./Core/Src/bmi270_maximum_fifo.cyclo ./Core/Src/bmi270_maximum_fifo.d ./Core/Src/bmi270_maximum_fifo.o ./Core/Src/bmi270_maximum_fifo.su ./Core/Src/bmi2_ois.cyclo ./Core/Src/bmi2_ois.d ./Core/Src/bmi2_ois.o ./Core/Src/bmi2_ois.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/helpers.cyclo ./Core/Src/helpers.d ./Core/Src/helpers.o ./Core/Src/helpers.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src


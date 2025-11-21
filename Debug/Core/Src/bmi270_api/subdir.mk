################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bmi270_api/bmi2.c \
../Core/Src/bmi270_api/bmi270.c \
../Core/Src/bmi270_api/bmi270_context.c \
../Core/Src/bmi270_api/bmi270_dsd.c \
../Core/Src/bmi270_api/bmi270_legacy.c \
../Core/Src/bmi270_api/bmi270_maximum_fifo.c \
../Core/Src/bmi270_api/bmi2_ois.c 

OBJS += \
./Core/Src/bmi270_api/bmi2.o \
./Core/Src/bmi270_api/bmi270.o \
./Core/Src/bmi270_api/bmi270_context.o \
./Core/Src/bmi270_api/bmi270_dsd.o \
./Core/Src/bmi270_api/bmi270_legacy.o \
./Core/Src/bmi270_api/bmi270_maximum_fifo.o \
./Core/Src/bmi270_api/bmi2_ois.o 

C_DEPS += \
./Core/Src/bmi270_api/bmi2.d \
./Core/Src/bmi270_api/bmi270.d \
./Core/Src/bmi270_api/bmi270_context.d \
./Core/Src/bmi270_api/bmi270_dsd.d \
./Core/Src/bmi270_api/bmi270_legacy.d \
./Core/Src/bmi270_api/bmi270_maximum_fifo.d \
./Core/Src/bmi270_api/bmi2_ois.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/bmi270_api/%.o Core/Src/bmi270_api/%.su Core/Src/bmi270_api/%.cyclo: ../Core/Src/bmi270_api/%.c Core/Src/bmi270_api/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Core/Inc/bmi270_api -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-bmi270_api

clean-Core-2f-Src-2f-bmi270_api:
	-$(RM) ./Core/Src/bmi270_api/bmi2.cyclo ./Core/Src/bmi270_api/bmi2.d ./Core/Src/bmi270_api/bmi2.o ./Core/Src/bmi270_api/bmi2.su ./Core/Src/bmi270_api/bmi270.cyclo ./Core/Src/bmi270_api/bmi270.d ./Core/Src/bmi270_api/bmi270.o ./Core/Src/bmi270_api/bmi270.su ./Core/Src/bmi270_api/bmi270_context.cyclo ./Core/Src/bmi270_api/bmi270_context.d ./Core/Src/bmi270_api/bmi270_context.o ./Core/Src/bmi270_api/bmi270_context.su ./Core/Src/bmi270_api/bmi270_dsd.cyclo ./Core/Src/bmi270_api/bmi270_dsd.d ./Core/Src/bmi270_api/bmi270_dsd.o ./Core/Src/bmi270_api/bmi270_dsd.su ./Core/Src/bmi270_api/bmi270_legacy.cyclo ./Core/Src/bmi270_api/bmi270_legacy.d ./Core/Src/bmi270_api/bmi270_legacy.o ./Core/Src/bmi270_api/bmi270_legacy.su ./Core/Src/bmi270_api/bmi270_maximum_fifo.cyclo ./Core/Src/bmi270_api/bmi270_maximum_fifo.d ./Core/Src/bmi270_api/bmi270_maximum_fifo.o ./Core/Src/bmi270_api/bmi270_maximum_fifo.su ./Core/Src/bmi270_api/bmi2_ois.cyclo ./Core/Src/bmi270_api/bmi2_ois.d ./Core/Src/bmi270_api/bmi2_ois.o ./Core/Src/bmi270_api/bmi2_ois.su

.PHONY: clean-Core-2f-Src-2f-bmi270_api


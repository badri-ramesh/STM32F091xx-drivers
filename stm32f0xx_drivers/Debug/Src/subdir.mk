################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001Led_toggle.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/001Led_toggle.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/001Led_toggle.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/001Led_toggle.o: ../Src/001Led_toggle.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DNUCLEO_F091RC -DSTM32F091RCTx -c -I"C:/Users/Badri Ramesh/Downloads/Learning/MCU1/stm32f0xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/001Led_toggle.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DNUCLEO_F091RC -DSTM32F091RCTx -c -I"C:/Users/Badri Ramesh/Downloads/Learning/MCU1/stm32f0xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DNUCLEO_F091RC -DSTM32F091RCTx -c -I"C:/Users/Badri Ramesh/Downloads/Learning/MCU1/stm32f0xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


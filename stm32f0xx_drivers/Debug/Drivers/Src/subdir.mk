################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f091xx_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32f091xx_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32f091xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f091xx_gpio_driver.o: ../Drivers/Src/stm32f091xx_gpio_driver.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DNUCLEO_F091RC -DSTM32F091RCTx -c -I"C:/Users/Badri Ramesh/Downloads/Learning/MCU1/stm32f0xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f091xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


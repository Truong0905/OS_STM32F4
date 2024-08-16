################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/Src/stm32f446xx_gpio_driver.c \
../Driver/Src/stm32f446xx_i2c_driver.c \
../Driver/Src/stm32f446xx_spi\ _driver.c 

OBJS += \
./Driver/Src/stm32f446xx_gpio_driver.o \
./Driver/Src/stm32f446xx_i2c_driver.o \
./Driver/Src/stm32f446xx_spi\ _driver.o 

C_DEPS += \
./Driver/Src/stm32f446xx_gpio_driver.d \
./Driver/Src/stm32f446xx_i2c_driver.d \
./Driver/Src/stm32f446xx_spi\ _driver.d 


# Each subdirectory must supply rules for building sources it contributes
Driver/Src/%.o Driver/Src/%.su: ../Driver/Src/%.c Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"C:/Code/stm32f4xx_drivers/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Driver/Src/stm32f446xx_spi\ _driver.o: ../Driver/Src/stm32f446xx_spi\ _driver.c Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"C:/Code/stm32f4xx_drivers/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Driver/Src/stm32f446xx_spi _driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Driver-2f-Src

clean-Driver-2f-Src:
	-$(RM) ./Driver/Src/stm32f446xx_gpio_driver.d ./Driver/Src/stm32f446xx_gpio_driver.o ./Driver/Src/stm32f446xx_gpio_driver.su ./Driver/Src/stm32f446xx_i2c_driver.d ./Driver/Src/stm32f446xx_i2c_driver.o ./Driver/Src/stm32f446xx_i2c_driver.su ./Driver/Src/stm32f446xx_spi\ _driver.d ./Driver/Src/stm32f446xx_spi\ _driver.o ./Driver/Src/stm32f446xx_spi\ _driver.su

.PHONY: clean-Driver-2f-Src


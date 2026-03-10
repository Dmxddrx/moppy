################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Middlewares/SSD1306/fonts.c \
../Drivers/Middlewares/SSD1306/ssd1306.c 

OBJS += \
./Drivers/Middlewares/SSD1306/fonts.o \
./Drivers/Middlewares/SSD1306/ssd1306.o 

C_DEPS += \
./Drivers/Middlewares/SSD1306/fonts.d \
./Drivers/Middlewares/SSD1306/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Middlewares/SSD1306/%.o Drivers/Middlewares/SSD1306/%.su Drivers/Middlewares/SSD1306/%.cyclo: ../Drivers/Middlewares/SSD1306/%.c Drivers/Middlewares/SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/DMXHP/Documents/Programing_Files_IoT/STM32CubeIDE/workspace_2.0.0/STM32F407VET6/Moppy/Drivers/Middlewares/SSD1306" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Middlewares-2f-SSD1306

clean-Drivers-2f-Middlewares-2f-SSD1306:
	-$(RM) ./Drivers/Middlewares/SSD1306/fonts.cyclo ./Drivers/Middlewares/SSD1306/fonts.d ./Drivers/Middlewares/SSD1306/fonts.o ./Drivers/Middlewares/SSD1306/fonts.su ./Drivers/Middlewares/SSD1306/ssd1306.cyclo ./Drivers/Middlewares/SSD1306/ssd1306.d ./Drivers/Middlewares/SSD1306/ssd1306.o ./Drivers/Middlewares/SSD1306/ssd1306.su

.PHONY: clean-Drivers-2f-Middlewares-2f-SSD1306


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Middlewares/VL53L0X/vl53l0x.c 

OBJS += \
./Drivers/Middlewares/VL53L0X/vl53l0x.o 

C_DEPS += \
./Drivers/Middlewares/VL53L0X/vl53l0x.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Middlewares/VL53L0X/%.o Drivers/Middlewares/VL53L0X/%.su Drivers/Middlewares/VL53L0X/%.cyclo: ../Drivers/Middlewares/VL53L0X/%.c Drivers/Middlewares/VL53L0X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/DMXHP/Documents/Programing_Files_IoT/STM32CubeIDE/workspace_2.0.0/STM32F407VET6/Moppy/Drivers/Middlewares/VL53L0X" -I"C:/Users/DMXHP/Documents/Programing_Files_IoT/STM32CubeIDE/workspace_2.0.0/STM32F407VET6/Moppy/Drivers/Middlewares/SSD1306" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Middlewares-2f-VL53L0X

clean-Drivers-2f-Middlewares-2f-VL53L0X:
	-$(RM) ./Drivers/Middlewares/VL53L0X/vl53l0x.cyclo ./Drivers/Middlewares/VL53L0X/vl53l0x.d ./Drivers/Middlewares/VL53L0X/vl53l0x.o ./Drivers/Middlewares/VL53L0X/vl53l0x.su

.PHONY: clean-Drivers-2f-Middlewares-2f-VL53L0X


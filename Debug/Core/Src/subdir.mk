################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/general.c \
../Core/Src/hmc5883l.c \
../Core/Src/main.c \
../Core/Src/motion.c \
../Core/Src/mpu6500.c \
../Core/Src/odometry.c \
../Core/Src/oled.c \
../Core/Src/pid.c \
../Core/Src/stable.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/general.o \
./Core/Src/hmc5883l.o \
./Core/Src/main.o \
./Core/Src/motion.o \
./Core/Src/mpu6500.o \
./Core/Src/odometry.o \
./Core/Src/oled.o \
./Core/Src/pid.o \
./Core/Src/stable.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/general.d \
./Core/Src/hmc5883l.d \
./Core/Src/main.d \
./Core/Src/motion.d \
./Core/Src/mpu6500.d \
./Core/Src/odometry.d \
./Core/Src/oled.d \
./Core/Src/pid.d \
./Core/Src/stable.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/DMXHP/Documents/Programing_Files_IoT/STM32CubeIDE/workspace_2.0.0/STM32F407VET6/Moppy/Drivers/Middlewares/SSD1306" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/general.cyclo ./Core/Src/general.d ./Core/Src/general.o ./Core/Src/general.su ./Core/Src/hmc5883l.cyclo ./Core/Src/hmc5883l.d ./Core/Src/hmc5883l.o ./Core/Src/hmc5883l.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion.cyclo ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/mpu6500.cyclo ./Core/Src/mpu6500.d ./Core/Src/mpu6500.o ./Core/Src/mpu6500.su ./Core/Src/odometry.cyclo ./Core/Src/odometry.d ./Core/Src/odometry.o ./Core/Src/odometry.su ./Core/Src/oled.cyclo ./Core/Src/oled.d ./Core/Src/oled.o ./Core/Src/oled.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/stable.cyclo ./Core/Src/stable.d ./Core/Src/stable.o ./Core/Src/stable.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src


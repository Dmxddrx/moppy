################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/coverage.c \
../Core/Src/encoder.c \
../Core/Src/general.c \
../Core/Src/hmc5883l.c \
../Core/Src/ir.c \
../Core/Src/main.c \
../Core/Src/mapping.c \
../Core/Src/motion.c \
../Core/Src/motor.c \
../Core/Src/motor_pwm.c \
../Core/Src/mpu6500.c \
../Core/Src/odometry.c \
../Core/Src/oled.c \
../Core/Src/pid.c \
../Core/Src/slam_lite.c \
../Core/Src/stable.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/ultrasonic.c \
../Core/Src/wall_follow.c 

OBJS += \
./Core/Src/coverage.o \
./Core/Src/encoder.o \
./Core/Src/general.o \
./Core/Src/hmc5883l.o \
./Core/Src/ir.o \
./Core/Src/main.o \
./Core/Src/mapping.o \
./Core/Src/motion.o \
./Core/Src/motor.o \
./Core/Src/motor_pwm.o \
./Core/Src/mpu6500.o \
./Core/Src/odometry.o \
./Core/Src/oled.o \
./Core/Src/pid.o \
./Core/Src/slam_lite.o \
./Core/Src/stable.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/ultrasonic.o \
./Core/Src/wall_follow.o 

C_DEPS += \
./Core/Src/coverage.d \
./Core/Src/encoder.d \
./Core/Src/general.d \
./Core/Src/hmc5883l.d \
./Core/Src/ir.d \
./Core/Src/main.d \
./Core/Src/mapping.d \
./Core/Src/motion.d \
./Core/Src/motor.d \
./Core/Src/motor_pwm.d \
./Core/Src/mpu6500.d \
./Core/Src/odometry.d \
./Core/Src/oled.d \
./Core/Src/pid.d \
./Core/Src/slam_lite.d \
./Core/Src/stable.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/ultrasonic.d \
./Core/Src/wall_follow.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/DMXHP/Documents/Programing_Files_IoT/STM32CubeIDE/workspace_2.0.0/STM32F407VET6/Moppy/Drivers/Middlewares/SSD1306" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/coverage.cyclo ./Core/Src/coverage.d ./Core/Src/coverage.o ./Core/Src/coverage.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/general.cyclo ./Core/Src/general.d ./Core/Src/general.o ./Core/Src/general.su ./Core/Src/hmc5883l.cyclo ./Core/Src/hmc5883l.d ./Core/Src/hmc5883l.o ./Core/Src/hmc5883l.su ./Core/Src/ir.cyclo ./Core/Src/ir.d ./Core/Src/ir.o ./Core/Src/ir.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mapping.cyclo ./Core/Src/mapping.d ./Core/Src/mapping.o ./Core/Src/mapping.su ./Core/Src/motion.cyclo ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/motor_pwm.cyclo ./Core/Src/motor_pwm.d ./Core/Src/motor_pwm.o ./Core/Src/motor_pwm.su ./Core/Src/mpu6500.cyclo ./Core/Src/mpu6500.d ./Core/Src/mpu6500.o ./Core/Src/mpu6500.su ./Core/Src/odometry.cyclo ./Core/Src/odometry.d ./Core/Src/odometry.o ./Core/Src/odometry.su ./Core/Src/oled.cyclo ./Core/Src/oled.d ./Core/Src/oled.o ./Core/Src/oled.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/slam_lite.cyclo ./Core/Src/slam_lite.d ./Core/Src/slam_lite.o ./Core/Src/slam_lite.su ./Core/Src/stable.cyclo ./Core/Src/stable.d ./Core/Src/stable.o ./Core/Src/stable.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/ultrasonic.cyclo ./Core/Src/ultrasonic.d ./Core/Src/ultrasonic.o ./Core/Src/ultrasonic.su ./Core/Src/wall_follow.cyclo ./Core/Src/wall_follow.d ./Core/Src/wall_follow.o ./Core/Src/wall_follow.su

.PHONY: clean-Core-2f-Src


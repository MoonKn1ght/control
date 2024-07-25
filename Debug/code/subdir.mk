################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../code/CCD.cpp \
../code/Chassis.cpp \
../code/Controller.cpp \
../code/Encoder.cpp \
../code/IMU.cpp \
../code/N20_Motor.cpp \
../code/PID.cpp \
../code/Remote.cpp \
../code/Tracking.cpp \
../code/task.cpp 

C_SRCS += \
../code/hal.c \
../code/imu660.c 

C_DEPS += \
./code/hal.d \
./code/imu660.d 

OBJS += \
./code/CCD.o \
./code/Chassis.o \
./code/Controller.o \
./code/Encoder.o \
./code/IMU.o \
./code/N20_Motor.o \
./code/PID.o \
./code/Remote.o \
./code/Tracking.o \
./code/hal.o \
./code/imu660.o \
./code/task.o 

CPP_DEPS += \
./code/CCD.d \
./code/Chassis.d \
./code/Controller.d \
./code/Encoder.d \
./code/IMU.d \
./code/N20_Motor.d \
./code/PID.d \
./code/Remote.d \
./code/Tracking.d \
./code/task.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.o code/%.su code/%.cyclo: ../code/%.cpp code/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../code -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
code/%.o code/%.su code/%.cyclo: ../code/%.c code/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../code -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-code

clean-code:
	-$(RM) ./code/CCD.cyclo ./code/CCD.d ./code/CCD.o ./code/CCD.su ./code/Chassis.cyclo ./code/Chassis.d ./code/Chassis.o ./code/Chassis.su ./code/Controller.cyclo ./code/Controller.d ./code/Controller.o ./code/Controller.su ./code/Encoder.cyclo ./code/Encoder.d ./code/Encoder.o ./code/Encoder.su ./code/IMU.cyclo ./code/IMU.d ./code/IMU.o ./code/IMU.su ./code/N20_Motor.cyclo ./code/N20_Motor.d ./code/N20_Motor.o ./code/N20_Motor.su ./code/PID.cyclo ./code/PID.d ./code/PID.o ./code/PID.su ./code/Remote.cyclo ./code/Remote.d ./code/Remote.o ./code/Remote.su ./code/Tracking.cyclo ./code/Tracking.d ./code/Tracking.o ./code/Tracking.su ./code/hal.cyclo ./code/hal.d ./code/hal.o ./code/hal.su ./code/imu660.cyclo ./code/imu660.d ./code/imu660.o ./code/imu660.su ./code/task.cyclo ./code/task.d ./code/task.o ./code/task.su

.PHONY: clean-code


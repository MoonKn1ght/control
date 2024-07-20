################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../code/CCD.cpp \
../code/Chassis.cpp \
../code/Encoder.cpp \
../code/N20_Motor.cpp \
../code/PID.cpp \
../code/Remote.cpp \
../code/task.cpp 

OBJS += \
./code/CCD.o \
./code/Chassis.o \
./code/Encoder.o \
./code/N20_Motor.o \
./code/PID.o \
./code/Remote.o \
./code/task.o 

CPP_DEPS += \
./code/CCD.d \
./code/Chassis.d \
./code/Encoder.d \
./code/N20_Motor.d \
./code/PID.d \
./code/Remote.d \
./code/task.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.o code/%.su code/%.cyclo: ../code/%.cpp code/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../code -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-code

clean-code:
	-$(RM) ./code/CCD.cyclo ./code/CCD.d ./code/CCD.o ./code/CCD.su ./code/Chassis.cyclo ./code/Chassis.d ./code/Chassis.o ./code/Chassis.su ./code/Encoder.cyclo ./code/Encoder.d ./code/Encoder.o ./code/Encoder.su ./code/N20_Motor.cyclo ./code/N20_Motor.d ./code/N20_Motor.o ./code/N20_Motor.su ./code/PID.cyclo ./code/PID.d ./code/PID.o ./code/PID.su ./code/Remote.cyclo ./code/Remote.d ./code/Remote.o ./code/Remote.su ./code/task.cyclo ./code/task.d ./code/task.o ./code/task.su

.PHONY: clean-code


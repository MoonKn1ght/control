################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../code/Encoder.cpp \
../code/LINEAR_CCD.cpp \
../code/N20_Motor.cpp \
../code/PID.cpp \
../code/task.cpp 

OBJS += \
./code/Encoder.o \
./code/LINEAR_CCD.o \
./code/N20_Motor.o \
./code/PID.o \
./code/task.o 

CPP_DEPS += \
./code/Encoder.d \
./code/LINEAR_CCD.d \
./code/N20_Motor.d \
./code/PID.d \
./code/task.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.o code/%.su code/%.cyclo: ../code/%.cpp code/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../code -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-code

clean-code:
	-$(RM) ./code/Encoder.cyclo ./code/Encoder.d ./code/Encoder.o ./code/Encoder.su ./code/LINEAR_CCD.cyclo ./code/LINEAR_CCD.d ./code/LINEAR_CCD.o ./code/LINEAR_CCD.su ./code/N20_Motor.cyclo ./code/N20_Motor.d ./code/N20_Motor.o ./code/N20_Motor.su ./code/PID.cyclo ./code/PID.d ./code/PID.o ./code/PID.su ./code/task.cyclo ./code/task.d ./code/task.o ./code/task.su

.PHONY: clean-code


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Encoder.c \
../Core/Src/MotorDriver.c \
../Core/Src/TrajectoryGenerator.c \
../Core/Src/VelocityFormPID.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/feedforward.c \
../Core/Src/gpio.c \
../Core/Src/kalman_filter.c \
../Core/Src/kalman_prs.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/position_control.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/velocity_control.c \
../Core/Src/waypoints.c 

OBJS += \
./Core/Src/Encoder.o \
./Core/Src/MotorDriver.o \
./Core/Src/TrajectoryGenerator.o \
./Core/Src/VelocityFormPID.o \
./Core/Src/adc.o \
./Core/Src/dma.o \
./Core/Src/feedforward.o \
./Core/Src/gpio.o \
./Core/Src/kalman_filter.o \
./Core/Src/kalman_prs.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/position_control.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/velocity_control.o \
./Core/Src/waypoints.o 

C_DEPS += \
./Core/Src/Encoder.d \
./Core/Src/MotorDriver.d \
./Core/Src/TrajectoryGenerator.d \
./Core/Src/VelocityFormPID.d \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/feedforward.d \
./Core/Src/gpio.d \
./Core/Src/kalman_filter.d \
./Core/Src/kalman_prs.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/position_control.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/velocity_control.d \
./Core/Src/waypoints.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/BasicMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/BayesFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/CommonTables" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/ComplexMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/ControllerFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/DistanceFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/FastMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/FilteringFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/InterpolationFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/MatrixFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/QuaternionMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/StatisticsFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/SupportFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/SVMFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/TransformFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/WindowFunctions" -I../Middlewares/Third_Party/ARM_CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Encoder.cyclo ./Core/Src/Encoder.d ./Core/Src/Encoder.o ./Core/Src/Encoder.su ./Core/Src/MotorDriver.cyclo ./Core/Src/MotorDriver.d ./Core/Src/MotorDriver.o ./Core/Src/MotorDriver.su ./Core/Src/TrajectoryGenerator.cyclo ./Core/Src/TrajectoryGenerator.d ./Core/Src/TrajectoryGenerator.o ./Core/Src/TrajectoryGenerator.su ./Core/Src/VelocityFormPID.cyclo ./Core/Src/VelocityFormPID.d ./Core/Src/VelocityFormPID.o ./Core/Src/VelocityFormPID.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/feedforward.cyclo ./Core/Src/feedforward.d ./Core/Src/feedforward.o ./Core/Src/feedforward.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/kalman_filter.cyclo ./Core/Src/kalman_filter.d ./Core/Src/kalman_filter.o ./Core/Src/kalman_filter.su ./Core/Src/kalman_prs.cyclo ./Core/Src/kalman_prs.d ./Core/Src/kalman_prs.o ./Core/Src/kalman_prs.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/position_control.cyclo ./Core/Src/position_control.d ./Core/Src/position_control.o ./Core/Src/position_control.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/velocity_control.cyclo ./Core/Src/velocity_control.d ./Core/Src/velocity_control.o ./Core/Src/velocity_control.su ./Core/Src/waypoints.cyclo ./Core/Src/waypoints.d ./Core/Src/waypoints.o ./Core/Src/waypoints.su

.PHONY: clean-Core-2f-Src


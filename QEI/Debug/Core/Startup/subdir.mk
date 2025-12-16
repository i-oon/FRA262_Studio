################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/BasicMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/BayesFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/CommonTables" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/ComplexMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/ControllerFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/DistanceFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/FastMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/FilteringFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/InterpolationFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/MatrixFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/QuaternionMathFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/StatisticsFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/SupportFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/SVMFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/TransformFunctions" -I"C:/Users/ioonz/STM32CubeIDE/workspace_1.17.0/QEI/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup


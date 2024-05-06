################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
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
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/BasicMathFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/BayesFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/CommonTables" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/ComplexMathFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/ControllerFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/DistanceFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/FastMathFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/FilteringFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/InterpolationFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/MatrixFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/QuaternionMathFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/StatisticsFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/SupportFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/SVMFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/TransformFunctions" -I"/Users/buzz/STM32CubeIDE/workspace_1.13.2/Studio_readRPM/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup


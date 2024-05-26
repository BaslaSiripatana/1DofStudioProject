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
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/BasicMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/BayesFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/CommonTables" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/ComplexMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/ControllerFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/DistanceFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/FastMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/FilteringFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/InterpolationFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/MatrixFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/QuaternionMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/StatisticsFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/SupportFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/SVMFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/TransformFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup


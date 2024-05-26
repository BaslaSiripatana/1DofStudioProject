################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/BasicMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/BayesFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/CommonTables" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/ComplexMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/ControllerFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/DistanceFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/FastMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/FilteringFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/InterpolationFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/MatrixFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/QuaternionMathFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/StatisticsFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/SupportFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/SVMFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/TransformFunctions" -I"C:/Users/bcc35/Downloads/Base1_DOF/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-ComplexMathFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-ComplexMathFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/ComplexMathFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-ComplexMathFunctions


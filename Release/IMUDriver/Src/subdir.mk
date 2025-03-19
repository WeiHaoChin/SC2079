################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../IMUDriver/Src/ICM20948.c 

OBJS += \
./IMUDriver/Src/ICM20948.o 

C_DEPS += \
./IMUDriver/Src/ICM20948.d 


# Each subdirectory must supply rules for building sources it contributes
IMUDriver/Src/%.o IMUDriver/Src/%.su IMUDriver/Src/%.cyclo: ../IMUDriver/Src/%.c IMUDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/STM32CubeIDE/workspace_1.17.0/MDP/PeripheralDriver/Inc" -I"C:/Users/user/STM32CubeIDE/workspace_1.17.0/MDP/PeripheralDriver" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/user/STM32CubeIDE/workspace_1.17.0/MDP/IMUDriver/Inc" -I"C:/Users/user/STM32CubeIDE/workspace_1.17.0/MDP/IMUDriver" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-IMUDriver-2f-Src

clean-IMUDriver-2f-Src:
	-$(RM) ./IMUDriver/Src/ICM20948.cyclo ./IMUDriver/Src/ICM20948.d ./IMUDriver/Src/ICM20948.o ./IMUDriver/Src/ICM20948.su

.PHONY: clean-IMUDriver-2f-Src


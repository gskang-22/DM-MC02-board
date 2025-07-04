################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Algo/Src/MahonyAHRS.c 

OBJS += \
./Algo/Src/MahonyAHRS.o 

C_DEPS += \
./Algo/Src/MahonyAHRS.d 


# Each subdirectory must supply rules for building sources it contributes
Algo/Src/%.o Algo/Src/%.su Algo/Src/%.cyclo: ../Algo/Src/%.c Algo/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/User/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/Algo/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/BSP/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/BSP" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/Algo" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/User" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Algo-2f-Src

clean-Algo-2f-Src:
	-$(RM) ./Algo/Src/MahonyAHRS.cyclo ./Algo/Src/MahonyAHRS.d ./Algo/Src/MahonyAHRS.o ./Algo/Src/MahonyAHRS.su

.PHONY: clean-Algo-2f-Src


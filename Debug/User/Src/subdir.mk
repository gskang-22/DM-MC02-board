################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/Src/buzzer_task.c \
../User/Src/dji_ndj_remote.c \
../User/Src/gm_6020.c \
../User/Src/imu_task.c \
../User/Src/j60_10motor_task.c \
../User/Src/pc_mcu_uart.c 

OBJS += \
./User/Src/buzzer_task.o \
./User/Src/dji_ndj_remote.o \
./User/Src/gm_6020.o \
./User/Src/imu_task.o \
./User/Src/j60_10motor_task.o \
./User/Src/pc_mcu_uart.o 

C_DEPS += \
./User/Src/buzzer_task.d \
./User/Src/dji_ndj_remote.d \
./User/Src/gm_6020.d \
./User/Src/imu_task.d \
./User/Src/j60_10motor_task.d \
./User/Src/pc_mcu_uart.d 


# Each subdirectory must supply rules for building sources it contributes
User/Src/%.o User/Src/%.su User/Src/%.cyclo: ../User/Src/%.c User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/User/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/Algo/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/BSP/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/BSP" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/Algo" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/User" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User-2f-Src

clean-User-2f-Src:
	-$(RM) ./User/Src/buzzer_task.cyclo ./User/Src/buzzer_task.d ./User/Src/buzzer_task.o ./User/Src/buzzer_task.su ./User/Src/dji_ndj_remote.cyclo ./User/Src/dji_ndj_remote.d ./User/Src/dji_ndj_remote.o ./User/Src/dji_ndj_remote.su ./User/Src/gm_6020.cyclo ./User/Src/gm_6020.d ./User/Src/gm_6020.o ./User/Src/gm_6020.su ./User/Src/imu_task.cyclo ./User/Src/imu_task.d ./User/Src/imu_task.o ./User/Src/imu_task.su ./User/Src/j60_10motor_task.cyclo ./User/Src/j60_10motor_task.d ./User/Src/j60_10motor_task.o ./User/Src/j60_10motor_task.su ./User/Src/pc_mcu_uart.cyclo ./User/Src/pc_mcu_uart.d ./User/Src/pc_mcu_uart.o ./User/Src/pc_mcu_uart.su

.PHONY: clean-User-2f-Src


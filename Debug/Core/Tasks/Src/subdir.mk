################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Tasks/Src/INS_task.c \
../Core/Tasks/Src/buzzing_task.c \
../Core/Tasks/Src/can_msg_processor.c \
../Core/Tasks/Src/can_relay_task.c \
../Core/Tasks/Src/control_input_task.c \
../Core/Tasks/Src/control_keyboard.c \
../Core/Tasks/Src/control_remote.c \
../Core/Tasks/Src/control_sbc.c \
../Core/Tasks/Src/error_handler.c \
../Core/Tasks/Src/gimbal_control_task.c \
../Core/Tasks/Src/hud_new.c \
../Core/Tasks/Src/imu_processing_task.c \
../Core/Tasks/Src/launcher_control_task.c \
../Core/Tasks/Src/mahony_filter.c \
../Core/Tasks/Src/master_task.c \
../Core/Tasks/Src/motor_config.c \
../Core/Tasks/Src/motor_control.c \
../Core/Tasks/Src/motor_control_task.c \
../Core/Tasks/Src/movement_control_task.c \
../Core/Tasks/Src/referee_processing_task.c \
../Core/Tasks/Src/startup_task.c \
../Core/Tasks/Src/supercap_comm_task.c \
../Core/Tasks/Src/telemetry_task.c \
../Core/Tasks/Src/usb_task.c 

OBJS += \
./Core/Tasks/Src/INS_task.o \
./Core/Tasks/Src/buzzing_task.o \
./Core/Tasks/Src/can_msg_processor.o \
./Core/Tasks/Src/can_relay_task.o \
./Core/Tasks/Src/control_input_task.o \
./Core/Tasks/Src/control_keyboard.o \
./Core/Tasks/Src/control_remote.o \
./Core/Tasks/Src/control_sbc.o \
./Core/Tasks/Src/error_handler.o \
./Core/Tasks/Src/gimbal_control_task.o \
./Core/Tasks/Src/hud_new.o \
./Core/Tasks/Src/imu_processing_task.o \
./Core/Tasks/Src/launcher_control_task.o \
./Core/Tasks/Src/mahony_filter.o \
./Core/Tasks/Src/master_task.o \
./Core/Tasks/Src/motor_config.o \
./Core/Tasks/Src/motor_control.o \
./Core/Tasks/Src/motor_control_task.o \
./Core/Tasks/Src/movement_control_task.o \
./Core/Tasks/Src/referee_processing_task.o \
./Core/Tasks/Src/startup_task.o \
./Core/Tasks/Src/supercap_comm_task.o \
./Core/Tasks/Src/telemetry_task.o \
./Core/Tasks/Src/usb_task.o 

C_DEPS += \
./Core/Tasks/Src/INS_task.d \
./Core/Tasks/Src/buzzing_task.d \
./Core/Tasks/Src/can_msg_processor.d \
./Core/Tasks/Src/can_relay_task.d \
./Core/Tasks/Src/control_input_task.d \
./Core/Tasks/Src/control_keyboard.d \
./Core/Tasks/Src/control_remote.d \
./Core/Tasks/Src/control_sbc.d \
./Core/Tasks/Src/error_handler.d \
./Core/Tasks/Src/gimbal_control_task.d \
./Core/Tasks/Src/hud_new.d \
./Core/Tasks/Src/imu_processing_task.d \
./Core/Tasks/Src/launcher_control_task.d \
./Core/Tasks/Src/mahony_filter.d \
./Core/Tasks/Src/master_task.d \
./Core/Tasks/Src/motor_config.d \
./Core/Tasks/Src/motor_control.d \
./Core/Tasks/Src/motor_control_task.d \
./Core/Tasks/Src/movement_control_task.d \
./Core/Tasks/Src/referee_processing_task.d \
./Core/Tasks/Src/startup_task.d \
./Core/Tasks/Src/supercap_comm_task.d \
./Core/Tasks/Src/telemetry_task.d \
./Core/Tasks/Src/usb_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Tasks/Src/%.o Core/Tasks/Src/%.su Core/Tasks/Src/%.cyclo: ../Core/Tasks/Src/%.c Core/Tasks/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/Core/BSP/Inc" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/Core/robot_config" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/Core/Tasks/Inc" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/System/BRoCo/include" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/System/Threads/Inc" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/System/utils/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Tasks-2f-Src

clean-Core-2f-Tasks-2f-Src:
	-$(RM) ./Core/Tasks/Src/INS_task.cyclo ./Core/Tasks/Src/INS_task.d ./Core/Tasks/Src/INS_task.o ./Core/Tasks/Src/INS_task.su ./Core/Tasks/Src/buzzing_task.cyclo ./Core/Tasks/Src/buzzing_task.d ./Core/Tasks/Src/buzzing_task.o ./Core/Tasks/Src/buzzing_task.su ./Core/Tasks/Src/can_msg_processor.cyclo ./Core/Tasks/Src/can_msg_processor.d ./Core/Tasks/Src/can_msg_processor.o ./Core/Tasks/Src/can_msg_processor.su ./Core/Tasks/Src/can_relay_task.cyclo ./Core/Tasks/Src/can_relay_task.d ./Core/Tasks/Src/can_relay_task.o ./Core/Tasks/Src/can_relay_task.su ./Core/Tasks/Src/control_input_task.cyclo ./Core/Tasks/Src/control_input_task.d ./Core/Tasks/Src/control_input_task.o ./Core/Tasks/Src/control_input_task.su ./Core/Tasks/Src/control_keyboard.cyclo ./Core/Tasks/Src/control_keyboard.d ./Core/Tasks/Src/control_keyboard.o ./Core/Tasks/Src/control_keyboard.su ./Core/Tasks/Src/control_remote.cyclo ./Core/Tasks/Src/control_remote.d ./Core/Tasks/Src/control_remote.o ./Core/Tasks/Src/control_remote.su ./Core/Tasks/Src/control_sbc.cyclo ./Core/Tasks/Src/control_sbc.d ./Core/Tasks/Src/control_sbc.o ./Core/Tasks/Src/control_sbc.su ./Core/Tasks/Src/error_handler.cyclo ./Core/Tasks/Src/error_handler.d ./Core/Tasks/Src/error_handler.o ./Core/Tasks/Src/error_handler.su ./Core/Tasks/Src/gimbal_control_task.cyclo ./Core/Tasks/Src/gimbal_control_task.d ./Core/Tasks/Src/gimbal_control_task.o ./Core/Tasks/Src/gimbal_control_task.su ./Core/Tasks/Src/hud_new.cyclo ./Core/Tasks/Src/hud_new.d ./Core/Tasks/Src/hud_new.o ./Core/Tasks/Src/hud_new.su ./Core/Tasks/Src/imu_processing_task.cyclo ./Core/Tasks/Src/imu_processing_task.d ./Core/Tasks/Src/imu_processing_task.o ./Core/Tasks/Src/imu_processing_task.su ./Core/Tasks/Src/launcher_control_task.cyclo ./Core/Tasks/Src/launcher_control_task.d ./Core/Tasks/Src/launcher_control_task.o ./Core/Tasks/Src/launcher_control_task.su ./Core/Tasks/Src/mahony_filter.cyclo ./Core/Tasks/Src/mahony_filter.d ./Core/Tasks/Src/mahony_filter.o ./Core/Tasks/Src/mahony_filter.su ./Core/Tasks/Src/master_task.cyclo ./Core/Tasks/Src/master_task.d ./Core/Tasks/Src/master_task.o ./Core/Tasks/Src/master_task.su ./Core/Tasks/Src/motor_config.cyclo ./Core/Tasks/Src/motor_config.d ./Core/Tasks/Src/motor_config.o ./Core/Tasks/Src/motor_config.su ./Core/Tasks/Src/motor_control.cyclo ./Core/Tasks/Src/motor_control.d ./Core/Tasks/Src/motor_control.o ./Core/Tasks/Src/motor_control.su ./Core/Tasks/Src/motor_control_task.cyclo ./Core/Tasks/Src/motor_control_task.d ./Core/Tasks/Src/motor_control_task.o ./Core/Tasks/Src/motor_control_task.su ./Core/Tasks/Src/movement_control_task.cyclo ./Core/Tasks/Src/movement_control_task.d ./Core/Tasks/Src/movement_control_task.o ./Core/Tasks/Src/movement_control_task.su ./Core/Tasks/Src/referee_processing_task.cyclo ./Core/Tasks/Src/referee_processing_task.d ./Core/Tasks/Src/referee_processing_task.o ./Core/Tasks/Src/referee_processing_task.su ./Core/Tasks/Src/startup_task.cyclo ./Core/Tasks/Src/startup_task.d ./Core/Tasks/Src/startup_task.o ./Core/Tasks/Src/startup_task.su ./Core/Tasks/Src/supercap_comm_task.cyclo ./Core/Tasks/Src/supercap_comm_task.d ./Core/Tasks/Src/supercap_comm_task.o ./Core/Tasks/Src/supercap_comm_task.su ./Core/Tasks/Src/telemetry_task.cyclo ./Core/Tasks/Src/telemetry_task.d ./Core/Tasks/Src/telemetry_task.o ./Core/Tasks/Src/telemetry_task.su ./Core/Tasks/Src/usb_task.cyclo ./Core/Tasks/Src/usb_task.d ./Core/Tasks/Src/usb_task.o ./Core/Tasks/Src/usb_task.su

.PHONY: clean-Core-2f-Tasks-2f-Src


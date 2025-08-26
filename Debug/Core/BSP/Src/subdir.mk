################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BSP/Src/bsp_buzzer.c \
../Core/BSP/Src/bsp_can.c \
../Core/BSP/Src/bsp_damiao.c \
../Core/BSP/Src/bsp_dbus_input.c \
../Core/BSP/Src/bsp_gpio.c \
../Core/BSP/Src/bsp_hall.c \
../Core/BSP/Src/bsp_imu.c \
../Core/BSP/Src/bsp_led.c \
../Core/BSP/Src/bsp_lk_motor.c \
../Core/BSP/Src/bsp_micros_timer.c \
../Core/BSP/Src/bsp_microswitch.c \
../Core/BSP/Src/bsp_oled.c \
../Core/BSP/Src/bsp_queue.c \
../Core/BSP/Src/bsp_referee.c \
../Core/BSP/Src/bsp_servo.c \
../Core/BSP/Src/bsp_usart.c \
../Core/BSP/Src/bsp_usb_redir.c \
../Core/BSP/Src/crc8_crc16.c 

OBJS += \
./Core/BSP/Src/bsp_buzzer.o \
./Core/BSP/Src/bsp_can.o \
./Core/BSP/Src/bsp_damiao.o \
./Core/BSP/Src/bsp_dbus_input.o \
./Core/BSP/Src/bsp_gpio.o \
./Core/BSP/Src/bsp_hall.o \
./Core/BSP/Src/bsp_imu.o \
./Core/BSP/Src/bsp_led.o \
./Core/BSP/Src/bsp_lk_motor.o \
./Core/BSP/Src/bsp_micros_timer.o \
./Core/BSP/Src/bsp_microswitch.o \
./Core/BSP/Src/bsp_oled.o \
./Core/BSP/Src/bsp_queue.o \
./Core/BSP/Src/bsp_referee.o \
./Core/BSP/Src/bsp_servo.o \
./Core/BSP/Src/bsp_usart.o \
./Core/BSP/Src/bsp_usb_redir.o \
./Core/BSP/Src/crc8_crc16.o 

C_DEPS += \
./Core/BSP/Src/bsp_buzzer.d \
./Core/BSP/Src/bsp_can.d \
./Core/BSP/Src/bsp_damiao.d \
./Core/BSP/Src/bsp_dbus_input.d \
./Core/BSP/Src/bsp_gpio.d \
./Core/BSP/Src/bsp_hall.d \
./Core/BSP/Src/bsp_imu.d \
./Core/BSP/Src/bsp_led.d \
./Core/BSP/Src/bsp_lk_motor.d \
./Core/BSP/Src/bsp_micros_timer.d \
./Core/BSP/Src/bsp_microswitch.d \
./Core/BSP/Src/bsp_oled.d \
./Core/BSP/Src/bsp_queue.d \
./Core/BSP/Src/bsp_referee.d \
./Core/BSP/Src/bsp_servo.d \
./Core/BSP/Src/bsp_usart.d \
./Core/BSP/Src/bsp_usb_redir.d \
./Core/BSP/Src/crc8_crc16.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BSP/Src/%.o Core/BSP/Src/%.su Core/BSP/Src/%.cyclo: ../Core/BSP/Src/%.c Core/BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/Core/BSP/Inc" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/Core/robot_config" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/Core/Tasks/Inc" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/System/BRoCo/include" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/System/Threads/Inc" -I"C:/Users/gskan/STM32CubeIDE/workspace_1.18.1/DM-MC02/System/utils/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BSP-2f-Src

clean-Core-2f-BSP-2f-Src:
	-$(RM) ./Core/BSP/Src/bsp_buzzer.cyclo ./Core/BSP/Src/bsp_buzzer.d ./Core/BSP/Src/bsp_buzzer.o ./Core/BSP/Src/bsp_buzzer.su ./Core/BSP/Src/bsp_can.cyclo ./Core/BSP/Src/bsp_can.d ./Core/BSP/Src/bsp_can.o ./Core/BSP/Src/bsp_can.su ./Core/BSP/Src/bsp_damiao.cyclo ./Core/BSP/Src/bsp_damiao.d ./Core/BSP/Src/bsp_damiao.o ./Core/BSP/Src/bsp_damiao.su ./Core/BSP/Src/bsp_dbus_input.cyclo ./Core/BSP/Src/bsp_dbus_input.d ./Core/BSP/Src/bsp_dbus_input.o ./Core/BSP/Src/bsp_dbus_input.su ./Core/BSP/Src/bsp_gpio.cyclo ./Core/BSP/Src/bsp_gpio.d ./Core/BSP/Src/bsp_gpio.o ./Core/BSP/Src/bsp_gpio.su ./Core/BSP/Src/bsp_hall.cyclo ./Core/BSP/Src/bsp_hall.d ./Core/BSP/Src/bsp_hall.o ./Core/BSP/Src/bsp_hall.su ./Core/BSP/Src/bsp_imu.cyclo ./Core/BSP/Src/bsp_imu.d ./Core/BSP/Src/bsp_imu.o ./Core/BSP/Src/bsp_imu.su ./Core/BSP/Src/bsp_led.cyclo ./Core/BSP/Src/bsp_led.d ./Core/BSP/Src/bsp_led.o ./Core/BSP/Src/bsp_led.su ./Core/BSP/Src/bsp_lk_motor.cyclo ./Core/BSP/Src/bsp_lk_motor.d ./Core/BSP/Src/bsp_lk_motor.o ./Core/BSP/Src/bsp_lk_motor.su ./Core/BSP/Src/bsp_micros_timer.cyclo ./Core/BSP/Src/bsp_micros_timer.d ./Core/BSP/Src/bsp_micros_timer.o ./Core/BSP/Src/bsp_micros_timer.su ./Core/BSP/Src/bsp_microswitch.cyclo ./Core/BSP/Src/bsp_microswitch.d ./Core/BSP/Src/bsp_microswitch.o ./Core/BSP/Src/bsp_microswitch.su ./Core/BSP/Src/bsp_oled.cyclo ./Core/BSP/Src/bsp_oled.d ./Core/BSP/Src/bsp_oled.o ./Core/BSP/Src/bsp_oled.su ./Core/BSP/Src/bsp_queue.cyclo ./Core/BSP/Src/bsp_queue.d ./Core/BSP/Src/bsp_queue.o ./Core/BSP/Src/bsp_queue.su ./Core/BSP/Src/bsp_referee.cyclo ./Core/BSP/Src/bsp_referee.d ./Core/BSP/Src/bsp_referee.o ./Core/BSP/Src/bsp_referee.su ./Core/BSP/Src/bsp_servo.cyclo ./Core/BSP/Src/bsp_servo.d ./Core/BSP/Src/bsp_servo.o ./Core/BSP/Src/bsp_servo.su ./Core/BSP/Src/bsp_usart.cyclo ./Core/BSP/Src/bsp_usart.d ./Core/BSP/Src/bsp_usart.o ./Core/BSP/Src/bsp_usart.su ./Core/BSP/Src/bsp_usb_redir.cyclo ./Core/BSP/Src/bsp_usb_redir.d ./Core/BSP/Src/bsp_usb_redir.o ./Core/BSP/Src/bsp_usb_redir.su ./Core/BSP/Src/crc8_crc16.cyclo ./Core/BSP/Src/crc8_crc16.d ./Core/BSP/Src/crc8_crc16.o ./Core/BSP/Src/crc8_crc16.su

.PHONY: clean-Core-2f-BSP-2f-Src


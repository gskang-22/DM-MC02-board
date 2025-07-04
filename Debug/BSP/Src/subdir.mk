################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/Src/BMI088Middleware.c \
../BSP/Src/BMI088driver.c \
../BSP/Src/bsp_fdcan.c \
../BSP/Src/buzzer.c \
../BSP/Src/j60_10.c \
../BSP/Src/uart_bsp.c 

OBJS += \
./BSP/Src/BMI088Middleware.o \
./BSP/Src/BMI088driver.o \
./BSP/Src/bsp_fdcan.o \
./BSP/Src/buzzer.o \
./BSP/Src/j60_10.o \
./BSP/Src/uart_bsp.o 

C_DEPS += \
./BSP/Src/BMI088Middleware.d \
./BSP/Src/BMI088driver.d \
./BSP/Src/bsp_fdcan.d \
./BSP/Src/buzzer.d \
./BSP/Src/j60_10.d \
./BSP/Src/uart_bsp.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/Src/%.o BSP/Src/%.su BSP/Src/%.cyclo: ../BSP/Src/%.c BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/User/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/Algo/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/BSP/Inc" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/BSP" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/Algo" -I"C:/Users/YI MING/Desktop/MC02_template2/CtrBoard-H7_FDCAN/User" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP-2f-Src

clean-BSP-2f-Src:
	-$(RM) ./BSP/Src/BMI088Middleware.cyclo ./BSP/Src/BMI088Middleware.d ./BSP/Src/BMI088Middleware.o ./BSP/Src/BMI088Middleware.su ./BSP/Src/BMI088driver.cyclo ./BSP/Src/BMI088driver.d ./BSP/Src/BMI088driver.o ./BSP/Src/BMI088driver.su ./BSP/Src/bsp_fdcan.cyclo ./BSP/Src/bsp_fdcan.d ./BSP/Src/bsp_fdcan.o ./BSP/Src/bsp_fdcan.su ./BSP/Src/buzzer.cyclo ./BSP/Src/buzzer.d ./BSP/Src/buzzer.o ./BSP/Src/buzzer.su ./BSP/Src/j60_10.cyclo ./BSP/Src/j60_10.d ./BSP/Src/j60_10.o ./BSP/Src/j60_10.su ./BSP/Src/uart_bsp.cyclo ./BSP/Src/uart_bsp.d ./BSP/Src/uart_bsp.o ./BSP/Src/uart_bsp.su

.PHONY: clean-BSP-2f-Src


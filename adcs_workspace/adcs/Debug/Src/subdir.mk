################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adcs_configuration.c \
../Src/adcs_state.c \
../Src/aries.c \
../Src/geomag.c \
../Src/main.c \
../Src/satutlSTM32.c \
../Src/sgdp4.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c 

OBJS += \
./Src/adcs_configuration.o \
./Src/adcs_state.o \
./Src/aries.o \
./Src/geomag.o \
./Src/main.o \
./Src/satutlSTM32.o \
./Src/sgdp4.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o 

C_DEPS += \
./Src/adcs_configuration.d \
./Src/adcs_state.d \
./Src/aries.d \
./Src/geomag.d \
./Src/main.d \
./Src/satutlSTM32.d \
./Src/sgdp4.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F405xx -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Inc" -I"/home/azisi/Documents/UPSat/ecss_services/platform/adcs" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/CMSIS/Include" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/azisi/Documents/UPSat/ecss_services/services"  -O0 -g3 -Wall -fmessage-length=0 -mlittle-endian -DARM_MATH_CM4=1 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



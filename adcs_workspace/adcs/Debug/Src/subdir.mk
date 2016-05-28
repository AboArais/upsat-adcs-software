################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adcs_configuration.c \
../Src/adcs_control.c \
../Src/adcs_state.c \
../Src/aries.c \
../Src/geomag.c \
../Src/gps_pqNAV_L1.c \
../Src/main.c \
../Src/sgdp4.c \
../Src/sgp4ext.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/sun_pos.c 

OBJS += \
./Src/adcs_configuration.o \
./Src/adcs_control.o \
./Src/adcs_state.o \
./Src/aries.o \
./Src/geomag.o \
./Src/gps_pqNAV_L1.o \
./Src/main.o \
./Src/sgdp4.o \
./Src/sgp4ext.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/sun_pos.o 

C_DEPS += \
./Src/adcs_configuration.d \
./Src/adcs_control.d \
./Src/adcs_state.d \
./Src/aries.d \
./Src/geomag.d \
./Src/gps_pqNAV_L1.d \
./Src/main.d \
./Src/sgdp4.d \
./Src/sgp4ext.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/sun_pos.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F405xx -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Inc" -I"/home/azisi/Documents/UPSat/ecss_services/core" -I"/home/azisi/Documents/UPSat/ecss_services/platform/adcs" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/CMSIS/Include" -I"/home/azisi/Documents/UPSat/upsat-adcs-software/adcs_workspace/adcs/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/azisi/Documents/UPSat/ecss_services/services"  -O0 -g3 -Wall -fmessage-length=0 -mlittle-endian -DARM_MATH_CM4=1 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



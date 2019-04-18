################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
M:/HC/BMS/GIT/BMS/Bootloader/Src/common.c \
M:/HC/BMS/GIT/BMS/Bootloader/Src/flash_if.c \
M:/HC/BMS/GIT/BMS/Bootloader/Src/main.c \
M:/HC/BMS/GIT/BMS/Bootloader/Src/menu.c \
M:/HC/BMS/GIT/BMS/Bootloader/Src/stm32f1xx_it.c \
M:/HC/BMS/GIT/BMS/Bootloader/Src/system_stm32f1xx.c \
M:/HC/BMS/GIT/BMS/Bootloader/Src/ymodem.c 

OBJS += \
./Src/common.o \
./Src/flash_if.o \
./Src/main.o \
./Src/menu.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/ymodem.o 

C_DEPS += \
./Src/common.d \
./Src/flash_if.d \
./Src/main.d \
./Src/menu.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/ymodem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/common.o: M:/HC/BMS/GIT/BMS/Bootloader/Src/common.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/STM32F1xx_HAL_Driver/Inc" -I"M:/HC/BMS/GIT/BMS/Bootloader/Inc" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/flash_if.o: M:/HC/BMS/GIT/BMS/Bootloader/Src/flash_if.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/STM32F1xx_HAL_Driver/Inc" -I"M:/HC/BMS/GIT/BMS/Bootloader/Inc" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/main.o: M:/HC/BMS/GIT/BMS/Bootloader/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/STM32F1xx_HAL_Driver/Inc" -I"M:/HC/BMS/GIT/BMS/Bootloader/Inc" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/menu.o: M:/HC/BMS/GIT/BMS/Bootloader/Src/menu.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/STM32F1xx_HAL_Driver/Inc" -I"M:/HC/BMS/GIT/BMS/Bootloader/Inc" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/stm32f1xx_it.o: M:/HC/BMS/GIT/BMS/Bootloader/Src/stm32f1xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/STM32F1xx_HAL_Driver/Inc" -I"M:/HC/BMS/GIT/BMS/Bootloader/Inc" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/system_stm32f1xx.o: M:/HC/BMS/GIT/BMS/Bootloader/Src/system_stm32f1xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/STM32F1xx_HAL_Driver/Inc" -I"M:/HC/BMS/GIT/BMS/Bootloader/Inc" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/ymodem.o: M:/HC/BMS/GIT/BMS/Bootloader/Src/ymodem.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"M:/HC/BMS/GIT/BMS/Drivers/STM32F1xx_HAL_Driver/Inc" -I"M:/HC/BMS/GIT/BMS/Bootloader/Inc" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



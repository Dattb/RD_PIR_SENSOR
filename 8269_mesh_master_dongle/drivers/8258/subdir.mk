################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/8258/flash.c 

OBJS += \
./drivers/8258/flash.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/8258/%.o: ../drivers/8258/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_PIR_15_12_2020\firmware" -D__PROJECT_8267_MASTER_KMA_DONGLE__=1 -DCHIP_TYPE=CHIP_TYPE_8269 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



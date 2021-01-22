################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/Rang_Dong/RD_Flash.c \
../vendor/Rang_Dong/rd_sensor_tx.c 

OBJS += \
./vendor/Rang_Dong/RD_Flash.o \
./vendor/Rang_Dong/rd_sensor_tx.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/Rang_Dong/%.o: ../vendor/Rang_Dong/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_PIR_15_12_2020\firmware" -I"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_PIR_15_12_2020\firmware\vendor\common\mi_api\mijia_ble_api" -I"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_PIR_15_12_2020\firmware\vendor\common\mi_api\libs" -D__PROJECT_MESH_LPN__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



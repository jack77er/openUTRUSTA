################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/cmsis/system_stm32l0xx.c \
../system/src/cmsis/vectors_stm32l0xx.c 

S_UPPER_SRCS += \
../system/src/cmsis/startup_stm32l051xx.S 

OBJS += \
./system/src/cmsis/startup_stm32l051xx.o \
./system/src/cmsis/system_stm32l0xx.o \
./system/src/cmsis/vectors_stm32l0xx.o 

S_UPPER_DEPS += \
./system/src/cmsis/startup_stm32l051xx.d 

C_DEPS += \
./system/src/cmsis/system_stm32l0xx.d \
./system/src/cmsis/vectors_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/cmsis/%.o: ../system/src/cmsis/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wuninitialized -Wmissing-declarations  -g3 -x assembler-with-cpp -DDEBUG -DSTM32L051xx -DUSE_HAL_DRIVER -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32l0xx" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/cmsis/%.o: ../system/src/cmsis/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wuninitialized -Wmissing-declarations  -g3 -DDEBUG -DSTM32L051xx -DUSE_HAL_DRIVER -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32l0xx" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



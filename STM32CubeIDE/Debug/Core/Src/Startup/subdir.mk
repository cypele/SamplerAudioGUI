################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/TouchGFXProjects/SamplerAudio1/Core/Src/Startup/startup_stm32f769nihx.s 

S_DEPS += \
./Core/Src/Startup/startup_stm32f769nihx.d 

OBJS += \
./Core/Src/Startup/startup_stm32f769nihx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Startup/startup_stm32f769nihx.o: C:/TouchGFXProjects/SamplerAudio1/Core/Src/Startup/startup_stm32f769nihx.s Core/Src/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I../../Core/Inc -I../../LIBJPEG/App -I../../LIBJPEG/Target -I../../TouchGFX/App -I../../TouchGFX/target/generated -I../../TouchGFX/target -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Middlewares/Third_Party/LibJPEG/include -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/Components/Common -I../../FATFS/Target -I../../FATFS/App -I../../Middlewares/Third_Party/FatFs/src -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/generated/videos/include -I../../TouchGFX/gui/include -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Src-2f-Startup

clean-Core-2f-Src-2f-Startup:
	-$(RM) ./Core/Src/Startup/startup_stm32f769nihx.d ./Core/Src/Startup/startup_stm32f769nihx.o

.PHONY: clean-Core-2f-Src-2f-Startup


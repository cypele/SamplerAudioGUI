################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/common/FrontendApplication.cpp \
C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/model/Model.cpp \
C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/screen1_screen/Screen1Presenter.cpp \
C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/screen1_screen/Screen1View.cpp 

OBJS += \
./Application/User/gui/FrontendApplication.o \
./Application/User/gui/Model.o \
./Application/User/gui/Screen1Presenter.o \
./Application/User/gui/Screen1View.o 

CPP_DEPS += \
./Application/User/gui/FrontendApplication.d \
./Application/User/gui/Model.d \
./Application/User/gui/Screen1Presenter.d \
./Application/User/gui/Screen1View.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/gui/FrontendApplication.o: C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/common/FrontendApplication.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DUSE_BPP=16 -DSTM32F769xx -DDEBUG -c -I../../Core/Inc -I../../Drivers/CMSIS/Include -I../../TouchGFX/target -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../TouchGFX/App -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../TouchGFX/target/generated -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Drivers/BSP/Components/Common -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../LIBJPEG/App -I../../LIBJPEG/Target -I../../Utilities/JPEG -I../../Middlewares/Third_Party/LibJPEG/include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/generated/videos/include -I../../TouchGFX/gui/include -I../../FATFS/Target -I../../FATFS/App -I../../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/gui/Model.o: C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/model/Model.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DUSE_BPP=16 -DSTM32F769xx -DDEBUG -c -I../../Core/Inc -I../../Drivers/CMSIS/Include -I../../TouchGFX/target -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../TouchGFX/App -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../TouchGFX/target/generated -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Drivers/BSP/Components/Common -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../LIBJPEG/App -I../../LIBJPEG/Target -I../../Utilities/JPEG -I../../Middlewares/Third_Party/LibJPEG/include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/generated/videos/include -I../../TouchGFX/gui/include -I../../FATFS/Target -I../../FATFS/App -I../../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/gui/Screen1Presenter.o: C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/screen1_screen/Screen1Presenter.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DUSE_BPP=16 -DSTM32F769xx -DDEBUG -c -I../../Core/Inc -I../../Drivers/CMSIS/Include -I../../TouchGFX/target -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../TouchGFX/App -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../TouchGFX/target/generated -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Drivers/BSP/Components/Common -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../LIBJPEG/App -I../../LIBJPEG/Target -I../../Utilities/JPEG -I../../Middlewares/Third_Party/LibJPEG/include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/generated/videos/include -I../../TouchGFX/gui/include -I../../FATFS/Target -I../../FATFS/App -I../../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/gui/Screen1View.o: C:/TouchGFXProjects/SamplerAudio1/TouchGFX/gui/src/screen1_screen/Screen1View.cpp Application/User/gui/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DUSE_BPP=16 -DSTM32F769xx -DDEBUG -c -I../../Core/Inc -I../../Drivers/CMSIS/Include -I../../TouchGFX/target -I../../Drivers/STM32F7xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../../TouchGFX/App -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../TouchGFX/target/generated -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../../Drivers/BSP/Components/Common -I../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../LIBJPEG/App -I../../LIBJPEG/Target -I../../Utilities/JPEG -I../../Middlewares/Third_Party/LibJPEG/include -I../../Middlewares/ST/touchgfx/framework/include -I../../TouchGFX/generated/fonts/include -I../../TouchGFX/generated/gui_generated/include -I../../TouchGFX/generated/images/include -I../../TouchGFX/generated/texts/include -I../../TouchGFX/generated/videos/include -I../../TouchGFX/gui/include -I../../FATFS/Target -I../../FATFS/App -I../../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-gui

clean-Application-2f-User-2f-gui:
	-$(RM) ./Application/User/gui/FrontendApplication.cyclo ./Application/User/gui/FrontendApplication.d ./Application/User/gui/FrontendApplication.o ./Application/User/gui/FrontendApplication.su ./Application/User/gui/Model.cyclo ./Application/User/gui/Model.d ./Application/User/gui/Model.o ./Application/User/gui/Model.su ./Application/User/gui/Screen1Presenter.cyclo ./Application/User/gui/Screen1Presenter.d ./Application/User/gui/Screen1Presenter.o ./Application/User/gui/Screen1Presenter.su ./Application/User/gui/Screen1View.cyclo ./Application/User/gui/Screen1View.d ./Application/User/gui/Screen1View.o ./Application/User/gui/Screen1View.su

.PHONY: clean-Application-2f-User-2f-gui


################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/flash_if.c \
C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/main.c \
C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stm32l4xx_hal_msp.c \
C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stm32l4xx_it.c \
C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stsafea_crypto_mbedtls_interface.c \
C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stsafea_service_interface.c \
C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/system_stm32l4xx.c 

OBJS += \
./Application/User/flash_if.o \
./Application/User/main.o \
./Application/User/stm32l4xx_hal_msp.o \
./Application/User/stm32l4xx_it.o \
./Application/User/stsafea_crypto_mbedtls_interface.o \
./Application/User/stsafea_service_interface.o \
./Application/User/system_stm32l4xx.o 

C_DEPS += \
./Application/User/flash_if.d \
./Application/User/main.d \
./Application/User/stm32l4xx_hal_msp.d \
./Application/User/stm32l4xx_it.d \
./Application/User/stsafea_crypto_mbedtls_interface.d \
./Application/User/stsafea_service_interface.d \
./Application/User/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/flash_if.o: C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/flash_if.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 '-DMBEDTLS_CONFIG_FILE=<config_mbedtls.h>' -DSTSAFE_A110 -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../Linker_Common/SW4STM32 -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/flash_if.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/main.o: C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 '-DMBEDTLS_CONFIG_FILE=<config_mbedtls.h>' -DSTSAFE_A110 -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../Linker_Common/SW4STM32 -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32l4xx_hal_msp.o: C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stm32l4xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 '-DMBEDTLS_CONFIG_FILE=<config_mbedtls.h>' -DSTSAFE_A110 -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../Linker_Common/SW4STM32 -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/stm32l4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32l4xx_it.o: C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stm32l4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 '-DMBEDTLS_CONFIG_FILE=<config_mbedtls.h>' -DSTSAFE_A110 -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../Linker_Common/SW4STM32 -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/stm32l4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stsafea_crypto_mbedtls_interface.o: C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stsafea_crypto_mbedtls_interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 '-DMBEDTLS_CONFIG_FILE=<config_mbedtls.h>' -DSTSAFE_A110 -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../Linker_Common/SW4STM32 -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/stsafea_crypto_mbedtls_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stsafea_service_interface.o: C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/stsafea_service_interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 '-DMBEDTLS_CONFIG_FILE=<config_mbedtls.h>' -DSTSAFE_A110 -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../Linker_Common/SW4STM32 -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/stsafea_service_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/system_stm32l4xx.o: C:/AWS/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/STSAFE_Provisioning/Src/system_stm32l4xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 '-DMBEDTLS_CONFIG_FILE=<config_mbedtls.h>' -DSTSAFE_A110 -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include -I../../../../../../../../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../Linker_Common/SW4STM32 -I../../../../../../../../Drivers/CMSIS/Include -Os -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/system_stm32l4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc_ex.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_firewall.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c 

OBJS += \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_cortex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc_ex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_firewall.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_gpio.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c_ex.o 

C_DEPS += \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_cortex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc_ex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_firewall.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_gpio.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_cortex.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_cortex.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc_ex.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_crc_ex.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_firewall.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_firewall.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_firewall.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ex.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ex.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_gpio.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c_ex.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c_ex.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

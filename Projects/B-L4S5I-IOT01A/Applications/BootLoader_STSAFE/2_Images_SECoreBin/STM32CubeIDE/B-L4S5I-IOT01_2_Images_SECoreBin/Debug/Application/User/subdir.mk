################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/kms_ext_token_stsafe.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/kms_low_level.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/nvms_low_level.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/se_low_level.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/stsafe_platf_objects.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/stsafea_crypto_stlib_interface.c \
C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/stsafea_service_interface.c 

OBJS += \
./Application/User/kms_ext_token_stsafe.o \
./Application/User/kms_low_level.o \
./Application/User/nvms_low_level.o \
./Application/User/se_low_level.o \
./Application/User/stsafe_platf_objects.o \
./Application/User/stsafea_crypto_stlib_interface.o \
./Application/User/stsafea_service_interface.o 

C_DEPS += \
./Application/User/kms_ext_token_stsafe.d \
./Application/User/kms_low_level.d \
./Application/User/nvms_low_level.d \
./Application/User/se_low_level.d \
./Application/User/stsafe_platf_objects.d \
./Application/User/stsafea_crypto_stlib_interface.d \
./Application/User/stsafea_service_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/kms_ext_token_stsafe.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/kms_ext_token_stsafe.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Application/User/kms_ext_token_stsafe.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Application/User/kms_low_level.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/kms_low_level.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Application/User/kms_low_level.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Application/User/nvms_low_level.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/nvms_low_level.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Application/User/nvms_low_level.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Application/User/se_low_level.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/se_low_level.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Application/User/se_low_level.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Application/User/stsafe_platf_objects.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/stsafe_platf_objects.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Application/User/stsafe_platf_objects.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Application/User/stsafea_crypto_stlib_interface.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/stsafea_crypto_stlib_interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Application/User/stsafea_crypto_stlib_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"
Application/User/stsafea_service_interface.o: C:/en.x-cube-aws_v2-0-0/STM32CubeExpansion_Cloud_AWS_V2.0.0/Projects/B-L4S5I-IOT01A/Applications/BootLoader_STSAFE/2_Images_SECoreBin/Src/stsafea_service_interface.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32L4S5xx -DUSE_HAL_DRIVER -DUSE_STM32L475E_IOT01 -DKMS_ENABLED -DSTSAFE_A110 '-DMBEDTLS_CONFIG_FILE=<../Inc/mbed_crypto_config.h>' -DENABLE_IMAGE_STATE_HANDLING -c -I../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/B-L475E-IOT01 -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Core -I../../../../../../../../Middlewares/ST/STM32_Secure_Engine/Key -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Core -I../../../../../../../../Middlewares/ST/STM32_Key_Management_Services/Interface -I../../../../../../../../Middlewares/ST/STSAFE_A1xx/CoreModules/Inc -I../../../../2_Images_SBSFU/SBSFU/App -I../../../../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc -I../../../../../../../../Drivers/CMSIS/Include -I../../ -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include -I../../../../Linker_Common/STM32CubeIDE -I../../../../../../../../Middlewares/Third_Party/mbed-crypto/include/mbedtls -Os -ffunction-sections -Wall -Wno-strict-aliasing -fstack-usage -MMD -MP -MF"Application/User/stsafea_service_interface.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"


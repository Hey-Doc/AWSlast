@echo off
echo prebuild.bat : started > %1\\output.txt
set "ecckey=%1\\..\\Binary\\ECCKEY.txt"
set "asmfile=%1\\se_key.s"
set "SBSFUBootLoader=%~d0%~p0\\..\\.."
::comment this line to force python
::python is used if windows executeable not found
pushd %1\..\..\..\..\..\..\Middlewares\ST\STM32_Secure_Engine\Utilities\KeysAndImages
set basedir=%cd%
popd
goto exe:
goto py:
:exe
::line for window executeable
echo Prebuild with windows executable
set "prepareimage=%basedir%\\win\\prepareimage\\prepareimage.exe"
set "python="
if exist %prepareimage% (
goto prebuild
)
:py
::line for python
echo Prebuild with python script
set "prepareimage=%basedir%\\prepareimage.py"
set "python=python "
echo "python: %prepareimage%" >> %1\\output.txt 2>>&1
:prebuild
set "crypto_h=%1\\..\\Inc\\se_crypto_config.h"

::clean
if exist %1\\crypto.txt (
  del %1\\crypto.txt
)
if exist %asmfile% (
  del %asmfile%
)
if exist %1\\postbuild.bat (
  del %1\\postbuild.bat
)

::get crypto name
set "command=%python%%prepareimage% conf %crypto_h% > %1\\crypto.txt"
%command%
IF %ERRORLEVEL% NEQ 0 goto :error
set /P crypto=<%1\\crypto.txt >> %1\\output.txt 2>>&1
echo crypto %crypto% selected >> %1\\output.txt 2>>&1

if "%crypto%"=="SECBOOT_AES128_GCM_AES128_GCM_AES128_GCM" goto AES128_GCM_AES128_GCM_AES128_GCM
if "%crypto%"=="SECBOOT_ECCDSA_WITH_AES128_CBC_SHA256" goto ECCDSA_WITH_AES128_CBC_SHA256
if "%crypto%"=="SECBOOT_ECCDSA_WITHOUT_ENCRYPT_SHA256" goto ECCDSA_WITHOUT_ENCRYPT_SHA256
if "%crypto%"=="SECBOOT_X509_ECDSA_WITHOUT_ENCRYPT_SHA256" goto X509_ECDSA_WITHOUT_ENCRYPT_SHA256
goto end

:AES128_GCM_AES128_GCM_AES128_GCM
goto end

:ECCDSA_WITH_AES128_CBC_SHA256
goto end

:ECCDSA_WITHOUT_ENCRYPT_SHA256
goto end

:X509_ECDSA_WITHOUT_ENCRYPT_SHA256
set "oemkey=%1\\..\\..\\STSAFE_Provisioning\\Binary\\STSAFE_PAIRING_keys.bin"
set "command=%python%%prepareimage% trans -k %oemkey% -f SE_ReadKey_Pairing -s .SE_Key_Data -e 1 -v V7M > %asmfile%"
%command%
IF %ERRORLEVEL% NEQ 0 goto :error
goto end

:end
::Patch KMS embedded keys with Blob Keys
set "kmsblob_ecckey=%SBSFUBootLoader%\\2_Images_KMS_Blob\\Binary\\ECCKEY.txt"
set "command=%python%%prepareimage% inject -k %kmsblob_ecckey% -f %1\..\Inc\kms_platf_objects_config.h.pattern -p @ECDSA_BLOB_KEY@ %1\kms_platf_objects_config.h.tmp"
%command%
IF %ERRORLEVEL% NEQ 0 goto :error
set "kmsblob_oemkey=%SBSFUBootLoader%\\2_Images_KMS_Blob\\Binary\\OEM_KEY_COMPANY1_key_AES_CBC.bin"
set "command=%python%%prepareimage% inject -k %kmsblob_oemkey% -f %1\kms_platf_objects_config.h.tmp -p @AES_BLOB_KEY@ %1\kms_platf_objects_config.h"
%command%
IF %ERRORLEVEL% NEQ 0 goto :error
del %1\\kms_platf_objects_config.h.tmp

set "command=copy %1\\%crypto%.bat %1\\postbuild.bat"
%command%
IF %ERRORLEVEL% NEQ 0 goto :error
exit 0
:error
echo %command% : failed >> %1\\output.txt 2>&1
echo %command% : failed
pause
exit 1
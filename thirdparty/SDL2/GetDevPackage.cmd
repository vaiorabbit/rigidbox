::
:: Usage:
:: - %1 : SDL2 version string (e.g. 2.0.12)
::

@echo off
setlocal EnableDelayedExpansion

:: Build common variables
set VERSION=%1
if "%1"=="" (
    set VERSION=2.0.12
)
set ARCHIVE=SDL2-devel-%VERSION%-VC.zip
set EXPANDED_PATH=SDL2-%VERSION%

:: Create temporary directory
if not exist .\tmp (
    mkdir tmp
    if !ERRORLEVEL! neq 0 goto error
)

pushd
cd tmp

:: Get SDL2 developer archive and then extract
if not exist %ARCHIVE% (
    curl https://www.libsdl.org/release/!ARCHIVE! -o !ARCHIVE!
    if !ERRORLEVEL! neq 0 goto error
    powershell Expand-Archive -Path !ARCHIVE! -DestinationPath . -Force
    if !ERRORLEVEL! neq 0 goto error
)

pushd
cd %EXPANDED_PATH%

:: Copy headers/libraries
xcopy /y include\* ..\..\include\
if !ERRORLEVEL! neq 0 goto error
xcopy /y lib\x64\* ..\..\lib\
if !ERRORLEVEL! neq 0 goto error

popd

:: rmdir /s /q %EXPANDED_PATH%

popd

echo setup SDL2 headers/libraries successfully
pause
exit /b 0

:error
echo failed to setup SDL2 headers/libraries
pause
exit /b 1

endlocal

::
:: Usage:
:: - %1 : imgui version string (e.g. 1.76)
::

@echo off
setlocal EnableDelayedExpansion

:: Build common variables
set VERSION=%1
if "%1"=="" (
    set VERSION=1.76
)
set ARCHIVE=v%VERSION%.zip
set EXPANDED_PATH=imgui-%VERSION%

:: Create temporary directory
if not exist .\tmp (
    mkdir tmp
    if !ERRORLEVEL! neq 0 goto error
)

pushd
cd tmp

:: Get imgui developer archive and then extract
if not exist %ARCHIVE% (
    curl https://codeload.github.com/ocornut/imgui/zip/v!VERSION! -o !ARCHIVE!
    if !ERRORLEVEL! neq 0 goto error
    powershell Expand-Archive -Path !ARCHIVE! -DestinationPath . -Force
    if !ERRORLEVEL! neq 0 goto error
)

pushd
cd %EXPANDED_PATH%

:: Copy headers/libraries
xcopy /y .\*.cpp ..\..\imgui\
if !ERRORLEVEL! neq 0 goto error
xcopy /y .\*.h ..\..\imgui\
if !ERRORLEVEL! neq 0 goto error
xcopy /y .\examples\imgui_impl_opengl2.* ..\..\imgui\
if !ERRORLEVEL! neq 0 goto error
xcopy /y .\examples\imgui_impl_sdl.* ..\..\imgui\
if !ERRORLEVEL! neq 0 goto error

popd

:: rmdir /s /q %EXPANDED_PATH%

popd

echo setup imgui headers/libraries successfully
pause
exit /b 0

:error
echo failed to setup imgui headers/libraries
pause
exit /b 1

endlocal

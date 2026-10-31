@echo off
setlocal
set "tdGuiExe=D:\Anlogic\TD5.6.2\bin\td.exe"
set "tdTempDirectory=%~dp0build\td_temp"
if not exist "%tdGuiExe%" (
    echo TD5.6.2 executable was not found: "%tdGuiExe%"
    pause
    exit /b 1
)
if not exist "%tdTempDirectory%" mkdir "%tdTempDirectory%"
if not exist "%tdTempDirectory%" (
    echo Unable to create TD temporary directory: "%tdTempDirectory%"
    pause
    exit /b 1
)
set "TEMP=%tdTempDirectory%"
set "TMP=%tdTempDirectory%"
start "" "%tdGuiExe%" "%~dp0src\td_project\func1.al"
if errorlevel 1 (
    echo TD failed to start.
    pause
    exit /b 1
)
exit /b 0

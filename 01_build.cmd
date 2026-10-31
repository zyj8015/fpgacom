@echo off
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1"
set "build_exit_code=%errorlevel%"
if not "%build_exit_code%"=="0" echo Build failed. Inspect the error and build log before programming the board.
pause
exit /b %build_exit_code%

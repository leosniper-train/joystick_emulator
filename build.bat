@echo off
REM Build a standalone JoystickEmulator.exe for Windows (no Python required on target PCs).
setlocal
cd /d "%~dp0"

echo Installing build dependencies...
py -m pip install -r requirements.txt -r requirements-build.txt
if errorlevel 1 goto :fail

echo.
echo Building with PyInstaller...
py -m PyInstaller --noconfirm --clean joystick_emulator.spec
if errorlevel 1 goto :fail

echo.
echo Done. Output:
echo   dist\JoystickEmulator.exe
echo.
echo Copy that .exe to any Windows PC and double-click to run.
echo config.json is created next to the .exe on first launch.
goto :eof

:fail
echo Build failed.
exit /b 1

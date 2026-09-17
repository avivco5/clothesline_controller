@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"

set PORT=%1
if "%PORT%"=="" set PORT=COM4

where esptool >nul 2>nul
if errorlevel 1 (
    echo [ERROR] esptool not found on this computer.
    echo Install it once with:
    echo     pip install esptool
    echo Then run this script again.
    echo.
    pause
    exit /b 1
)

if not exist "firmware.bin" (
    echo [ERROR] firmware.bin not found next to flash.bat.
    pause
    exit /b 1
)

echo ================================================
echo  Flashing Clothesline-Control firmware to %PORT%
echo ================================================
esptool --chip esp8266 --port %PORT% --baud 460800 write_flash 0x0 firmware.bin
if errorlevel 1 (
    echo.
    echo [FAILED] Flashing failed -- see errors above.
    echo Check the board is connected, drivers installed, and %PORT% is correct.
    echo Usage: flash.bat [COM_PORT]
    echo Find the port number in Windows Device Manager under "Ports (COM and LPT)".
    echo.
    pause
    exit /b 1
)

echo.
echo ================================================
echo  Done. Flashed successfully to %PORT%.
echo  Connect to WiFi "Clothesline-Control" (password: clothesline)
echo  and open http://192.168.4.1
echo ================================================
pause

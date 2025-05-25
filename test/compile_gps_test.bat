@echo off
REM Compile the GPS_Test_Serial sketch

REM For Teensy 4.0
arduino-cli compile --fqbn teensy:avr:teensy40 GPS_Test_Serial.ino

REM Uncomment for STM32
REM arduino-cli compile --fqbn STMicroelectronics:stm32:GenF4 GPS_Test_Serial.ino

echo.
echo To upload to your board, run:
echo arduino-cli upload -p COM_PORT --fqbn teensy:avr:teensy40 GPS_Test_Serial.ino
echo Replace COM_PORT with your actual port (e.g., COM3) 
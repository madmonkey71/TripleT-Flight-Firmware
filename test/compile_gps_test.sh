#!/bin/bash
# Compile the GPS_Test_Serial sketch

# For Teensy 4.0
arduino-cli compile --fqbn teensy:avr:teensy40 GPS_Test_Serial.ino

# Uncomment for STM32
# arduino-cli compile --fqbn STMicroelectronics:stm32:GenF4 GPS_Test_Serial.ino

echo ""
echo "To upload to your board, run:"
echo "arduino-cli upload -p /dev/ttyACM0 --fqbn teensy:avr:teensy40 GPS_Test_Serial.ino"
echo "Replace /dev/ttyACM0 with your actual port" 
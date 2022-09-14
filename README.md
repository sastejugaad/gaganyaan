# Gagayaan File structure
  DMP_Wireless for testing.\
  Gganyaan_with_cam main ESP32 file.\
  Gaganyaan_UI processing file for dashboard
# Install libraries
  Install processing 3.5 https://processing.org/download \
  Install toxilab library in processing. Extract both folder and place in Documents>Processing>Libraries \
  Copy eloquent vision to arduino library folder. 
# Adding ESP32 board to  arduino library
  Copy this line to Arduino IDE File>Preferences>Additonal boards manager URL. 
  http://arduino.esp8266.com/stable/package_esp8266com_index.json,https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
# Circuit
 Pins to  connect ESP32 to MPU 6050 pins
 Pin 2 < Int \
 Pin 14 < SDA \
 Pin 15 < SCL \
 3V < VCC or Vin \
 Gnd < Gnd \
 Since we are only using MPU6050 there are only 3 connections. 

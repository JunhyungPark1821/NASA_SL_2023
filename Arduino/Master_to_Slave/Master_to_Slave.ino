// OpenMV Cam Master I2C Data  (P5) - Teensy SDA (18)
// OpenMV Cam Master I2C Clock (P4) - Teensy SCL (19)
// OpenMV Cam Ground                - Arduino Ground

#include <Wire.h> 

String tasks = "CDCECFCGCHC";

void setup() {
  // Start the I2C Bus as Master
  Wire.begin(); 
}

void loop() {
  Wire.beginTransmission(0x12); // transmit to device with address 12
  Wire.write(tasks.c_str()); // send the string over I2C
  Wire.endTransmission(); // stop transmitting
  delay(1000); // wait for 1 second
}

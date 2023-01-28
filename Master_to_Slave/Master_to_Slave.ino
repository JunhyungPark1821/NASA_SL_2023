#include <Wire.h> 

String tasks = "A1B2C3D4E5F6G7H8";

void setup() {
  // Start the I2C Bus as Master
  Wire.begin(); 
}

void loop() {
  Wire.beginTransmission(8); // transmit to device with address 8
  Wire.write(tasks.c_str()); // send the string over I2C
  Wire.endTransmission(); // stop transmitting
  delay(1000); // wait for 1 second
}

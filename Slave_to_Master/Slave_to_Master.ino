#include <Wire.h>

String incomingData;

void setup() {
  Serial.begin(9600);
  Wire.begin(8); // join i2c bus with address 8
}

void loop() {
  // do something here
  Wire.onReceive(receiveEvent); // register event
  delay(1000);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent() {
  incomingData = "";
  while (Wire.available()) {
    char c = Wire.read();
    incomingData += c;
  }
  Serial.println(incomingData);
}

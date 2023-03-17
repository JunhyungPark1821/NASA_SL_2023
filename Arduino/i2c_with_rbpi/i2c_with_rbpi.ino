#include <Wire.h>

#define SLAVE_ADDRESS 0x08

boolean landed = true;
boolean sent = false;
boolean received = false;

char tasks[100];

int charIndex = 0;

void setup() {
  Wire.begin(SLAVE_ADDRESS);        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {
  while (Wire.available() && landed && !sent) { // peripheral may send less than requested
    Wire.write(1);
    delay(1000);
    sent = true;
  }
  
  while (Wire.available() && !received) {
    char c = Wire.read(); // receive a byte as character
    tasks[charIndex] = c;
    charIndex++;
    if (c == '6') {
      received = true;
      for (int i = 0; i<charIndex; i++) {
        Serial.print(tasks[i]);
      }
    }
  }
}

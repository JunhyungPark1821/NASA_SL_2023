#include "FeedBackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 3

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

int motorDegree = 0;

void setup() {
    // set servo control pin number
    servo.setServoControl(SERVO_PIN); // Sets where it is as 0 deg
    servo.setKp(1.0);
}

void loop() {
  motorDegree += 60; // Assume it is all right 60 deg turn
  if (motorDegree <= 180) {
    servo.rotate(motorDegree, 1);
    delay(1000);
  }
  else {
    servo.rotate(-120, 1);
    delay(1000);
    motorDegree = -120;
  }
}

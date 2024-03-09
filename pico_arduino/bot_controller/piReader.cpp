#include "piReader.h"

// default constructor uses Serial with pins GP0 and GP1
SerialInstructions::SerialInstructions() {

  // init speed and direction
  target_speed = 0;
  target_direction = 0;

  // set up Serial1
  Serial1.setTimeout(10);

  // clear garbage
  while (Serial1.available()) {
    Serial1.readString();
  }

  // Set up ISR
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), readISR, RISING);
}

// ISR to read and parse data from buffer
void SerialInstructions::readISR() {

  if (Serial1.available()) {
    
    // read the string
    arduino::String speed_str = Serial1.readStringUntil(',');
    arduino::String dir_str = Serial1.readString();
    // parse the string
    if (speed_str[0] != '\0' && dir_str[0] != '\0') {
      short speed = speed_str.toInt();
      short dir = dir_str.toInt();

      // store values
      target_speed = speed;
      target_direction = dir;
    }
  }

}

// get current target speed and direction values
short SerialInstructions::getTargetSpeed() { return target_speed; }
short SerialInstructions::getTargetDirection() { return target_direction; }

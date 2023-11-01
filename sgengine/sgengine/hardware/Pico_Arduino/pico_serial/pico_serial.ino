#include "piReader.h"

SerialInstructions instructions;

void setup() {
  
  Serial1.begin(9600);
  Serial.begin(9600);

}

void loop() {
  
  Serial1.print("hello");
  Serial.print("hello");

  delay(1000);

}


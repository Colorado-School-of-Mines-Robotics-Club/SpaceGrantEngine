#include "piReader.h"

SerialInstructions instructions;

void setup() {
  
  Serial1.begin(9600);
  Serial.begin(9600);

}

void loop() {
  
  Serial.println(instructions.getTargetSpeed());
  Serial.println(instructions.getTargetDirection());

  delay(2000);

}


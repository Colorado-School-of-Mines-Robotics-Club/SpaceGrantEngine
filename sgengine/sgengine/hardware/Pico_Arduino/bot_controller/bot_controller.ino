#include "bot.h"
#include "piReader.h"
#include "math.h"

Bot bot;
SerialInstructions directions;

void setup() {
  
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);

}

void loop() {
  
  // update directions
  int target_speed = directions.getTargetSpeed();
  int direction = directions.getTargetDirection();

  // doesn't work for all speeds and directions yet
  int left_speed = target_speed - ((90 * direction) / 255);
  int right_speed = target_speed + ((90 * direction) / 255);

  bot.drive(left_speed, right_speed);
  delay(100);
  
}

#include "bot.h"

Bot bot;

int left_speed = 0;
int right_speed = 0;

void setup() {
  
  Serial.begin(9600);
  Serial1.begin(9600);

  delay(1000);

}

void loop() {
  
  // update instructions
  if (Serial1.available()) {
    
    // read the string
    arduino::String left_speed_str = Serial1.readStringUntil(',');
    arduino::String right_speed_str = Serial1.readStringUntil('\0');
    // parse the string
    if (left_speed_str[0] != '\0' && right_speed_str[0] != '\0') {
      left_speed = left_speed_str.toInt();
      right_speed = right_speed_str.toInt();

      // debug
      Serial.println("left: " + (String) left_speed + " | right: " + (String) right_speed);
    }
  }

  // drive
  bot.drive(left_speed, right_speed);

  // test_drive();
  
}


void test_drive() {
  bot.drive(100);
  delay(3000);
  bot.drive(50);
  delay(3000);

  bot.drive(0,255);
  delay(3000);
  bot.drive(255,0);
  delay(3000);
}

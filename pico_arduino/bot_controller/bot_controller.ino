#include "bot.h"

Bot bot;

int left_speed = 0;
int right_speed = 0;

void setup() {
  
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);
}

void loop() {
  // update instructions
  if (Serial.available()) {
    // read the string
    arduino::String left_speed_str = Serial.readStringUntil(',');
    arduino::String right_speed_str = Serial.readStringUntil('\0');
    // parse the string
    if (left_speed_str.length() > 0 && right_speed_str.length() > 0 && left_speed_str.length() <= 3 && right_speed_str.length() <= 3) {
      left_speed = left_speed_str.toInt();
      right_speed = right_speed_str.toInt();

      Serial.println("left: " + (String) left_speed + " | right: " + (String) right_speed);

      if (left_speed != 0 || right_speed != 0)
        digitalWrite(LED_BUILTIN, HIGH);
      else
        digitalWrite(LED_BUILTIN, LOW);
    }
  }

  bot.drive(left_speed, right_speed);
}

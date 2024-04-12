#include "bot.h"

Bot bot;

int left_speed = 0;
int right_speed = 0;

unsigned long previous_input_time = 0;

#define ENABLE_PIN 1

void setup() {

  Serial.begin(9600, SERIAL_8O1);

  pinMode(ENABLE_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);
}

void loop() {
  // update instructions
  bool enabled = digitalRead(ENABLE_PIN) == HIGH;
  if (enabled && Serial.available()) {
    // char input;
    // Serial.readBytes(&input, 1);
    // read the string

    arduino::String left_speed_str = Serial.readStringUntil(',');
    arduino::String right_speed_str = Serial.readStringUntil('/');

    if (left_speed_str.length() > 0 && right_speed_str.length() > 0 && left_speed_str.length() <= 3 && right_speed_str.length() <= 3) {
      int left_speed = constrain(left_speed_str.toInt(), -254, 254);
      int right_speed = constrain(right_speed_str.toInt(), -254, 254);
      bot.drive(left_speed, right_speed);

      previous_input_time = millis();

      Serial.println("left: " + (String)left_speed + " | right: " + (String)right_speed);

      if (left_speed != 0 || right_speed != 0)
        digitalWrite(LED_BUILTIN, HIGH);
      else
        digitalWrite(LED_BUILTIN, LOW);
    }
  }

  if (!enabled || millis() - previous_input_time > 1000) {
    bot.drive(0, 0);
  }
}

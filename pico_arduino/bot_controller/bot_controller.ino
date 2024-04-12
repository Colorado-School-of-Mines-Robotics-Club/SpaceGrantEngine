#include "bot.h"

Bot bot;

int left_speed = 0;
int right_speed = 0;

#define ENABLE_PIN 0

unsigned long previous_timer = 0;

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENABLE_PIN, INPUT);

  delay(1000);
}

void loop() {
  bool enabled = digitalRead(ENABLE_PIN) == LOW;
  if (!enabled || millis() - previous_timer > 1000) {
    left_speed = 0;
    right_speed = 0;
    while (Serial.available() > 0) {
        Serial.read(); // Read and discard the incoming byte
    }
  }
  // update instructions
  if (enabled && Serial.available()) {
    // read the string
    arduino::String left_speed_str = Serial.readStringUntil(',');
    arduino::String right_speed_str = Serial.readStringUntil('\0');
    unsigned long currentMillis = millis();
    // parse the string
    if (left_speed_str.length() > 0 && right_speed_str.length() > 0 && left_speed_str.length() <= 3 && right_speed_str.length() <= 3) {
      left_speed = left_speed_str.toInt();
      right_speed = right_speed_str.toInt();

      Serial.println("left: " + (String)left_speed + " | right: " + (String)right_speed);

      if (left_speed != 0 || right_speed != 0)
        digitalWrite(LED_BUILTIN, HIGH);
      else
        digitalWrite(LED_BUILTIN, LOW);
    }
  }

  bot.drive(left_speed, right_speed);
}

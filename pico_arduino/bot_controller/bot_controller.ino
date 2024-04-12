#include "bot.h"

Bot bot;

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
  if (digitalRead(ENABLE_PIN) == HIGH && Serial.available()) {
    // char input;
    // Serial.readBytes(&input, 1);
    // read the string

    previous_input_time = millis();

    int left_speed = constrain(Serial.parseInt(), -254, 254);
    int right_speed = constrain(Serial.parseInt(), -254, 254);


    bot.drive(left_speed, right_speed);

    Serial.println("left: " + (String) left_speed + " | right: " + (String) right_speed);

    if (left_speed != 0 || right_speed != 0)
      digitalWrite(LED_BUILTIN, HIGH);
    else
      digitalWrite(LED_BUILTIN, LOW);
  }

  if (millis() - previous_input_time > 1000) {
    bot.drive(0, 0);
  }
}

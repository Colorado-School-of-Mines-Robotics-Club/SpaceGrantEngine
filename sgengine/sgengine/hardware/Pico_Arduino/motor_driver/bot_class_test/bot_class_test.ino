#include "Bot.h"

Bot bot;

void setup() {

}

void loop() {
  
  // run backward
  bot.drive(-255);
  delay(2000);

  // run forward
  bot.drive(255);
  delay(2000);

  // hard left
  bot.adjust_direction(200);
  delay(2000);

  // stop
  bot.stop();
  delay(5000);

}

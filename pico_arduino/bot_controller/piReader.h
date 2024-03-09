#ifndef PIREADER_H
#define PIREADER_H

#include "Serial.h"

#define INTERRUPT_PIN 2

// Constantly reads speed and direction instructions from the Pi and updates target values.
class SerialInstructions {
public:

  // default constructor uses Serial1 with pins GP0 and GP1
  SerialInstructions();

  // get current target speed and direction values
  short getTargetSpeed();
  short getTargetDirection();

private:

  static inline short target_speed;
  static inline short target_direction;
  
  typedef enum {
    DEFAULT,
    CUSTOM
  } serial_mode_t;

  serial_mode_t serial_mode;

  // ISR to read and parse data from buffer
  static void readISR();

};

#endif

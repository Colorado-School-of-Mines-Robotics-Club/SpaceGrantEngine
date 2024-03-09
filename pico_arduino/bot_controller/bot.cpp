#include "Bot.h"
#include "piReader.h"

Bot::Bot() {
  // make sure motors aren't spinning
  br_motor.stop();
  fr_motor.stop();
  bl_motor.stop();
  fl_motor.stop();
  // initialize default motor speed
  br_motor.setSpeed(255);
  fr_motor.setSpeed(255);
  bl_motor.setSpeed(255);
  fl_motor.setSpeed(255);

  left_speed = 0;
  right_speed = 0;

  // start communications with the RPi
  SerialInstructions piInstructions;
}

// drive the left and right motors at different speeds.
// negative speeds will correspond to driving backwards.
// only speeds between 0 - 255 are accepted
void Bot::drive(int left_speed, int right_speed) {

  L298N::Direction l_direction = L298N::FORWARD;
  L298N::Direction r_direction = L298N::FORWARD;

  // set motor directions
  if (left_speed < 0) {
    left_speed = 0 - left_speed;
    l_direction = L298N::BACKWARD;
  }
  if (right_speed < 0) {
    right_speed = 0 - right_speed;
    r_direction = L298N::BACKWARD;
  }

  // set speeds
  fl_motor.setSpeed(left_speed);
  bl_motor.setSpeed(left_speed);
  fr_motor.setSpeed(right_speed);
  br_motor.setSpeed(right_speed);

  this->left_speed = left_speed;
  this->right_speed = right_speed;

  // drive
  br_motor.run(r_direction);
  fr_motor.run(r_direction);
  bl_motor.run(l_direction);
  fl_motor.run(l_direction);

}

// drive both sides at the same speed
// negative speeds will correspond to driving backwards.
// only speeds between 0 - 255 are accepted.
void Bot::drive(int speed) {
  drive(speed, speed);
}


// stop the motors from running
void Bot::stop() {
  br_motor.stop();
  fr_motor.stop();
  bl_motor.stop();
  fl_motor.stop();

  left_speed = 0;
  right_speed = 0;
}


// Change the driving direction by a pwm count.
// With the current implementation, the count corresponds to how much
//   the speed will increase/decrease on the respective sides.
// positive corresponds to a clockwise angle change
void Bot::adjust_direction(int pwm_count) {

  // adjust speed
  left_speed += pwm_count;
  right_speed += pwm_count;

  // check if values are over the limit
  if (left_speed > 255) left_speed = 255;
  else if (left_speed < -255) left_speed = -255;
  if (right_speed > 255) right_speed = 255;
  else if (right_speed < -255) right_speed = -255;

  // set speeds
  drive(left_speed, right_speed);
}


// different function calls to set the speed while driving
void Bot::set_speed(int left_speed, int right_speed) {
  drive(left_speed, right_speed);
}

void Bot::set_speed(int speed) {
  drive(speed);
}
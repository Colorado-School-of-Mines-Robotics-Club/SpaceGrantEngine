#ifndef BOT_H
#define BOT_H

#include <L298N.h>

// Controls the robot for the F23 Mines Space Grant Robotics Team
// NOTE: You must include Serial1.begin(baud) in your setup for the communications to work.
class Bot {
public:

  // four motors
  // EN, IN1, IN2
  L298N br_motor = L298N(4, 3, 2);
  L298N fr_motor = L298N(8, 7, 6);
  L298N fl_motor = L298N(11, 13, 12);
  L298N bl_motor = L298N(21, 20, 19);


  Bot();

  // drive the left and right motors at different speeds.
  // negative speeds will correspond to driving backwards.
  // only speeds between 0 - 255 are accepted
  void drive(int left_speed, int right_speed);

  // drive both sides at the same speed
  // negative speeds will correspond to driving backwards.
  // only speeds between 0 - 255 are accepted.
  void drive(int speed);

  // stop the motors from running
  void stop();

  // Change the driving direction by a pwm count.
  // With the current implementation, the count corresponds to how much
  //   the speed will increase/decrease on the respective sides.
  // positive corresponds to a clockwise angle change
  void adjust_direction(int pwm_count);

  // different function calls to set the speed while driving
  void set_speed(int left_speed, int right_speed);
  void set_speed(int speed);

private:

  int left_speed;
  int right_speed;

};

#endif

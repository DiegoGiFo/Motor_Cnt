#ifndef __ROBOT__HPP__
#define __ROBOT__HPP__

#include <StepperDriver.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

class Robot {
  public:
    Robot(int step1, int dir1, int step2, int dir2);

    void set_motors(const geometry_msgs::Twist &movements);
    void run_mt();

  private:
    Motor right_mt;
    Motor left_mt;
    float w_right;
    float w_left;
};

#endif

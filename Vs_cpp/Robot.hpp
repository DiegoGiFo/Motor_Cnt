#ifndef __ROBOT__HPP__
#define __ROBOT__HPP__

#include <StepperDriver.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

class Robot {
  public:
    Robot();

    void set_motors(const geometry_msgs::Twist &movements);
    void run_mt();

  private:
    //Motor right_mt(int p, int d);
    //Motor right_mt(int p, int d);
    float w_right;
    float w_left;
};

#endif

#include <StepperDriver.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Motor.hpp"
#include "Robot.hpp"

#define L 0.20f // distance between the two wheels of the robot
#define R 0.05f //radius of the wheel od the robot

const float A = L/(2.0f*R);
const float B = 1.0f/R;

Motor right_mt(2,5);
Motor left_mt(3,6);

Robot::Robot(){
  this->w_right = 0.0;
  this->w_left = 0.0;
}

 void Robot::set_motors(const geometry_msgs::Twist &movements) {
   this->w_right = A*movements.angular.z + B*movements.linear.x;
   this->w_left = A*movements.angular.z - B*movements.linear.x;

   right_mt.set_speed(this->w_right);
   left_mt.set_speed(this->w_left);

 }

 void Robot::run_mt(){

   left_mt.run();
   right_mt.run();

 }

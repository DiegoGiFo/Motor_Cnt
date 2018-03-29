#include <StepperDriver.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Motor.hpp"
#include "Robot.hpp"

#define L 0.20f // distance between the two wheels of the robot
#define R 0.05f //radius of the wheel od the robot

const float A = L/(2.0f*R);
const float B = 1.0f/R;



Robot::Robot(int step1, int dir1, int step2, int dir2){
  this->right_mt(step1,dir1);
  this->left_mt(step2,dir2);
  this->w_right = 0.0;
  this->w_left = 0.0;
}

 void Robot::set_motors(const geometry_msgs::Twist &movements) {
   this->w_right = A*movements.angular.z + B*movements.linear.x;
   this->w_left = A*movements.angular.z - B*movements.linear.x;

   this->right_mt.set_speed(this->w_right);
   this->left_mt.set_speed(this->w_left);

 }

 void Robot::run_mt(){

   this->left_mt.run();
   this->right_mt.run();

 }

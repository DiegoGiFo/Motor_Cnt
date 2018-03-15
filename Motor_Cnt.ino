
#include <ros.h>
#include <StepperDriver.h>


#define EN 8
#define L 10 // distance between the two wheels of the robot
#define r 5 //radius of the wheel od the robot

axis_t right, left;

void rotation(int wr, int ws, int wz, int vx){
  wr=wz*(L/2*r)-(vx/r);
  wl=wz*(L/2*r)+(vx/r);
}

void setup ()
{
  //need to set the enable to LOW because if it is HIGH the motors are desabled
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  StepperDriver.init ();

  left = StepperDriver.newAxis (2, 5, 255, 200);
  right = StepperDriver.newAxis (3, 6, 255, 200);

  StepperDriver.enable(left);
  StepperDriver.enable(right);
}

void loop(){

}

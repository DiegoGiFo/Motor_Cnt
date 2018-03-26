# STEP MOTORS CONTROLLER

This is the implemented version of the [Vs_1](https://github.com/DiegoGiFo/Motor_Cnt/tree/master/Vs_1).
The implementation includes:
- the creation of the class Motor;
- the add of an equation that breaks not istantaneously the motors.

##CODE

### MAIN
~~~cpp
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include "Motor.hpp"

ros::NodeHandle  nh; // allows to create publisher/subscriber
geometry_msgs::Twist vel;
std_msgs::Float32 msg;

Motor right_mt(2, 5);
Motor left_mt(3, 6);

#define EN 8
#define L 0.20f // distance between the two wheels of the robot
#define R 0.05f //radius of the wheel od the robot

const float A = L/(2.0f*R);
const float B = 1.0f/R;

int cnt;

void motors_cb(const geometry_msgs::Twist &move)
{
  const float w_right = A*move.angular.z + B*move.linear.x;
  const float w_left = A*move.angular.z - B*move.linear.x;

  right_mt.set_speed(w_right);
  left_mt.set_speed(w_left);

  cnt ++;

  vel.linear.x = move.linear.x;
  vel.angular.z = move.angular.z;

  msg.data = w_right;

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);
ros::Publisher pub("/info_vel", &vel);
ros::Publisher pubtest("/test_speed", &msg);

void setup ()
{
  //need to set the enable to LOW because if it is HIGH the motors are desabled
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  StepperDriver.init();

  nh.initNode(); // initialize ROS nodes
  nh.advertise(pubtest);
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{
  if((cnt%10) == 1)
  {
    pub.publish(&vel);
    pubtest.publish(&msg);
  }

  left_mt.run();
  right_mt.run();

  nh.spinOnce();
}
~~~

### FILE Motor.cpp
~~~cpp
#include "Motor.hpp"
#include <StepperDriver.h>

const float LAMBDA = 0.999f;

int get_direction(float x)
{
    if(x > 0) return FORWARD;
    else return BACKWARD;
}

Motor::Motor(int p1, int p2) {
    this->target_w = 0.0;
    this->current_w = 0.0;
    this->motor = StepperDriver.newAxis(p1, p2, 255, 200);
    StepperDriver.enable(this->motor);
}

float Motor::get_speed() {
  return this->current_w;
}

void Motor::set_speed(float speed) {
  this->target_w = speed;
}

void Motor::run() {
  this->current_w = LAMBDA*this->current_w + (1-LAMBDA)*this->target_w;
  const float w_abs = fabs(this->current_w)*(100.0/M_PI);
  const float direction = get_direction(this->current_w);
  StepperDriver.setDir (this->motor, direction);
  StepperDriver.setSpeed (this->motor, w_abs);
}
~~~

### FILE Motor.hpp
~~~cpp
#ifndef __MOTOR__HPP__
#define __MOTOR__HPP__

#include <StepperDriver.h>

class Motor {
  public:
    Motor(int p1, int p2);

    float get_speed();
    void set_speed(float speed);
    void run();

  private:
    float current_w;
    float target_w;

    axis_t motor;
};

#endif
~~~

Now analyze the program part by part.

## MAIN
### LIBRARIES

~~~cpp
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include "Motor.hpp"
~~~
Include "Motor.hpp" bacause is the header file that allows to use the class Motor.

### DECLARATION

~~~cpp
Motor right_mt(2, 5);
Motor left_mt(3, 6);
~~~
Declare right_mt and left_mt as elements belonging to the class Motor.

~~~cpp
#define EN 8
#define L 0.20f // distance between the two wheels of the robot
#define R 0.05f //radius of the wheel od the robot

const float A = L/(2.0f*R);
const float B = 1.0f/R;
~~~
Define the two constants A and B for increase the velocity of computation of the angular velocity.
Define cnt that is used to reduce the number of publications on the info_vel topic.

### CALLBACK FUNCTION

~~~cpp
void motors_cb(const geometry_msgs::Twist &move)
{
  const float w_right = A*move.angular.z + B*move.linear.x;
  const float w_left = A*move.angular.z - B*move.linear.x;

  right_mt.set_speed(w_right);
  left_mt.set_speed(w_left);

  cnt ++;

  vel.linear.x = move.linear.x;
  vel.angular.z = move.angular.z;

  msg.data = w_right;

}
~~~
The funtion receives from the topic cmd_vel the linear and angular velocity, calculates the velocity of the motors and calls the funciotn set_speed that is contained in the file Motor.cpp.


### VOID LOOP
~~~cpp
void loop()
{
  if((cnt%10) == 1)
  {
    pub.publish(&vel);
    pubtest.publish(&msg);
  }

  left_mt.run();
  right_mt.run();

  nh.spinOnce();
}
~~~

The variable cnt and the check cnt%10 allows to reduce the number of publications on the info_vel topic because the informations about velocities are published only the the counter is a multiple of 10.

right_mt.run() and left_mt.run are two functions contained in the file Motor.cpp that set the direction and the velocity of the two motors.


## FILE Motors.hpp
### INITIALILZATION

~~~cpp
#ifndef __MOTOR__HPP__
#define __MOTOR__HPP__

// operations

#endif
~~~
This part is need to initialize the header.

### CLASS

~~~cpp
class Motor {
  public:
    Motor(int p1, int p2);

    float get_speed();
    void set_speed(float speed);
    void run();

  private:
    float current_w;
    float target_w;

    axis_t motor;
};
~~~
In this section is defined the class Motor.
There are only the prototypes of the funtions that are defined in the Motor.cpp file.
The command public: defines that the following functions are available also for external functions.
The command private: defines that the following functions/variabels are visible only for the class, are not visible by external functions.


## FILE Motors.cpp
### INITIALILZATION

~~~cpp
#include "Motor.hpp"
~~~
Need to inclulde the header file Motors.hpp.

### DECLARATION

~~~cpp
const float LAMBDA = 0.999f;
~~~
Declare the constant LAMBDA that is used in an equation that allows to break the motors not istantaneously.

###FUNCTIONS

~~~cpp
int get_direction(float x)
{
    if(x > 0) return FORWARD;
    else return BACKWARD;
}
~~~
If the value of x is positive returns FORWARD that is the positive direction of rotation of the motor,
if  it is negative returns BACKWARD, the negative one.

~~~cpp
Motor::Motor(int p1, int p2) {
    this->target_w = 0.0;
    this->current_w = 0.0;
    this->motor = StepperDriver.newAxis(p1, p2, 255, 200);
    StepperDriver.enable(this->motor);
}
~~~
Defines the class Motor.
Receives two value, the pulse and the direction pin of the motor.
The variables target_w and current_w are used for the control of the breaking function.

~~~cpp
void Motor::run() {
  this->current_w = LAMBDA*this->current_w + (1-LAMBDA)*this->target_w;
  const float w_abs = fabs(this->current_w)*(100.0/M_PI);
  const float direction = get_direction(this->current_w);
  StepperDriver.setDir (this->motor, direction);
  StepperDriver.setSpeed (this->motor, w_abs);
}
~~~
With this equation: ![eq](https://github.com/DiegoGiFo/Motor_Cnt/blob/master/Vs_2/CodeCogsEqn.gif?raw=true "Figure 1")
is obtained a **low pas filter** that has a decreasing exponential behaviour and not a step one then we have a gradual acceleration/slowdown.
Is computed the absolute value of the current velocity and converted in rad/s.
The direction is calculated by the function get_direction and then are set the direction and the speed of the motor.

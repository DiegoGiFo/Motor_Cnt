# STEP MOTORS CONTROLLER

This is the implemented version of the [Vs_1](https://github.com/DiegoGiFo/Motor_Cnt/tree/master/Vs_1).

##CODE

~~~cpp
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh; // allows to create publisher/subscriber
geometry_msgs::Twist vel;

#define EN 8
#define L 0.100 // distance between the two wheels of the robot
#define R 0.05 //radius of the wheel od the robot

const float A = L/(2*R);
const float B = 1/R;

axis_t right_mt, left_mt;
int cnt;

int sign(float x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}

void set_motors( axis_t motor, float w )
{
  float w_abs;
  int w_s;

  w_abs = abs(w);
  w_s = sign(w);

  if(w_s > 0){
    StepperDriver.setDir (motor, FORWARD);
    StepperDriver.setSpeed (motor, w_abs);
  }else{
    StepperDriver.setDir (motor, BACKWARD);
    StepperDriver.setSpeed (motor, w_abs);
  }
}


void motors_cb(const geometry_msgs::Twist &move)
{
  float wr,wl;

  wr = A*move.angular.z + B*move.linear.x;
  wl = A*move.angular.z - B*move.linear.x;

  set_motors(right_mt, wr);
  set_motors(left_mt, wl);

  cnt ++;

  vel.linear.x = move.linear.x;
  vel.angular.z = move.angular.z;

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);
ros::Publisher pub("/info_vel", &vel);

void setup ()
{
  //need to set the enable to LOW because if it is HIGH the motors are desabled
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  StepperDriver.init();

  right_mt = StepperDriver.newAxis(2, 5, 255, 200);
  left_mt = StepperDriver.newAxis(3, 6, 255, 200);

  StepperDriver.enable(right_mt);
  StepperDriver.enable(left_mt);

  nh.initNode(); // initialize ROS node
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{
  if((cnt%10) == 1)
  {
    pub.publish(&vel);
  }

  nh.spinOnce();
}
~~~

## MODIFICATIONS

The main changes are done in this sections:

### DECLARATION
~~~cpp
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh; // allows to create publisher/subscriber
geometry_msgs::Twist vel;

#define EN 8
#define L 0.100 // distance between the two wheels of the robot
#define R 0.05 //radius of the wheel od the robot

const float A = L/(2*R);
const float B = 1/R;

axis_t right_mt, left_mt;
int cnt;
~~~

I defined the two constants A and B for increase the velocity of computation of the angular velocity.
I defined cnt that is used to reduce the number of publications on the info_vel topic.

### CALLBACK FUNCTION
~~~cpp
void motors_cb(const geometry_msgs::Twist &move)
{
  float wr,wl;

  wr = A*move.angular.z + B*move.linear.x;
  wl = A*move.angular.z - B*move.linear.x;

  set_motors(right_mt, wr);
  set_motors(left_mt, wl);

  cnt ++;

  vel.linear.x = move.linear.x;
  vel.angular.z = move.angular.z;

}
~~~

The funtion receives from the topic cmd_vel the linear and angular velocity, calculates the velocity of the motors and calls the funciotn set_motors.

### SET_MOTORS
~~~cpp
void set_motors( axis_t motor, float w )
{
  float w_abs;
  int w_s;

  w_abs = abs(w);
  w_s = sign(w);

  if(w_s > 0){
    StepperDriver.setDir (motor, FORWARD);
    StepperDriver.setSpeed (motor, w_abs);
  }else{
    StepperDriver.setDir (motor, BACKWARD);
    StepperDriver.setSpeed (motor, w_abs);
  }
}
~~~

Sign function:
~~~cpp
int sign(float x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}
~~~

This function receives the name of the motor on which needs to operate and its velocity.
The function calculates the absolute value and the sign of the velocity. The sign is calculated by the function sign.
After this calculations the function sets the direction and the spedd of the motor with respectly StepperDriver.setDir() and StepperDriver.setSpeed().
If the sign is positive set the direction FORWARD otherwise BACKWARD.

### VOID LOOP
~~~cpp
void loop()
{
  if((cnt%10) == 1)
  {
    pub.publish(&vel);
  }

  nh.spinOnce();
}
~~~

The variable cnt and the check cnt%10 allows to reduce the number of publications on the info_vel topic because the informations about velocities are published only the the counter is a multiple of 10.
Checking with the command: rostopic hz info_vel ,can be obserbed that the avg rate decreases from 112 to 40. 

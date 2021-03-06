# STEP MOTORS CONTROLLER
### This project allow to control two step motors receiving as input the values of the topic cmd_vel that controls the turtlebot3 simulator.

For the realization of the project I've done this passages:

- Found this library :[CNC library](https://github.com/DIMRobotics/ArduinoStepperDriver/wiki/Reference)
and i understood how to use it.

- Did some sketches to test the code.
Can find them here: [Step Motors test](https://github.com/DiegoGiFo/Step_Motor)

- Understood the cinematics equations for the motion of the motors. The equation that I used is:

![eq](https://github.com/DiegoGiFo/Motor_Cnt/blob/master/Vs_1/render.png?raw=true "Figure")

- Write the final code that allows to control the two step motors receiving as inputs the linear velocity and he angular one of the topic cmd_vel.

The code full code is :

~~~cpp
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>

ros::NodeHandle  nh; // allows to create publisher/subscriber
geometry_msgs::Twist vel;
//nav_msgs::Odometry mov;

#define EN 8
#define L 0.100 // distance between the two wheels of the robot
#define r 0.05 //radius of the wheel od the robot

axis_t right, left;

void motors_cb(const geometry_msgs::Twist &m_r){
  int wr,wl;
  if (m_r.linear.x > 0 && m_r.angular.z == 0){ //va avanti
    wr=(m_r.linear.x/r);
    wl=(m_r.linear.x/r);
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x < 0 && m_r.angular.z == 0){
    wr=(abs(m_r.linear.x)/r);
    wl=(abs(m_r.linear.x)/r);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, BACKWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x == 0 && m_r.angular.z <= 0){
    wr=(1/(2*r))*(abs(m_r.angular.z)*L-m_r.linear.x*2);
    wl=(1/(2*r))*(abs(m_r.angular.z)*L+m_r.linear.x*2);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x == 0 && m_r.angular.z >= 0){
    wr=(1/(2*r))*(m_r.angular.z*L+m_r.linear.x*2);
    wl=(1/(2*r))*(m_r.angular.z*L-m_r.linear.x*2);
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, BACKWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x > 0 && m_r.angular.z <= 0){
    wr=(1/(2*r))*abs((abs(m_r.angular.z)*L+m_r.linear.x*2));
    wl=(1/(2*r))*abs((abs(m_r.angular.z)*L-m_r.linear.x*2));
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x > 0 && m_r.angular.z >= 0){
    wr=(1/(2*r))*abs((m_r.angular.z*L-m_r.linear.x*2));
    wl=(1/(2*r))*abs((m_r.angular.z*L+m_r.linear.x*2));
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  vel.linear.x=wr;
  vel.linear.y=wl;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);
ros::Publisher pub("/info_vel", &vel);
//ros::Publisher pub("/odom", &mov);

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

  nh.initNode(); // initialize ROS node
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop(){
  pub.publish(&vel);
  nh.spinOnce();
}
~~~

### Now lets analize the code:

## LIBRARIES

~~~cpp
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>
~~~

In this part is included ros_libs that allows to use all the others library: geometry_msgs and nav_msgs.
Is it also inlcuded StedderDriver that allows to control the stepper in syncronous way with the cnc shield.
You can find the StepperDriver lirary here: [library folder](https://github.com/DiegoGiFo/Motor_Cnt/tree/master/StepperD_lib/StepperDriver).

## DECLARATION

~~~cpp
ros::NodeHandle  nh; // allows to create publisher/subscriber
geometry_msgs::Twist vel;
//nav_msgs::Odometry mov;
~~~
Create a ROS node and declair a variable of type geometry_msgs/Twist.

~~~cpp
#define EN 8
#define L 0.100 // distance between the two wheels of the robot
#define r 0.05 //radius of the wheel od the robot

axis_t right, left;
~~~
Define EN that is the enable pin of the stepper.
Define L that is the length between the 2 wheels (cm).
Define r that is the radius of the wheels (cm).
Declare the two variables rigth and left of type axis_t.

## CALLBACK FUNCTION

~~~cpp
void motors_cb(const geometry_msgs::Twist &m_r){
  int wr,wl;
  if (m_r.linear.x > 0 && m_r.angular.z == 0){ //va avanti
    wr=(m_r.linear.x/r);
    wl=(m_r.linear.x/r);
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x < 0 && m_r.angular.z == 0){ // va indietro
    wr=(abs(m_r.linear.x)/r);
    wl=(abs(m_r.linear.x)/r);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, BACKWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x == 0 && m_r.angular.z <= 0){
    wr=(1/(2*r))*(abs(m_r.angular.z)*L-m_r.linear.x*2);
    wl=(1/(2*r))*(abs(m_r.angular.z)*L+m_r.linear.x*2);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x == 0 && m_r.angular.z >= 0){
    wr=(1/(2*r))*(m_r.angular.z*L+m_r.linear.x*2);
    wl=(1/(2*r))*(m_r.angular.z*L-m_r.linear.x*2);
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, BACKWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x > 0 && m_r.angular.z <= 0){
    wr=(1/(2*r))*abs((abs(m_r.angular.z)*L+m_r.linear.x*2));
    wl=(1/(2*r))*abs((abs(m_r.angular.z)*L-m_r.linear.x*2));
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x > 0 && m_r.angular.z >= 0){
    wr=(1/(2*r))*abs((m_r.angular.z*L-m_r.linear.x*2));
    wl=(1/(2*r))*abs((m_r.angular.z*L+m_r.linear.x*2));
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  vel.linear.x=wr;
  vel.linear.y=wl;
}
~~~
In the callback function all computations of the velocity of rotation of the wheels are made.
declair at first wr and wl that are respectly the angular velocity of right and left wheel.
After calculate the two values (wr,wf) need to set the direction of rotation of the motors(forward or backward) with the command StepperDriver.setDir().
With the command StepperDriver.setSpeed() is setted the velocity of the motor.

At the end of the 6 cases of motors's motion at the variable vel are assigned the values of the 2 velocities, one angular and the other one linear.

## VOID SETUP

~~~cpp
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

  nh.initNode(); // initialize ROS node
  nh.subscribe(sub);
  nh.advertise(pub);
}
~~~
In void setup sets the enable pin to LOW because if it is HIGH the motors are desabled,
initializes the StepperDriver, sets the left and rigth motor and enable them.
Initializes the ROS node and advertise the system that there is a publisher and a subscriber.

## VOID LOOP

~~~cpp
void loop(){
  pub.publish(&vel);
  nh.spinOnce();
}
~~~
In void loop publish the two velocities on the topic info_vel.

# STEP MOTORS CONTROLLER
### This project allow to control two step motors receiving as input the values of the topic cmd_vel that controls the turtlebot3 simulator.

For the realization of the project I've done this passages:

- Found this library :[CNC library](https://github.com/DIMRobotics/ArduinoStepperDriver/wiki/Reference)
and i understood how to use it.

- Did some sketches to test the code.
Can find them here: [Step Motors test](https://github.com/DiegoGiFo/Step_Motor)

- Understood the cinematics equations for the motion of the motors. The equation that I used is:

![eq](http://www.sciweavers.org/upload/Tex2Img_1521467554/render.png?raw=true "Figure")

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
  else if (m_r.linear.x < 0 && m_r.angular.z == 0){ // va indietro
    wr=(abs(m_r.linear.x)/r);
    wl=(abs(m_r.linear.x)/r);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, BACKWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x == 0 && m_r.angular.z <= 0){  //gira a destra
    wr=(1/(2*r))*(abs(m_r.angular.z)*L-m_r.linear.x*2);
    wl=(1/(2*r))*(abs(m_r.angular.z)*L+m_r.linear.x*2);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x == 0 && m_r.angular.z >= 0){  //gira a sinistra
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

Now lets analize the code:

~~~cpp
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>
~~~

In this part is included ros_libs that allows to use all the others library: geometry_msgs and nav_msgs.
Is it also inlcuded StedderDriver that allows to control the stepper in syncronous way with the cnc shield.
You can find the StepperDriver lirary here: [library folder](https://github.com/DiegoGiFo/Motor_Cnt/tree/master/StepperD_lib/StepperDriver).

~~~cpp

~~~

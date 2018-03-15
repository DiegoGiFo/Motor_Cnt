
#include <ros.h>
#include <StepperDriver.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define EN 8
#define L 10 // distance between the two wheels of the robot
#define r 5 //radius of the wheel od the robot

axis_t right, left;

void motors_cb(const geometry_msgs::Twist &m_r){

  if (m_r.linear.x >= 0 && m_r.angular.z = 0){
    wr=(m_r.linear.x/r);
    wl=(m_r.linear.x/r);
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x <= 0 && m_r.angular.z = 0){
    wr=(abs(m_r.linear.x)/r);
    wl=(abs(m_r.linear.x)/r);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, BACKWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x >= 0 && m_r.angular.z >= 0){
    wr=(1/(2*r))(m_r.angular.z*L-m_r.linear.x);
    wl=(1/(2*r))(m_r.angular.z*L+m_r.linear.x);
    StepperDriver.setDir (left, FORWARD);
    StepperDriver.setDir (right, BACKWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x >= 0 && m_r.angular.z <= 0){
    wr=(1/(2*r))(abs(m_r.angular.z)*L+m_r.linear.x);
    wl=(1/(2*r))(abs(m_r.angular.z)*L-m_r.linear.x);
    StepperDriver.setDir (left, BACKWARD);
    StepperDriver.setDir (right, FORWARD);
    StepperDriver.setSpeed (left, wr);
    StepperDriver.setSpeed (right, wl);
  }
  else if (m_r.linear.x <= 0 && m_r.angular.z >= 0){
      wr=(1/(2*r))(abs(m_r.angular.z)*L+abs(m_r.linear.x));
      wl=(1/(2*r))(abs(m_r.angular.z)*L-abs(m_r.linear.x));
      StepperDriver.setDir (left, FORWARD);
      StepperDriver.setDir (right, BACKWARD);
      StepperDriver.setSpeed (left, wr);
      StepperDriver.setSpeed (right, wl);
    }
    else if (m_r.linear.x <= 0 && m_r.angular.z <= 0){
        wr=(1/(2*r))(abs(m_r.angular.z)*L-abs(m_r.linear.x));
        wl=(1/(2*r))(abs(m_r.angular.z)*L+abs(m_r.linear.x));
        StepperDriver.setDir (left, BACKWARD);
        StepperDriver.setDir (right, FORWARD);
        StepperDriver.setSpeed (left, wr);
        StepperDriver.setSpeed (right, wl);
      }
}

ros::NodeHandle  nh; // allows to create publisher/subscriber
nav_msgs::Odometry mov;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);
ros::Publisher pub("/odom", &mov);

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
  nh.spinOnce();
}

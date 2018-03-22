#include <ros.h>
#include <StepperDriver.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh; // allows to create publisher/subscriber
nav_msgs::Odometry vel;

#define EN 8
#define L 0.100 // distance between the two wheels of the robot
#define R 0.05 //radius of the wheel od the robot

const float A = L/(2*R);
const float B = 1/R;

axis_t right_mt, left_mt;

int sign(float x)
{
    if(x > 0) return 1;
    if(x < 0) return -1;
    return 0;
}

void set_motors( axis_t motor,float w_abs,int w_s )
{
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
  float wr,wl,wr_abs,wl_abs;
  int wr_sign,wl_sign;

  wr = A*move.angular.z + B*move.linear.x;
  wl = A*move.angular.z - B*move.linear.x;
  wr_abs = abs(wr);
  wl_abs = abs(wl);
  wr_sign = sign(wr);
  wl_sign = sign(wl);

  set_motors(right_mt, wr_abs, wr_sign);
  set_motors(left_mt, wl_abs, wl_sign);

  // pubblica su nav_msgs/Odometry
  vel.twist.twist.linear.x = wr;
  vel.twist.twist.linear.y = wl;
  vel.twist.twist.angular.z = move.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);
ros::Publisher pub("/odom_info", &vel);

void setup ()
{
  //need to set the enable to LOW because if it is HIGH the motors are desabled
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  StepperDriver.init ();

  right_mt = StepperDriver.newAxis (2, 5, 255, 200);
  left_mt = StepperDriver.newAxis (3, 6, 255, 200);

  StepperDriver.enable(right_mt);
  StepperDriver.enable(left_mt);

  nh.initNode(); // initialize ROS node
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{
  pub.publish(&vel);
  nh.spinOnce();
}

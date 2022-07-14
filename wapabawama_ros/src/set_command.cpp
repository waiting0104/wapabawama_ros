/*
 * Interface of ROS <> STM32 UART
 * Design to 3 linear actuator gantry
 *
 * Subscribe each topic and publish to stm32 in loop
 *
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */

/* #define SIMULATION */
#define REVERSE

/* #define GANTRY_RATIO 33.5233 // Cartision to Real 0.1(m)/(rad), Wheel R = 9.5cm = 0.095m, st ratio = 0.1 */
/* #define GANTRY_RATIO 42.5 // Cartision to Real 0.1(m)/(rad), Wheel R = 9.5cm = 0.095m, st ratio = 0.1 */
#define GANTRY_RATIO 21.25 // Cartision to Real 0.1(m)/(rad), Wheel R = 9.5cm = 0.095m, st ratio = 0.1
#define LA_RATIO 36666.66 // Cartision to Real (m)/(tick)
#define MAX_FLOW 32.31 // ml/sec                                                                                         
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <string>
#ifndef SIMULATION
#include "system.hpp"
#endif // SIMULATION

#ifndef SIMULATION
/* Dsmotor sys; */
System sys;
std::string dev_name;
int bdrate;
#endif // SIMULATION

/* Topic name */
/* Pub to tf */
std::string pub_gantry_tpname;
std::string pub_gantry_speed_tpname;
std::string pub_la1_tpname;
std::string pub_la2_tpname;
std::string pub_la3_tpname;
/* Sub from topic and save to global var */
std::string sub_gantry_tpname;
std::string sub_la1_tpname;
std::string sub_la2_tpname;
std::string sub_la3_tpname;
/* Sub Valve state and pwm */
std::string sub_valve1_state;
std::string sub_valve2_state;
std::string sub_valve3_state;
std::string sub_valve1_pwm_name;
std::string sub_valve2_pwm_name;
std::string sub_valve3_pwm_name;
/* Position */
double la1_pos = 0;
double la2_pos = 0;
double la3_pos = 0;
double gantry_speed_cmd = 0;
double gantry_speed_fb = 0;
double gantry_pos = 0;
int valve1_pwm = 0;
int valve2_pwm = 0;
int valve3_pwm = 0;
double valve1_flow = 0;
double valve2_flow = 0;
double valve3_flow = 0;
int freq; // commend send freq
 
void set_la1_poseCallback(const std_msgs::Float32::ConstPtr& msg) {
  la1_pos = msg->data;
}
void set_la2_poseCallback(const std_msgs::Float32::ConstPtr& msg) {
  la2_pos = msg->data;
}
void set_la3_poseCallback(const std_msgs::Float32::ConstPtr& msg) {
  la3_pos = msg->data;
}

void set_gantryspeedCallback(const std_msgs::Float32::ConstPtr& msg) {
  gantry_speed_cmd = msg->data;
  // ROS_INFO("Gantry Speed set to %f", gantry_speed_cmd);
}
void set_valve1_pwmCallback(const std_msgs::Int8::ConstPtr& msg) {
  valve1_pwm = msg->data;
  // ROS_INFO("Valve1 pwm set to %d", valve1_pwm);
}
void set_valve2_pwmCallback(const std_msgs::Int8::ConstPtr& msg) {
  valve2_pwm = msg->data;
  // ROS_INFO("Valve2 pwm set to %d", valve2_pwm);
}
void set_valve3_pwmCallback(const std_msgs::Int8::ConstPtr& msg) {
  valve3_pwm = msg->data;
  // ROS_INFO("Valve3 pwm set to %d", valve3_pwm);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "set_command");
  ros::NodeHandle pn_("~");
#ifndef SIMULATION
  pn_.param<std::string>( "device_name", dev_name, "/dev/ttyTHS0");
  pn_.param<int>(         "baud_rate",   bdrate,   115200);
#endif // SIMULATION
  pn_.param<std::string>( "gantry_pub", pub_gantry_tpname, "gantry/pose");
  pn_.param<std::string>( "gantry_pub_speed", pub_gantry_speed_tpname, "gantry/speed");
  pn_.param<std::string>( "la1_pub", pub_la1_tpname, "la1/pose");
  pn_.param<std::string>( "la2_pub", pub_la2_tpname, "la2/pose");
  pn_.param<std::string>( "la3_pub", pub_la3_tpname, "la3/pose");
  pn_.param<std::string>( "gantry_sub", sub_gantry_tpname, "gantry/set_speed");
  pn_.param<std::string>( "la1_sub", sub_la1_tpname, "la1/set_pose");
  pn_.param<std::string>( "la2_sub", sub_la2_tpname, "la2/set_pose");
  pn_.param<std::string>( "la3_sub", sub_la3_tpname, "la3/set_pose");
  // pn_.param<std::string>( "sub_valve1_state" , sub_valve1_state , "valve1/state" );
  // pn_.param<std::string>( "sub_valve2_state" , sub_valve2_state , "valve2/state" );  
  // pn_.param<std::string>( "sub_valve3_state" , sub_valve3_state , "valve3/state" );
  pn_.param<std::string>( "sub_valve1_pwm" , sub_valve1_pwm_name , "valve1/pwm" );
  pn_.param<std::string>( "sub_valve2_pwm" , sub_valve2_pwm_name , "valve2/pwm" );  
  pn_.param<std::string>( "sub_valve3_pwm" , sub_valve3_pwm_name , "valve3/pwm" );
  pn_.param<int>( "freq", freq, 30);

#ifndef SIMULATION
  sys.init(dev_name,bdrate);
#endif // SIMULATION
  int a = 0 ;
  ros::NodeHandle n;
  ros::Subscriber set_gtsped_sub   = n.subscribe(sub_gantry_tpname, 10, &set_gantryspeedCallback);
  ros::Subscriber set_la1_pose_sub = n.subscribe(sub_la1_tpname, 10, &set_la1_poseCallback);
  ros::Subscriber set_la2_pose_sub = n.subscribe(sub_la2_tpname, 10, &set_la2_poseCallback);
  ros::Subscriber set_la3_pose_sub = n.subscribe(sub_la3_tpname, 10, &set_la3_poseCallback);
  ros::Subscriber set_valve1_pwm_sub = n.subscribe(sub_valve1_pwm_name, 10, &set_valve1_pwmCallback);
  ros::Subscriber set_valve2_pwm_sub = n.subscribe(sub_valve2_pwm_name, 10, &set_valve2_pwmCallback);
  ros::Subscriber set_valve3_pwm_sub = n.subscribe(sub_valve3_pwm_name, 10, &set_valve3_pwmCallback);
  ros::Publisher  gantry_pub       = n.advertise<std_msgs::Float32>(pub_gantry_tpname, 10);
  ros::Publisher  gantry_speed_pub = n.advertise<std_msgs::Float32>(pub_gantry_speed_tpname, 10);
  ros::Publisher  la1_pub          = n.advertise<std_msgs::Float32>(pub_la1_tpname, 10);
  ros::Publisher  la2_pub          = n.advertise<std_msgs::Float32>(pub_la2_tpname, 10);
  ros::Publisher  la3_pub          = n.advertise<std_msgs::Float32>(pub_la3_tpname, 10);

#ifndef SIMULATION
  sys.run();
  sys.resetOrig();
#endif // SIMULATION

  ros::Rate loop_rate(freq);

  ros::Time current_time, last_time , ini_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ini_time = ros::Time::now();
  
  while (ros::ok()) {
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    
#ifdef SIMULATION
    std_msgs::Float32 la1_pos_, la2_pos_, la3_pos_, gantry_pos_, gantry_speed_; // ros Float32 to send msg
    la1_pos_.data = la1_pos;
    la2_pos_.data = la2_pos;
    la3_pos_.data = la3_pos;
    gantry_speed_fb = gantry_speed_cmd;
    gantry_pos += gantry_speed_fb * dt;
    gantry_pos_.data = gantry_pos;
    gantry_speed_.data = gantry_speed_fb;

    gantry_pub.publish(gantry_pos_);
    gantry_speed_pub.publish(gantry_speed_);
    la1_pub.publish(la1_pos_);
    la2_pub.publish(la2_pos_);
    la3_pub.publish(la3_pos_);
#else // Not simulation
    std_msgs::Float32 la1_pos_, la2_pos_, la3_pos_, gantry_pos_, gantry_speed_; // ros Float32 to send msg
    if (sys.is_running()) {
      // Get position data
#ifdef REVERSE
      la1_pos_.data = sys.getMotorPos(0) / LA_RATIO;
      la2_pos_.data = sys.getMotorPos(1) / LA_RATIO;
      la3_pos_.data = sys.getMotorPos(2) / LA_RATIO;
      gantry_speed_fb = - ( sys.getGantrySpd(0) + sys.getGantrySpd(1) ) / 2; // Rad/s
#else
      la1_pos_.data = - sys.getMotorPos(0) / LA_RATIO;
      la2_pos_.data = - sys.getMotorPos(1) / LA_RATIO;
      la3_pos_.data = - sys.getMotorPos(2) / LA_RATIO;
      gantry_speed_fb = ( sys.getGantrySpd(0) + sys.getGantrySpd(1) ) / 2; // Rad/s
#endif // REVERSE
      if(gantry_speed_fb!=0 && a==0){
        ini_time = ros::Time::now();
        a=1;
      }
      gantry_pos += gantry_speed_fb * dt / GANTRY_RATIO;
      gantry_pos_.data = gantry_pos;
      gantry_speed_.data = gantry_speed_fb/ GANTRY_RATIO;
      // std::cout<<"1 : "<<gantry_speed_fb/ GANTRY_RATIO<<std::endl;
      // std::cout<<"2 : "<<gantry_pos/(current_time-ini_time).toSec()<<std::endl;
      gantry_speed_pub.publish(gantry_speed_);
      gantry_pub.publish(gantry_pos_);
      la1_pub.publish(la1_pos_);
      la2_pub.publish(la2_pos_);
      la3_pub.publish(la3_pos_);

      // Send position commend
#ifdef REVERSE
      sys.setnPos(0, int(la1_pos * LA_RATIO));
      sys.setnPos(1, int(la2_pos * LA_RATIO));
      sys.setnPos(2, int(la3_pos * LA_RATIO));
      sys.setGoSpeed(- int(gantry_speed_cmd * GANTRY_RATIO));
#else
      sys.setnPos(0, -int(la1_pos * LA_RATIO));
      sys.setnPos(1, -int(la2_pos * LA_RATIO));
      sys.setnPos(2, -int(la3_pos * LA_RATIO));
      sys.setGoSpeed(int(gantry_speed_cmd * GANTRY_RATIO));
#endif // REVERSE
      
      /* ROS_INFO("%d %d %d", -int(la1_pos * LA_RATIO), -int(la2_pos * LA_RATIO), -int(la3_pos * LA_RATIO) ); */
      //set valve pwm
      sys.setPWM(0,valve1_pwm);
      sys.setPWM(1,valve2_pwm);
      sys.setPWM(2,valve3_pwm);  
      //set valve flow
      // sys.setFlow(0,valve1_flow);
      // sys.setFlow(1,valve2_flow);
      // sys.setFlow(2,valve3_flow);  
    }

#endif // SIMULATION

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

#ifndef SIMULATION
  // Stop serial if node stop
  sys.stop();
#endif // SIMULATION

  return 0;
}



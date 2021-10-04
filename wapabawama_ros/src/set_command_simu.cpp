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

#define SIMULATION

#define GANTRY_RATIO 33.5233 // Cartision to Real 0.1(m)/(rad), Wheel R = 9.5cm = 0.095m, st ratio = 0.1
#define LA_RATIO 36666.66 // Cartision to Real (m)/(tick)

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float32.h>

#include <string>

#ifndef SIMULATION
#include "motor.hpp"
#endif // SIMULATION

#ifndef SIMULATION
/* Dsmotor mt; */
Dsmotor mt;
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

/* Position */
double la1_pos = 0;
double la2_pos = 0;
double la3_pos = 0;
double gantry_speed_cmd = 0;
double gantry_speed_fb = 0;
double gantry_pos = 0;

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
  ROS_INFO("Gantry Speed set to %f", gantry_speed_cmd);
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
  pn_.param<int>( "freq", freq, 20);

#ifndef SIMULATION
  mt.init(dev_name,bdrate);
#endif // SIMULATION

  ros::NodeHandle n;
  ros::Subscriber set_gtsped_sub   = n.subscribe(sub_gantry_tpname, 10, &set_gantryspeedCallback);
  ros::Subscriber set_la1_pose_sub = n.subscribe(sub_la1_tpname, 10, &set_la1_poseCallback);
  ros::Subscriber set_la2_pose_sub = n.subscribe(sub_la2_tpname, 10, &set_la2_poseCallback);
  ros::Subscriber set_la3_pose_sub = n.subscribe(sub_la3_tpname, 10, &set_la3_poseCallback);
  ros::Publisher  gantry_pub       = n.advertise<std_msgs::Float32>(pub_gantry_tpname, 10);
  ros::Publisher  gantry_speed_pub = n.advertise<std_msgs::Float32>(pub_gantry_speed_tpname, 10);
  ros::Publisher  la1_pub          = n.advertise<std_msgs::Float32>(pub_la1_tpname, 10);
  ros::Publisher  la2_pub          = n.advertise<std_msgs::Float32>(pub_la2_tpname, 10);
  ros::Publisher  la3_pub          = n.advertise<std_msgs::Float32>(pub_la3_tpname, 10);

#ifndef SIMULATION
  mt.run();
  mt.resetOrig();
#endif // SIMULATION

  ros::Rate loop_rate(freq);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


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
    if (mt.is_running()) {
      // Get position data
      la1_pos_.data = - mt.getMotorPos(0) / LA_RATIO;
      la2_pos_.data = - mt.getMotorPos(1) / LA_RATIO;
      la3_pos_.data = - mt.getMotorPos(2) / LA_RATIO;
      gantry_speed_fb = ( mt.getGantrySpd(0) + mt.getGantrySpd(1) ) / 2; // Rad/s
      gantry_pos += gantry_speed_fb * dt / GANTRY_RATIO;
      gantry_pos_.data = gantry_pos;
      gantry_speed_.data = gantry_speed_fb;

      gantry_speed_pub.publish(gantry_speed_);
      gantry_pub.publish(gantry_pos_);
      la1_pub.publish(la1_pos_);
      la2_pub.publish(la2_pos_);
      la3_pub.publish(la3_pos_);

      // Send position commend
      mt.setnPos(0, -int(la1_pos * LA_RATIO));
      mt.setnPos(1, -int(la2_pos * LA_RATIO));
      mt.setnPos(2, -int(la3_pos * LA_RATIO));
      mt.setGoSpeed(int(gantry_speed_cmd * GANTRY_RATIO));
      /* ROS_INFO("%d %d %d", -int(la1_pos * LA_RATIO), -int(la2_pos * LA_RATIO), -int(la3_pos * LA_RATIO) ); */
    }

#endif // SIMULATION

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

#ifndef SIMULATION
  // Stop serial if node stop
  mt.stop();
#endif // SIMULATION

  return 0;
}



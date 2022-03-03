/*
 * main.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float32.h>
#include <cmath>

class Follower{
  std::string path_sub_name;
  std::string pose_pub_name;
  std::string frame_name;

  ros::NodeHandle nh_;

  ros::Subscriber path_sub;
  ros::Publisher  pose_pub;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  std::vector<geometry_msgs::PoseStamped> path_;
  float integrator = 0;
  float kp = 0.1;
  float integrator_max; // 0 for no limit
  float orig;

  public:
  Follower() : tfListener(tfBuffer){
    ros::NodeHandle pn_("~");

    pn_.param<std::string>( "path_sub_name", path_sub_name, "la/path");
    pn_.param<std::string>( "pose_pub_name", pose_pub_name, "la/pose");
    pn_.param<std::string>( "frame_name", frame_name, "la");
    pn_.param<float>( "kp", kp, 0.1);
    pn_.param<float>( "integrator_max", integrator_max, 1);
    pn_.param<float>( "orig", orig, 0);

    pose_pub = nh_.advertise<std_msgs::Float32>(pose_pub_name, 10);
    path_sub = nh_.subscribe(path_sub_name, 10, &Follower::pathCallback, this);
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    path_ = msg->poses;
  }

  void loop() {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform("map", frame_name, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    // Linear Actuator Follow Path
    float la_y = transformStamped.transform.translation.y;
    float la_x = transformStamped.transform.translation.x;
    // std::cout << "frame_name:" << 1000*la_y << std::endl;
    if ( !path_.empty() ) {
      auto cursor = path_.rbegin();
      while (cursor != ( path_.rend()-1 ) && cursor->pose.position.y > la_y) { cursor++; }
      float follow_x = cursor->pose.position.x;

      std_msgs::Float32 pub_pose_;
      /* if ( integrator_max == 0 ) { */
      /*   integrator += ( follow_x - la_x ) * kp; */
      /* } else { */
      /*   integrator += ( follow_x - la_x ) * kp; */
      /*   integrator = integrator > integrator_max ? integrator_max : integrator; */
      /*   integrator = integrator < -integrator_max ? -integrator_max : integrator; */
      /* } */
      /* pub_pose_.data = integrator; */
      pub_pose_.data = follow_x - orig;

      /* ROS_INFO("%f %f %f %f", integrator, follow_x, la_x, pub_pose_.data); */

      pose_pub.publish(pub_pose_);
    }
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "follower");
  Follower fo_;
  // ros::Rate loop_rate(50);
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    fo_.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



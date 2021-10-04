/*
 * main.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

class Recorder{
  std::string frame_name;
  std::string record_name;
  nav_msgs::Path path_;

  ros::NodeHandle nh_;
  ros::Publisher  path_pub;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  public:
  Recorder() : tfListener(tfBuffer){
    ros::NodeHandle pn_("~");
    pn_.param<std::string>( "record_name", record_name, "la/path");
    pn_.param<std::string>( "frame_name", frame_name, "la");

    path_.header.frame_id = "map";

    path_pub = nh_.advertise<nav_msgs::Path>(record_name, 10);

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

    geometry_msgs::PoseStamped pose_;
    pose_.pose.position.x = transformStamped.transform.translation.x;
    pose_.pose.position.y = transformStamped.transform.translation.y;
    pose_.header.stamp = ros::Time::now();
    pose_.header.frame_id = "map";
    path_.poses.push_back(pose_);

    path_.header.stamp = ros::Time::now();
    path_pub.publish(path_);

  }
  
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_recorder");
  Recorder rc_;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    rc_.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



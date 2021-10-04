/*
 * tf.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

std::string gantry_name;
std::string header_name;

/* void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){ */
void poseCallback(const std_msgs::Float32::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = header_name;
  transformStamped.child_frame_id = gantry_name;
  transformStamped.transform.translation.x = 0.0;
  /* transformStamped.transform.translation.y = msg->position.y; */
  transformStamped.transform.translation.y = msg->data;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gantry_tf2_broadcaster");
  ros::NodeHandle pn_("~");

  pn_.param<std::string>( "gantry_name", gantry_name, "gantry" );
  pn_.param<std::string>( "header_name", header_name, "world" );

  ROS_INFO("Gantry Name: %s, Header Name: %s", gantry_name.c_str(), header_name.c_str());

  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe(gantry_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

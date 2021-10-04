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
std::string la_name;
float x_bias;
float y_bias;

void poseCallback(const std_msgs::Float32::ConstPtr& msg){
/* void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){ */
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = gantry_name;
  transformStamped.child_frame_id = la_name;
  /* transformStamped.transform.translation.x = msg->position.x + x_bias; */
  transformStamped.transform.translation.x = msg->data + x_bias;
  transformStamped.transform.translation.y = y_bias;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  br.sendTransform(transformStamped);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "linear_actuator_tf2_broadcaster");

  ros::NodeHandle pn_("~");
  pn_.param<std::string>( "gantry_name", gantry_name, "gantry" );
  pn_.param<std::string>( "la_name", la_name, "la0" );
  pn_.param<float>( "x_bias", x_bias, 0.0 );
  pn_.param<float>( "y_bias", y_bias, 0.0 );

  ROS_INFO("Gantry Name: %s, Actuator Name: %s", gantry_name.c_str(), la_name.c_str());

  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe(la_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

/*
 * main.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <cmath>

/*
 * pabawama(Path Base Water Machine)
 *
 * Sub:
 *  -  /move_base_simple/goal(geometry_msgs::PoseStamped) from rviz
 *
 * Pub:
 *  - pabawama_ros/lgvs(geometry_msgs::PoseArray)
 */

class LgvDetector{
  ros::NodeHandle nh_;

  ros::Publisher lgv_pub_;
  ros::Subscriber sub_;


public:
  LgvDetector() {

    lgv_pub_ = nh_.advertise<geometry_msgs::PoseArray>("lgvs", 10);
    sub_ = nh_.subscribe("/move_base_simple/goal", 10, &LgvDetector::goalCallback, this);


  }

  ~LgvDetector() {
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseArray lgvs_array;
    lgvs_array.header.stamp = ros::Time::now();
    lgvs_array.header.frame_id = "map";
    lgvs_array.poses.push_back(msg->pose);
    lgv_pub_.publish(lgvs_array);
  }

};


int main(int argc, char **argv) {
  ros::init(argc, argv, "lgv_detector_node");
  LgvDetector lgvd;
  ros::spin();
  return 0;
}



/*
 * main.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
/* #include <algorithm> */
#include <boost/algorithm/clamp.hpp>

class Planner
{
  std::string path_name;
  std::string sub_topic;
  float center_x;         // Path center
  float max_y;            // Max path dist ( Path size)
  float interval;         // Interval between each point.y
  float lgv_dist_range;   // Distance from center_x that make thix lgv avaliable
  float lgv_effect_range; // Distance that lgv can modify path
  float lgv_fusion_ratio; // lgv fusion path ratio
  float path_x_max;
  int path_size;

  ros::NodeHandle nh_;

  ros::Publisher path_pub;
  ros::Subscriber lgvs_sub;

  nav_msgs::Path la_path;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

public:
  Planner() : tfListener(tfBuffer)
  {
    ros::NodeHandle pn_("~");
    pn_.param<std::string>("path_name", path_name, "la/path");
    pn_.param<std::string>("sub_topic", sub_topic, "lgvs");
    pn_.param<float>("center_x", center_x, 0);
    pn_.param<float>("max_y", max_y, 1);
    pn_.param<float>("interval", interval, 1e-4);
    pn_.param<float>("lgv_dist_range", lgv_dist_range, 0.1);
    pn_.param<float>("lgv_effect_range", lgv_effect_range, 0.1);
    pn_.param<float>("lgv_fusion_ratio", lgv_fusion_ratio, 3);
    pn_.param<float>("path_x_max", path_x_max, 0.3);
    path_size = max_y / interval;

    la_path.header.frame_id = "map";

    path_pub = nh_.advertise<nav_msgs::Path>(path_name, 10);
    lgvs_sub = nh_.subscribe(sub_topic, 10, &Planner::lgvsCallback, this);

    // Init path
    for (int i = 0; i < path_size; i++)
    {
      push_new_point(i * interval);
    }
  }

  void push_new_point(float y)
  {
    geometry_msgs::PoseStamped pose_;
    pose_.pose.position.x = center_x;
    pose_.pose.position.y = y;
    pose_.header.stamp = ros::Time::now();
    pose_.header.frame_id = "map";
    la_path.poses.push_back(pose_);
  }

  // void lgvsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
  //   for (auto& lgv:msg->lgvs) {
  //     if ( abs( lgv.pose.position.x - center_x ) < lgv_dist_range ) {
  //       auto last_y = la_path.poses.back().pose.position.y;
  //       if ( lgv.pose.position.y < last_y ) {
  //         // Only if lgv in this range will be done
  //         auto q = lgv.pose.orientation;
  //         float angle = atan2(2 * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  //         /* angle = boost::algorithm::clamp(angle, 0.5, 2.64); */
  //         if (angle < 0.2 && angle > -0.2) continue;

  //         lgv_main(
  //             lgv.pose.position.x,
  //             lgv.pose.position.y,
  //             angle);
  //       }
  //     }
  //   }
  // }
  void lgvsCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
  {
    for (auto &lgv : msg->poses)
    {
      if (abs(lgv.position.x - center_x) < lgv_dist_range)
      {
        auto last_y = la_path.poses.back().pose.position.y;
        if (lgv.position.y < last_y)
        {
          // Only if lgv in this range will be done
          auto q = lgv.orientation;
          float angle = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
          /* angle = boost::algorithm::clamp(angle, 0.5, 2.64); */
          if (angle < 0.2 && angle > -0.2)
            continue;

          lgv_main(
              lgv.position.x,
              lgv.position.y,
              angle);
        }
      }
    }
  }
  void lgv_main(float x_, float y_, float angle_)
  {

    auto center_cursor = la_path.poses.rbegin();
    // Use path end rather then begin
    // Because end always keep a distance from gantry, but begin is not

    // Search lgv center is where
    while (center_cursor != (la_path.poses.rend()) && center_cursor->pose.position.y > y_)
    {
      center_cursor++;
    }

    if (center_cursor == la_path.poses.rend())
      return;

    // from center, modify the path to fit vector
    // Use simple line method
    auto cursor = center_cursor; // Search to positive
    while (cursor != la_path.poses.rbegin() && cursor->pose.position.y < y_ + lgv_effect_range)
    {
      /* float target_x = x_ + (cursor->pose.position.y - y_) / tan(angle_); */
      float target_x = x_ + (cursor->pose.position.y - y_) / tan(angle_ + M_PI / 2) * cos((cursor->pose.position.y - y_) / lgv_effect_range * M_PI / 2);
      cursor->pose.position.x = (cursor->pose.position.x + target_x * lgv_fusion_ratio) / (1 + lgv_fusion_ratio);
      cursor->pose.position.x = boost::algorithm::clamp(cursor->pose.position.x, center_x - path_x_max, center_x + path_x_max);
      /* cursor->pose.position.x = cursor->pose.position.x > path_x_max ? path_x_max : cursor->pose.position.x; */
      cursor--;
    }
    cursor = center_cursor + 1; // Search to negative
    while (cursor != (la_path.poses.rend()) && cursor->pose.position.y > y_ - lgv_effect_range)
    {
      /* float target_x = x_ + (cursor->pose.position.y - y_) / tan(angle_); */
      float target_x = x_ + (cursor->pose.position.y - y_) / tan(angle_ + M_PI / 2) * cos((cursor->pose.position.y - y_) / lgv_effect_range * M_PI / 2);
      cursor->pose.position.x = (cursor->pose.position.x + target_x * lgv_fusion_ratio) / (1 + lgv_fusion_ratio);
      /* cursor->pose.position.x = cursor->pose.position.x > path_x_max ? path_x_max : cursor->pose.position.x; */
      cursor->pose.position.x = boost::algorithm::clamp(cursor->pose.position.x, center_x - path_x_max, center_x + path_x_max);
      cursor++;
    }
  }

  void loop()
  {
    // Get Gantry <> Map tf
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("map", "gantry", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    try
    {
      // Always push more point to path so that back - now > y_max
      float gantry_y = transformStamped.transform.translation.y;


      auto last_y = la_path.poses.back().pose.position.y;
      while (last_y - gantry_y < max_y)
      {
        last_y += interval;
        push_new_point(last_y);
      }

      // Pub
      la_path.header.stamp = ros::Time::now();
      path_pub.publish(la_path);
    }
    catch (char const *error)
    {
      ROS_ERROR("%s", error);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_planner");
  Planner pn_;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    pn_.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

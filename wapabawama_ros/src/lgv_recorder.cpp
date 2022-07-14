/*
 * main.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */
#define Max_lgvs 500

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <wapabawama_ros/lgvs.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <unordered_map>
std::string sub_lgvs_topic_left;
std::string sub_lgvs_topic_right;
std::string pub_lgvs_topic;
geometry_msgs::PoseArray lgvs_record_left;
geometry_msgs::PoseArray lgvs_record_right;
std::unordered_map<int,geometry_msgs::Pose> lgvs_map_left;
std::unordered_map<int,geometry_msgs::Pose> lgvs_map_right;
int left_record_limit;
int right_record_limit;
void left_lgvs_Callback(const wapabawama_ros::lgvs::ConstPtr &msg)
{

    for (auto &lgv : msg->lgvs)
    {
        // std::cout << "left:" << lgv.pose.position.x << std::endl;
        if (lgv.pose.position.x <= left_record_limit)
        {   
            // add vector capacity when new object be tracked
            if (lgvs_map_left.count(lgv.ID)){ 
                if (lgv.count == 1)
                {
                    lgvs_map_left[lgv.ID] = lgv.pose;
                }
                else
                {   
                    int n = lgv.count;
                    lgvs_map_left[lgv.ID].position.x = (lgvs_map_left[lgv.ID].position.x * (n - 1) + lgv.pose.position.x) / n;
                    lgvs_map_left[lgv.ID].position.y = (lgvs_map_left[lgv.ID].position.y * (n - 1) + lgv.pose.position.y) / n;
                    auto q_old = lgvs_map_left[lgv.ID].orientation;
                    auto q_new = lgv.pose.orientation;
                    float angle_old = atan2(2 * (q_old.w * q_old.z), q_old.w * q_old.w - q_old.z * q_old.z);
                    float angle_new = atan2(2 * (q_new.w * q_new.z), q_new.w * q_new.w - q_new.z * q_new.z);
                    if (abs(angle_new - angle_old) < M_PI/2)
                    {
                        angle_old = (angle_old * (n - 1) + angle_new) / n;
                        
                    }
                    else 
                    {   
                        angle_old = angle_new;    
                        // angle_old = (angle_old * (n - 1) + angle_new - M_PI) / n;
                        // if(angle_old<0)
                        //     angle_old+=M_PI;

                    }   
                    tf2::Quaternion quat_tf2;
                    quat_tf2.setRPY(0, 0, angle_old);
                    quat_tf2.normalize();
                    lgvs_map_left[lgv.ID].orientation = tf2::toMsg(quat_tf2);
                }
            }
            else
            {
                lgvs_map_left[lgv.ID] = lgv.pose;
            }
        }
    }
}
void right_lgvs_Callback(const wapabawama_ros::lgvs::ConstPtr &msg)
{

    for (auto &lgv : msg->lgvs)
    {
        // std::cout << "right:" << lgv.pose.position.x << std::endl;
        if (lgv.pose.position.x >= right_record_limit)
        {   
            // add vector capacity when new object be tracked
            if (lgvs_map_right.count(lgv.ID)){ 
                if (lgv.count == 1)
                {
                    lgvs_map_right[lgv.ID] = lgv.pose;
                }
                else
                {   
                    int n = lgv.count;
                    lgvs_map_right[lgv.ID].position.x = (lgvs_map_right[lgv.ID].position.x * (n - 1) + lgv.pose.position.x) / n;
                    lgvs_map_right[lgv.ID].position.y = (lgvs_map_right[lgv.ID].position.y * (n - 1) + lgv.pose.position.y) / n;
                    auto q_old = lgvs_map_right[lgv.ID].orientation;
                    auto q_new = lgv.pose.orientation;
                    float angle_old = atan2(2 * (q_old.w * q_old.z), q_old.w * q_old.w - q_old.z * q_old.z);
                    float angle_new = atan2(2 * (q_new.w * q_new.z), q_new.w * q_new.w - q_new.z * q_new.z);
                    
                    if (abs(angle_new - angle_old) < M_PI/2)
                    {
                        angle_old = (angle_old * (n - 1) + angle_new ) / n;
                      
                    }
                    else 
                    {   
                        angle_old = angle_new;    
                        // angle_old = (angle_old * (n - 1) + angle_new - M_PI) / n;
                        // if(angle_old<0)
                        //     angle_old+=M_PI;

                    }   
                    // std::cout<<angle_old<<std::endl;
                    tf2::Quaternion quat_tf2;
                    quat_tf2.setRPY(0, 0, angle_old);
                    quat_tf2.normalize();
                    lgvs_map_right[lgv.ID].orientation = tf2::toMsg(quat_tf2);
                }
            }
            else
            {
                lgvs_map_right[lgv.ID] = lgv.pose;
            }
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lgv_recorder");
    ros::NodeHandle pn_("~");
    pn_.param<std::string>("sub_lgvs_topic_left", sub_lgvs_topic_left, "/left/lgvs_tracked");
    pn_.param<std::string>("sub_lgvs_topic_right", sub_lgvs_topic_right, "/right/lgvs_tracked");
    pn_.param<std::string>("pub_lgvs_topic", pub_lgvs_topic, "/lgvs_record");
    pn_.param<int>("left_record_limit", left_record_limit, 0.04);
    pn_.param<int>("right_record_limit", right_record_limit, -0.05);
    ros::NodeHandle n;
    ros::Subscriber lgvs_subl = n.subscribe("/left/lgvs_tracked", 10, &left_lgvs_Callback);
    ros::Subscriber lgvs_subr = n.subscribe("/right/lgvs_tracked", 10, &right_lgvs_Callback);
    ros::Publisher lgvs_pub = n.advertise<geometry_msgs::PoseArray>(pub_lgvs_topic, 10);
    ros::Rate loop_rate(10);
    ros::Time current_time, ini_time;
    ini_time = ros::Time::now();
    while (ros::ok())
    {
        current_time = ros::Time::now();
        int dt = (current_time - ini_time).toSec();
        if (dt > 5)
            break;
    }
    while (ros::ok())
    {
        geometry_msgs::PoseArray lgvs_record;
        int orchid_count;
        orchid_count = lgvs_record_left.poses.size() + lgvs_record_right.poses.size();

        for(const auto& lgv :lgvs_map_left){
            lgvs_record.poses.push_back(lgv.second);
        }
        for(const auto& lgv :lgvs_map_right){
            lgvs_record.poses.push_back(lgv.second);
        }
        lgvs_record.header.stamp = ros::Time::now();
        lgvs_record.header.frame_id = "map";
        // std::cout<<lgvs_record;
        lgvs_pub.publish(lgvs_record);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
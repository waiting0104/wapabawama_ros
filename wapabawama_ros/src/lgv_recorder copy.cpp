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

std::string sub_lgvs_topic_left;
std::string sub_lgvs_topic_right;
std::string pub_lgvs_topic;
geometry_msgs::PoseArray lgvs_record_left;
geometry_msgs::PoseArray lgvs_record_right;
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
            if (lgv.ID > lgvs_record_left.poses.size())
            { 
                lgvs_record_left.poses.resize(lgv.ID);
                lgvs_record_left.poses.at(lgv.ID - 1) = lgv.pose;
            }

            else
            {
                // pushback the pose of this ID object to pose array 
                if (lgv.count == 1)
                {
                    lgvs_record_left.poses.at(lgv.ID - 1) = lgv.pose;
                }
                else
                {   //average pose
                    int n = lgv.count;
                    if (lgvs_record_left.poses.at(lgv.ID - 1).position.y != 0)
                    {
                        lgvs_record_left.poses.at(lgv.ID - 1).position.x = (lgvs_record_left.poses.at(lgv.ID - 1).position.x * (n - 1) + lgv.pose.position.x) / n;
                        lgvs_record_left.poses.at(lgv.ID - 1).position.y = (lgvs_record_left.poses.at(lgv.ID - 1).position.y * (n - 1) + lgv.pose.position.y) / n;
                        auto q_old = lgvs_record_left.poses.at(lgv.ID - 1).orientation;
                        auto q_new = lgv.pose.orientation;
                        float angle_old = atan2(2 * (q_old.w * q_old.z), q_old.w * q_old.w - q_old.z * q_old.z);
                        float angle_new = atan2(2 * (q_new.w * q_new.z), q_new.w * q_new.w - q_new.z * q_new.z);
                        if (abs(angle_new - angle_old) < 2.5)
                        {
                            angle_old = (angle_old * (n - 1) + angle_new) / n;
                        }
                        else
                        {
                            angle_old = angle_new;
                        }
                        tf2::Quaternion quat_tf;
                        quat_tf.setRPY(0, 0, angle_old);
                        quat_tf.normalize();
                        lgvs_record_left.poses.at(lgv.ID - 1).orientation = tf2::toMsg(quat_tf);
                    }
                    else
                    {
                        lgvs_record_left.poses.at(lgv.ID - 1) = lgv.pose;
                    }
                }
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
            if (lgv.ID > lgvs_record_right.poses.size())
            { // add vector capacity when new object be tracked
                lgvs_record_right.poses.resize(lgv.ID);
                lgvs_record_right.poses.at(lgv.ID - 1) = lgv.pose;
            }

            else
            {
                if (lgv.count == 1)
                {
                    lgvs_record_right.poses.at(lgv.ID - 1) = lgv.pose;
                }
                else
                { //Pose fusion
                    int n = lgv.count;
                    if (lgvs_record_right.poses.at(lgv.ID - 1).position.y != 0)
                    {
                        lgvs_record_right.poses.at(lgv.ID - 1).position.x = (lgvs_record_right.poses.at(lgv.ID - 1).position.x * (n - 1) + lgv.pose.position.x) / n;
                        lgvs_record_right.poses.at(lgv.ID - 1).position.y = (lgvs_record_right.poses.at(lgv.ID - 1).position.y * (n - 1) + lgv.pose.position.y) / n;
                        auto q_old = lgvs_record_right.poses.at(lgv.ID - 1).orientation;
                        auto q_new = lgv.pose.orientation;
                        float angle_old = atan2(2 * (q_old.w * q_old.z), q_old.w * q_old.w - q_old.z * q_old.z);
                        float angle_new = atan2(2 * (q_new.w * q_new.z), q_new.w * q_new.w - q_new.z * q_new.z);
                        if (abs(angle_new - angle_old) < 2.5)
                        {
                            angle_old = (angle_old * (n - 1) + angle_new) / n;
                        }
                        else
                        {
                            // angle_old = (angle_old * (n - 1) + angle_new + M_PI) / n;
                            angle_old = angle_new;
                        }
                        tf2::Quaternion quat_tf2;
                        quat_tf2.setRPY(0, 0, angle_old);
                        quat_tf2.normalize();
                        lgvs_record_right.poses.at(lgv.ID - 1).orientation = tf2::toMsg(quat_tf2);
                    }
                    else
                    {
                        lgvs_record_right.poses.at(lgv.ID - 1) = lgv.pose;
                    }
                }
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
    ros::Rate loop_rate(15);
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

        if (!lgvs_record_left.poses.empty())
        {
            lgvs_record.poses.insert(lgvs_record.poses.end(), lgvs_record_left.poses.begin(), lgvs_record_left.poses.end());
        }
        if (!lgvs_record_right.poses.empty())
        {
            lgvs_record.poses.insert(lgvs_record.poses.end(), lgvs_record_right.poses.begin(), lgvs_record_right.poses.end());
        }
        lgvs_record.header.stamp = ros::Time::now();
        lgvs_record.header.frame_id = "map";
        lgvs_pub.publish(lgvs_record);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
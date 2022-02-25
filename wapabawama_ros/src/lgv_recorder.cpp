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
int left_record_limit = 0.04;
int right_record_limit = -0.05;
void left_lgvs_Callback(const wapabawama_ros::lgvs::ConstPtr &msg)
{
    // std::cout<<"l : "<<msg->lgvs.size()<<std::endl;
    for (auto &lgv : msg->lgvs)
    {
        // std::cout << "left.y:" << lgv.pose.position.y << std::endl;
        if (lgv.pose.position.x <= left_record_limit)
        {
            if (lgv.ID > lgvs_record_left.poses.size())
            { // add vector capacity when new object be tracked
                lgvs_record_left.poses.resize(lgv.ID);
                lgvs_record_left.poses.at(lgv.ID - 1) = lgv.pose;
            }

            else
            {
                if (lgv.count == 1)
                {
                    lgvs_record_left.poses.at(lgv.ID - 1) = lgv.pose;
                }
                else
                { //Pose fusion
                    int n = lgv.count;

                    lgvs_record_left.poses.at(lgv.ID - 1).position.x = (lgvs_record_left.poses.at(lgv.ID - 1).position.x * (n - 1) + lgv.pose.position.x) / n;
                    lgvs_record_left.poses.at(lgv.ID - 1).position.y = (lgvs_record_left.poses.at(lgv.ID - 1).position.y * (n - 1) + lgv.pose.position.y) / n;
                   
                    lgvs_record_left.poses.at(lgv.ID - 1).orientation.z = (lgvs_record_left.poses.at(lgv.ID - 1).orientation.z * (n - 1) + lgv.pose.orientation.z) / n;
                    lgvs_record_left.poses.at(lgv.ID - 1).orientation.w = (lgvs_record_left.poses.at(lgv.ID - 1).orientation.w * (n - 1) + lgv.pose.orientation.w) / n;
                    // auto q = lgvs_record_left.poses.at(lgv.ID - 1).orientation;
                    // float angle = atan2(2 * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)*180/3.14;
                    
                    // lgvs_record_left.header.stamp = ros::Time::now();
                    // lgvs_record_left.header.frame_id = "map";
                }
            }
        }
    }
}
void right_lgvs_Callback(const wapabawama_ros::lgvs::ConstPtr &msg)
{

    for (auto &lgv : msg->lgvs)
    {
        // std::cout << "right.y:" << lgv.pose.position.y << std::endl;
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

                    lgvs_record_right.poses.at(lgv.ID - 1).position.x = (lgvs_record_right.poses.at(lgv.ID - 1).position.x * (n - 1) + lgv.pose.position.x) / n;
                    lgvs_record_right.poses.at(lgv.ID - 1).position.y = (lgvs_record_right.poses.at(lgv.ID - 1).position.y * (n - 1) + lgv.pose.position.y) / n;
                    lgvs_record_right.poses.at(lgv.ID - 1).orientation.z = (lgvs_record_right.poses.at(lgv.ID - 1).orientation.z * (n - 1) + lgv.pose.orientation.z) / n;
                    lgvs_record_right.poses.at(lgv.ID - 1).orientation.w = (lgvs_record_right.poses.at(lgv.ID - 1).orientation.w * (n - 1) + lgv.pose.orientation.w) / n;
                    // auto q = lgvs_record_right.poses.at(lgv.ID - 1).orientation;
                    // float angle = atan2(2 * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)*180/3.14;
                    // lgvs_record_right.header.stamp = ros::Time::now();
                    // lgvs_record_right.header.frame_id = "map";
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
    // pn_.param<std::string>( "pub_lgvs_topic_right" , pub_lgvs_topic_right, "/right/lgvs_record" );
    ros::NodeHandle n;
    ros::Subscriber lgvs_subl = n.subscribe("/left/lgvs_tracked", 10, &left_lgvs_Callback);
    ros::Subscriber lgvs_subr = n.subscribe("/right/lgvs_tracked", 10, &right_lgvs_Callback);
    ros::Publisher lgvs_pub = n.advertise<geometry_msgs::PoseArray>(pub_lgvs_topic, 10);
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        geometry_msgs::PoseArray lgvs_record;
        int orchid_count;
        orchid_count = lgvs_record_left.poses.size() + lgvs_record_right.poses.size();
        // std::cout<<orchid_count<<std::endl;
        lgvs_record.poses.insert(lgvs_record.poses.end(), lgvs_record_left.poses.begin(), lgvs_record_left.poses.end());
        lgvs_record.poses.insert(lgvs_record.poses.end(), lgvs_record_right.poses.begin(), lgvs_record_right.poses.end());
        lgvs_record.header.stamp = ros::Time::now();
        lgvs_record.header.frame_id = "map";
        lgvs_pub.publish(lgvs_record);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
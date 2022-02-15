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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>


std::string sub_lgvs_topic_left;
std::string sub_lgvs_topic_right;
std::string pub_lgvs_topic;
struct Lgv {
    double x, y, angle;
    int ID;
    int count;
};
wapabawama_ros::lgvs lgvs_record_left;
wapabawama_ros::lgvs lgvs_record_right;

void set_lgvs_Callbackl(const wapabawama_ros::lgvs::ConstPtr& msg){
    std::cout<<"l : "<<msg->lgvs.size()<<std::endl;
     for ( auto& lgv:msg->lgvs ){

         if ( lgv.ID>lgvs_record.lgvs.size() ) { // add vector capacity when new object be tracked
             lgvs_record_left.lgvs.resize(lgv.ID);
             lgvs_record_left.lgvs.at(lgv.ID-1) = lgv; 
         }

         else {
             if (lgv.count == 1){
                lgvs_record.lgvs.at(lgv.ID-1) = lgv; 
             }
             else{ //Pose fusion
                int n = lgv.count;
                auto q = lgvs_record_left.lgvs.at(lgv.ID-1).pose ;
                q.position.x = (q.position.x*(n-1) + lgv.pose.position.x)/n;
                q.position.y = (q.position.y*(n-1) + lgv.pose.position.y)/n;
                q.orientation.z = (q.orientation.z*(n-1) + lgv.pose.orientation.z)/n;
                q.orientation.w = (q.orientation.w*(n-1) + lgv.pose.orientation.w)/n;
                lgvs_record_left.lgvs.at(lgv.ID-1).count = n ;

                lgvs_record_left.header.stamp = ros::Time::now();
                lgvs_record_left.header.frame_id = "map";
             }
         }
     }
}
void set_lgvs_Callbackr(const wapabawama_ros::lgvs::ConstPtr& msg){
    std::cout<<"r : "<<msg->lgvs.size()<<std::endl;
     for ( auto& lgv:msg->lgvs ){

         if ( lgv.ID>lgvs_record.lgvs.size() ) { // add vector capacity when new object be tracked
             lgvs_record_right.lgvs.resize(lgv.ID);
             lgvs_record_right.lgvs.at(lgv.ID-1) = lgv; 
         }

         else {
             if (lgv.count == 1){
                lgvs_record_right.lgvs.at(lgv.ID-1) = lgv; 
             }
             else{ //Pose fusion
                int n = lgv.count;
                auto q = lgvs_record_right.lgvs.at(lgv.ID-1).pose ;
                q.position.x = (q.position.x*(n-1) + lgv.pose.position.x)/n;
                q.position.y = (q.position.y*(n-1) + lgv.pose.position.y)/n;
                q.orientation.z = (q.orientation.z*(n-1) + lgv.pose.orientation.z)/n;
                q.orientation.w = (q.orientation.w*(n-1) + lgv.pose.orientation.w)/n;
                lgvs_record_right.lgvs.at(lgv.ID-1).count = n ;

                lgvs_record_right.header.stamp = ros::Time::now();
                lgvs_record_right.header.frame_id = "map";
             }
         }
     }
}
int main(int argc, char **argv) {
    
    ros::init(argc, argv, "lgv_recorder");
    ros::NodeHandle pn_("~");
    pn_.param<std::string>( "sub_lgvs_topic", sub_lgvs_topic_left, "/left/lgvs_tracked" );
    pn_.param<std::string>( "sub_lgvs_topic", sub_lgvs_topic_right, "/right/lgvs_tracked" );
    pn_.param<std::string>( "pub_lgvs_topic" , pub_lgvs_topic, "lgvs_record" );
    ros::NodeHandle n;
    ros::Subscriber lgvs_subl = n.subscribe("/left/lgvs_tracked", 10, &set_lgvs_Callbackl);
    ros::Subscriber lgvs_subr = n.subscribe("/right/lgvs_tracked", 10, &set_lgvs_Callbackr);
    ros::Publisher  lgvs_pub = n.advertise<wapabawama_ros::lgvs>(pub_lgvs_topic, 10);
    ros::Rate loop_rate(30);

    while (ros::ok()) {
        lgvs_pub.publish(lgvs_record);
        ros::spinOnce();
        loop_rate.sleep();        
    }
}
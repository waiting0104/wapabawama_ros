/*
 * watering.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <boost/algorithm/clamp.hpp>
/*
watering_mode : 1 is keep watering mode
                2 is stop watering when moving mode
                3 is stop moving when detecting orchids
                4 is stop action
                */
class Valve
{
    ros::NodeHandle nh_;
    std::string path_name;
    std::string frame_name;
    std::string pwm_name;
    std::string gantry_speed_name = "gantry/set_speed";
    ros::Subscriber lgvs_sub;

    ros::Time current_stopping_time, ini_stopping_time;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std_msgs::Float32 gantry_speed;
    geometry_msgs::PoseArray lgvs_record;
    int watering_mode;
    int set_pwm;
    int stop_sec;
    float bias;
    float center_x;          // Path center
    float lgv_dist_range;    // Distance from center_x that make thix lgv avaliable
    float start_water_range; // Distance that lgv can modify path
    float finish_water_range;
    float set_gantry_speed;
    bool water_sign = false; 
public:
    std_msgs::Int8 valve_pwm;
    ros::Publisher valve_pwm_pub;
    ros::Publisher gantry_speed_pub;
    Valve() : tfListener(tfBuffer)
    {
        ros::NodeHandle pn_("~");
        // pn_.param<std::string>("gantry_speed_name", gantry_speed_name, "");
        pn_.param<std::string>("path_name", path_name, "la/path");
        pn_.param<std::string>("frame_name", frame_name, "la");
        pn_.param<std::string>("pwm_name", pwm_name, "va/pwm");
        pn_.param<int>("watering_mode", watering_mode, 4);
        pn_.param<float>("center_x", center_x, 0);
        pn_.param<int>("set_pwm", set_pwm, 50);
        pn_.param<int>("stop_sec", stop_sec, 2);
        pn_.param<float>("bias", bias, 0);
        pn_.param<float>("set_gantry_speed", set_gantry_speed, 0.3);
        pn_.param<float>("lgv_dist_range", lgv_dist_range, 0.1);
        pn_.param<float>("start_water_range", start_water_range, 0.08);
        pn_.param<float>("finish_water_range", finish_water_range, 0.08);
        valve_pwm_pub = nh_.advertise<std_msgs::Int8>(pwm_name, 10);
        gantry_speed_pub = nh_.advertise<std_msgs::Float32>(gantry_speed_name, 10);
        lgvs_sub = nh_.subscribe("lgvs_record", 10, &Valve::lgvsCallback, this);
    }
    ~Valve(){
        close();
    }
    void lgvsCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        lgvs_record = * msg;

        // std::cout<<"la_x:"<<la_x<<std::endl;
        // std::cout<<"la_y:"<<la_y<<std::endl;
    }
    
    void open(){
        valve_pwm.data = set_pwm;
        valve_pwm_pub.publish(valve_pwm);
    }
    void close(){
        valve_pwm.data = 0;
        valve_pwm_pub.publish(valve_pwm);
    }
    void stop_gantry(){
        gantry_speed.data = 0;
        gantry_speed_pub.publish(gantry_speed);
    }
    void stop(int t){
        std::cout << "stop!" << std::endl;
        stop_gantry();
        open();
        ini_stopping_time = ros::Time::now();
        while (1)
        {   
            current_stopping_time = ros::Time::now();
            int timer = (current_stopping_time - ini_stopping_time).toSec();
            if (timer > t)
            {   
                close();
                ros::Duration(1.5).sleep();
                gantry_speed.data = set_gantry_speed;
                gantry_speed_pub.publish(gantry_speed);
                break;
            }
        }        
    }
    void loop()
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("map", frame_name, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        // Linear Actuator Follow Path
        float la_y = transformStamped.transform.translation.y;
        float la_x = transformStamped.transform.translation.x;
        for (auto &lgv : lgvs_record.poses)
        {
            // std::cout<<"lgv.pose.position.y - start_water_range:"<<lgv.pose.position.y - start_water_range<<std::endl;
            // std::cout<<"lgv.pose.position.y + finish_water_range:"<<lgv.pose.position.y + finish_water_range<<std::endl;
            if (abs(lgv.position.x - center_x) < lgv_dist_range)
            {

                switch (watering_mode)
                {
                case 1:
                    if (!water_sign)
                    {
                        // auto q = lgv.pose.orientation;
                        // float angle = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
                        open();
                        water_sign = true;
                    }
                    break;
                case 2:
                    if (la_y > lgv.position.y - start_water_range && la_y < lgv.position.y + finish_water_range)
                    {
                        // auto q = lgv.pose.orientation;
                        // float angle = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
                        valve_pwm.data = set_pwm;
                    }
                    else
                        valve_pwm.data = 0;
                    break;
                case 3:
                    if(frame_name=="la2"){
                        std::cout << "lgv.position.y:" << round(abs(10000*(la_y-lgv.position.y-bias)))<< std::endl;
                        // std::cout << frame_name << ":"<<10000*la_y << std::endl;
                    }


                    if (round(abs(10000*(la_y-lgv.position.y-bias)))<=11 ) // if gantry is on top of orchid , stop for 3 sec to water
                    {
                        stop(stop_sec);
                    }
                    break;
                case 4:
                    gantry_speed.data = 0;
                    gantry_speed_pub.publish(gantry_speed);
                    valve_pwm.data = 0;
                default:
                    break;
                }
                
            }
        }
 
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_water");
    Valve vl;
    ros::Rate loop_rate(30);
    ros::Time current_time, ini_time;
    ini_time = ros::Time::now();

    while (ros::ok())
    {

        current_time = ros::Time::now();
        int dt = (current_time - ini_time).toSec();
        // if (dt == 2)
        // {
        //     vl.valve_pwm.data = 100;
        //     vl.valve_pwm_pub.publish(vl.valve_pwm);
        // }
        // if (dt == 3)
        // {
        //     vl.valve_pwm.data = 0;
        //     vl.valve_pwm_pub.publish(vl.valve_pwm);
        // }
        // if (dt == 25)
        // {
        //     vl.watering_mode = 4;
        // }

        ros::spinOnce();
        vl.loop();
        loop_rate.sleep();
    }
    return 0;
}
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
/* #include <algorithm> */
#include <boost/algorithm/clamp.hpp>

class Valve
{
    std::string path_name;
    std::string frame_name;
    std::string pwm_name;
    float center_x;          // Path center
    float lgv_dist_range;    // Distance from center_x that make thix lgv avaliable
    float start_water_range; // Distance that lgv can modify path
    float finish_water_range;
    bool detect_sign = false;
    ros::Subscriber lgvs_sub;
    ros::Publisher valve_pwm_pub;
    ros::NodeHandle nh_;
    // nav_msgs::Path la_path;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

public:
    std_msgs::Int8 valve_pwm;
    int watering_mode;
    int last_pwm;
    int set_pwm;
    Valve() : tfListener(tfBuffer)
    {
        ros::NodeHandle pn_("~");
        pn_.param<std::string>("path_name", path_name, "la/path");
        pn_.param<std::string>("frame_name", frame_name, "la");
        pn_.param<std::string>("pwm_name", pwm_name, "va/pwm");
        pn_.param<int>("watering_mode", watering_mode, 2);
        pn_.param<float>("center_x", center_x, 0);
        pn_.param<int>("set_pwm", set_pwm, 50);
        pn_.param<float>("lgv_dist_range", lgv_dist_range, 0.1);
        pn_.param<float>("start_water_range", start_water_range, 0.08);
        pn_.param<float>("finish_water_range", finish_water_range, 0.08);
        valve_pwm_pub = nh_.advertise<std_msgs::Int8>(pwm_name, 10);
        
        if (watering_mode == 2)
        {
            lgvs_sub = nh_.subscribe("lgvs", 10, &Valve::lgvsCallback, this);
        }
    }

    void lgvsCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        detect_sign = true;
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

        for (auto &lgv : msg->poses)
        {
            if (abs(lgv.position.x - center_x) < lgv_dist_range)
            {
                // auto last_y = la_path.poses.back().pose.position.y;
                // if (lgv.position.y < last_y)
                // {
                // Only if lgv in this range will be done
                auto q = lgv.orientation;
                float angle = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
                /* angle = boost::algorithm::clamp(angle, 0.5, 2.64); */

                if (la_y > lgv.position.y - start_water_range && la_y < lgv.position.y + finish_water_range)
                {
                    valve_pwm.data = set_pwm;
                }
                else
                    valve_pwm.data = 0;
            }
        }
    }

    void loop()
    {
        if (!detect_sign && watering_mode == 2)
            valve_pwm.data = 0;
        if (valve_pwm.data != last_pwm)
        {
            valve_pwm_pub.publish(valve_pwm);
            std::cout << "Publish " << pwm_name << " is : " << valve_pwm << std::endl;
        }

        last_pwm = valve_pwm.data;
        detect_sign = false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_water");
    Valve vl;
    ros::Rate loop_rate(10);
    ros::Time current_time, ini_time;
    ini_time = ros::Time::now();
    while (ros::ok())
    {
        if (vl.watering_mode == 1)
        {
            current_time = ros::Time::now();
            int dt = (current_time - ini_time).toSec();
            if (dt == 5)
            {
                vl.valve_pwm.data = vl.set_pwm;
            }
            if (dt == 20)
            {
                vl.valve_pwm.data = 0;
            }
        }

        ros::spinOnce();
        vl.loop();
        loop_rate.sleep();
    }
    return 0;
}

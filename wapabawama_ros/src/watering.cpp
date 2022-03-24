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
    
    
    float bias;
    float center_x;          // Path center
    float lgv_dist_range;    // Distance from center_x that make thix lgv avaliable
    float start_water_range; // Distance that lgv can modify path
    float finish_water_range;
    float set_gantry_speed;
    float flow_1,flow_2,flow_3,flow_4,flow_5,flow_6,flow_7,flow_8,flow_9,flow_10;
    bool water_sign = false; 
public:
    int set_pwm;
    int stop_sec;
    int stop_state=0;
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
        pn_.param<float>("lgv_dist_range", lgv_dist_range, 0.12);
        pn_.param<float>("start_water_range", start_water_range, 0.08);
        pn_.param<float>("finish_water_range", finish_water_range, 0.08);
        pn_.param<float>("flow_10", flow_10, 0.08);
        pn_.param<float>("flow_9", flow_9, 0.08);
        pn_.param<float>("flow_8", flow_8, 0.08);
        pn_.param<float>("flow_7", flow_7, 0.08);
        pn_.param<float>("flow_6", flow_6, 0.08);
        pn_.param<float>("flow_5", flow_5, 0.08);
        pn_.param<float>("flow_4", flow_4, 0.08);
        pn_.param<float>("flow_3", flow_3, 0.08);
        pn_.param<float>("flow_2", flow_2, 0.08);
        pn_.param<float>("flow_1", flow_1, 0.08);

        valve_pwm_pub = nh_.advertise<std_msgs::Int8>(pwm_name, 10);
        gantry_speed_pub = nh_.advertise<std_msgs::Float32>(gantry_speed_name, 10);
        lgvs_sub = nh_.subscribe("lgvs_record", 10, &Valve::lgvsCallback, this);
    }
    ~Valve(){
        close();
        stop_gantry();
    }
    void lgvsCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        lgvs_record = * msg;

   
    }
    //open valve
    void open(int pwm){
        valve_pwm.data = pwm;
        valve_pwm_pub.publish(valve_pwm);
    }
    //close valve
    void close(){
        valve_pwm.data = 0;
        valve_pwm_pub.publish(valve_pwm);
    }
    void stop_gantry(){
        gantry_speed.data = 0;
        gantry_speed_pub.publish(gantry_speed);
    }
    //stop gantry and watering for t sec
    void stop(int t){
        std::cout << "stop!" << std::endl;
        stop_state=1;
        stop_gantry();
        open(set_pwm);
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
    void testing(int t){
        open(set_pwm);
        ini_stopping_time = ros::Time::now();
        while (1)
        {   
            current_stopping_time = ros::Time::now();
            int timer = (current_stopping_time - ini_stopping_time).toSec();
            if (timer > t)
            {   
                close();
                ros::Duration(1.5).sleep();
                // gantry_speed.data = set_gantry_speed;
                // gantry_speed_pub.publish(gantry_speed);
                break;
            }
        }        
    }
    int amount_to_duty_moving(float velocity,int amount, float radius, float angle){
        float flow = velocity*amount/radius/cos(angle);
        float flowlist[11] = {0, 0.39, 6.26, 10.45, 15.31, 20.28, 22.85, 25.95, 28.40, 30.14, 32.31};
        int duty = 0;
        for(int i=0 ;i<10;i++){
            if(flowlist[i]<flow && flowlist[i+1]>=flow){
                duty = int((i+1)*10-10*(flowlist[i+1]-flow)/(flowlist[i+1]-flowlist[i]));
                break;
            }
        }
        return duty;
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
                        open(set_pwm);
                    }
                    break;
                case 2:
                    if (la_y > lgv.position.y - start_water_range && la_y < lgv.position.y + finish_water_range)
                    {
                        auto q = lgv.orientation;
                        float angle = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
                        float velocity ;
                        float radius = 7.6 ;
                        // int amount ;
                        // int amount_to_duty();
                        open(set_pwm);
                    }
                    else
                        valve_pwm.data = 0;
                    break;
                case 3:
                    
                    // std::cout << frame_name<<":lgv.position.y:" << round(abs(10000*(la_y-lgv.position.y-bias)))<< std::endl;

                    if (round(abs(10000*(la_y-lgv.position.y-bias)))<=20 && stop_state == 0 ) // if gantry is on top of orchid , stop for 3 sec to water
                    {   
                        
                        stop(stop_sec);
                    }
                    if (stop_state<=30&&stop_state!=0){
                        stop_state++;
                        // std::cout<<stop_state<<std::endl;
                    }
                    if (stop_state>30){
                        stop_state=0;
                    }
                    break;
                // case 4:
                //     testing(stop_sec);
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
    ros::Duration(2).sleep();
    // vl.testing(vl.stop_sec);
    // vl.open(vl.set_pwm);
    while (ros::ok())
    {

        // current_time = ros::Time::now();
        // int dt = (current_time - ini_time).toSec();
        // if (dt %13 == 0)
        // {
        //     vl.testing(vl.stop_sec);
        // }
        

        ros::spinOnce();
        vl.loop();
        loop_rate.sleep();
    }
    return 0;
}


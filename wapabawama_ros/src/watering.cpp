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
#include <wapabawama_ros/moisture.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <boost/algorithm/clamp.hpp>
#include <unordered_map>
#include <cmath>
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
    ros::Subscriber moisture_sub;
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
    std::unordered_map<int, float> moisture_map;
    std::vector<float> moisture_x;
    std::vector<float> moisture_y;
    std::vector<float> amount_list;
    std::vector<float> flow_list;
    std::vector<int> amount_level;
    bool water_sign = false;

public:
    int set_pwm, high_first, high_second, medium, low_second, low_first;
    int stop_sec;
    int count = 0;
    int stop_state = 0;
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
        pn_.param<float>("bias", bias, 0);
        pn_.param<float>("set_gantry_speed", set_gantry_speed, 0.3);
        pn_.param<float>("lgv_dist_range", lgv_dist_range, 0.12);
        pn_.param<float>("start_water_range", start_water_range, 0.08);
        pn_.param<float>("finish_water_range", finish_water_range, 0.08);
        pn_.getParam("amount_list", amount_list);   //amount per 10 duty cycle
        pn_.getParam("amount_level", amount_level); //amount of different soil moisture
        pn_.getParam("flow_list", flow_list);
        pn_.getParam("moisture_x", moisture_x);
        pn_.getParam("moisture_y", moisture_y);
        valve_pwm_pub = nh_.advertise<std_msgs::Int8>(pwm_name, 10);
        gantry_speed_pub = nh_.advertise<std_msgs::Float32>(gantry_speed_name, 10);
        lgvs_sub = nh_.subscribe("lgvs_record", 10, &Valve::lgvsCallback, this);
        moisture_sub = nh_.subscribe("/moisture", 10, &Valve::moistureCallback, this);
    }
    ~Valve()
    {
        close();
        stop_gantry();
    }
    void lgvsCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        lgvs_record = *msg;
    }
    void moistureCallback(const wapabawama_ros::moisture::ConstPtr &msg)
    {
        if (moisture_map[msg->ID] == 0)
        {
            moisture_map[msg->ID] = msg->data;
        }

        std::cout << "ID:" << msg->ID << std::endl
                  << "moisture:" << msg->data << std::endl
                  << "Position:(" << moisture_x[msg->ID] << "," << moisture_y[msg->ID] << ")" << std::endl;
    }
    //open valve
    void open(int pwm)
    {
        valve_pwm.data = pwm;
        valve_pwm_pub.publish(valve_pwm);
    }
    //close valve
    void close()
    {
        valve_pwm.data = 0;
        valve_pwm_pub.publish(valve_pwm);
    }
    void stop_gantry()
    {
        gantry_speed.data = 0;
        gantry_speed_pub.publish(gantry_speed);
    }
    //stop gantry and watering for t sec
    void stop(int duty)
    {
        std::cout << "stop!" << std::endl;
        stop_state = 1;
        stop_gantry();

        open(duty);
        ini_stopping_time = ros::Time::now();
        while (1)
        {
            current_stopping_time = ros::Time::now();
            int timer = (current_stopping_time - ini_stopping_time).toSec();
            if (timer > 2)
            {   std::cout<<(current_stopping_time - ini_stopping_time).toSec()<<std::endl;
                close();
                ros::Duration(1.5).sleep();
                gantry_speed.data = set_gantry_speed;
                gantry_speed_pub.publish(gantry_speed);
                break;
            }
        }
    }
    void open_moving(int duty)
    {
        

        open(duty);
        ini_stopping_time = ros::Time::now();
        while (1)
        {
            current_stopping_time = ros::Time::now();
            int timer = (current_stopping_time - ini_stopping_time).toSec();
            if (timer > 2)
            {
                close();
                ros::Duration(1.5).sleep();
                gantry_speed.data = set_gantry_speed;
                gantry_speed_pub.publish(gantry_speed);
                break;
            }
        }
    }
    void testing_amount(int t)
    {
        float amount;
        std::cout << std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl;
        std::cout << "Enter water amount"<<std::endl;
        std::cin >> amount;
        open(amount_to_duty_stop(amount));
        ini_stopping_time = ros::Time::now();
        std::cout << "Watering for " << amount << " ml";
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
    void testing_duty(int t)
    {
        int duty;
        std::cout << "enter";
        std::cin >> duty;
        open(duty);
        std::cout << "Watering for " << duty << "%";
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
    int amount_to_duty_stop(int amount)
    { // calc the duty cylce by the specific amount (open for 2 sec)
        int duty;
        for (int i = 0; i < 11; i++)
        {
            if (amount_list[i] < amount && amount_list[i + 1] >= amount)
            {
                duty = round(int((i + 1) * 10 - 10 * (amount_list[i + 1] - amount) / (amount_list[i + 1] - amount_list[i])));
                break;
            }
        }
        // std::cout<<frame_name<<":"<<duty<<std::endl;
        return duty;
    }
    int amount_to_duty_moving(float velocity, int amount, float radius, float angle)
    {
        float flow = velocity * amount / radius / cos(angle);

        int duty = 0;
        for (int i = 0; i < 10; i++)
        {
            if (flow_list[i] < flow && flow_list[i + 1] >= flow)
            {
                duty = int((i + 1) * 10 - 10 * (flow_list[i + 1] - flow) / (flow_list[i + 1] - flow_list[i]));
                break;
            }
        }
        return duty;
    }
    float get_moisture(std::unordered_map<int, float> moisture, geometry_msgs::Pose lgv)
    { //find the nearest moisture sensor
        float min_distance = 200;
        int temp;
        for (auto &moisture : moisture_map)
        {
            float distance = pow((moisture_x[moisture.first] - lgv.position.x), 2) + pow((moisture_y[moisture.first] - lgv.position.y), 2);
            if (distance < min_distance)
            {
                min_distance = distance;
                temp = moisture.first;
            }
        }
        return moisture_map[temp];
    }
    int amount_by_moisture(float moisture)
    { //assign the proper amount to specific moisture level
        if (moisture <= 33)
        {
            return amount_level[0];
        }
        else if (moisture > 33 && moisture <= 67)
        {
            return amount_level[1];
        }
        else if (moisture > 67 && moisture <= 100)
        {
            return amount_level[2];
        }
    }

    void loop()
    {
        if (watering_mode == 4)
        {
            testing_duty(2);
        }
        else if (watering_mode == 5)
        {
            testing_amount(2);
        }
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
        float distance ;
        for (auto &lgv : lgvs_record.poses)
        {
            // std::cout<<"lgv.pose.position.y - start_water_range:"<<lgv.pose.position.y - start_water_range<<std::endl;
            // std::cout<<"lgv.pose.position.y + finish_water_range:"<<lgv.pose.position.y + finish_water_range<<std::endl;
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
                        float velocity;
                        float radius = 7.6;
                        open(set_pwm);
                    }
                    else if(la_y <lgv.position.y + finish_water_range+0.01&& la_y > lgv.position.y + finish_water_range)
                        close();
                    break;
                case 3:

                    // std::cout << frame_name<<":lgv.position.y:" << round(abs(10000*(la_y-lgv.position.y-bias)))<< std::endl;
                    // distance = sqrt( pow((la_y - lgv.position.y),2) +pow((la_x - lgv.position.x),2))*100;
                    // if (frame_name=="la3"){
                    //     // std::cout<<"frame_name="<<frame_name<<":"<<distance<<std::endl;
                    // }
                    // if (distance>=2.2&&distance <= 2.65&&la_y>lgv.position.y&& stop_state == 0) // if gantry is on top of orchid , stop for 3 sec to water
                    // {   std::cout<<"STOP!!!!="<<frame_name<<":"<<int(distance)<<std::endl;
                    //     int moisture = get_moisture(moisture_map, lgv);
                    //     int amount = amount_by_moisture(moisture);
                    //     int duty = amount_to_duty_stop(30);
                    //     // std::cout << "ID:" << count << std::endl
                    //     //           << "moisture:" << moisture << "amount:" << std::endl;
                    //     stop(duty);
                    // }
                    if (round(abs(10000 * (la_y - lgv.position.y - bias))) <= 20 && stop_state == 0) // if gantry is on top of orchid , stop for 3 sec to water
                    {   std::cout<<"STOP!!!!="<<frame_name<<":"<<int(distance)<<std::endl;
                        int moisture = get_moisture(moisture_map, lgv);
                        int amount = amount_by_moisture(moisture);
                        int duty = amount_to_duty_stop(60);
                        // std::cout << "ID:" << count << std::endl
                        //           << "moisture:" << moisture << "amount:" << std::endl;
                        stop(duty);
                    }
                    
                    // if (round(abs(10000 * (la_y - lgv.position.y )))   && stop_state == 0) // if gantry is on top of orchid , stop for 3 sec to water
                    // {
                    //     int moisture = get_moisture(moisture_map, lgv);
                    //     int amount = amount_by_moisture(moisture);
                    //     int duty = amount_to_duty_stop(30);
                    //     stop(duty);
                    // }
                    if (stop_state <= 60 && stop_state != 0)
                    {
                        stop_state++;
                        std::cout<<stop_state<<std::endl;
                        // std::cout<<stop_state<<std::endl;
                    }
                    if (stop_state > 60)
                    {
                        stop_state = 0;
                    }
                    break;

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
    ros::Duration(2).sleep();
    ini_time = ros::Time::now();
    // ini_stopping_time = ros::Time::now();
    // if (vl.watering_mode==4 ||vl.watering_mode==5 ) {
    //     while (1)
    //     {
    //         current_time = ros::Time::now();
    //         int timer = (current_time - ini_time).toSec();
    //         vl.open(90);
    //         if (timer > 1)
    //         {   
    //             vl.close();
    //             ros::Duration(1.5).sleep();
    //             // gantry_speed.data = set_gantry_speed;
    //             // gantry_speed_pub.publish(gantry_speed);
    //             break;
    //         }
    //     }
    // }
        
    while (ros::ok())
    {

        // current_time = ros::Time::now();
        // int dt = (current_time - ini_time).toSec();
        // if (dt %13 == 0)
        // {
        //     vl.testing(2);
        // }

        ros::spinOnce();
        vl.loop();
        loop_rate.sleep();
    }
    return 0;
}

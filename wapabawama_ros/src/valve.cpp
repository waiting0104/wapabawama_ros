#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

int freq;// command send freq
int watering_mode;
std_msgs::Int8 pwm;
std_msgs::Float32 speed;
//ros::Time current_time,first_detect_time,first_no_detect_time;
std::string sub_state_topic;
bool set_flag = false;
bool set_valve_state = false;
bool temp_state = false;

int main(int argc, char **argv) {
    ros::init(argc, argv, "set_valve");
    ros::NodeHandle nh;
    ros::NodeHandle pn_("~");
    ros::Publisher valve1_pwm_pub_;   
    ros::Publisher valve2_pwm_pub_;   
    ros::Publisher valve3_pwm_pub_;    
    ros::Publisher gantry_speed_pub_;  
    // pn_.param<std::string>( "valve_state" , sub_state_topic , "/valve_state" );
    pn_.param<int>( "watering_mode" , watering_mode , 0);
    pn_.param<int>( "freq", freq, 20);
    valve1_pwm_pub_ = nh.advertise<std_msgs::Int8>("valve1/pwm",10);  
    gantry_speed_pub_ = nh.advertise<std_msgs::Float32>("gantry/set_speed",10); 
    // valve2_pwm_pub_ = nh.advertise<std_msgs::Int8>("valve2/pwm",10);
    // valve3_pwm_pub_ = nh.advertise<std_msgs::Int8>("valve3/pwm",10);
    ros::Rate loop_rate(freq);
    ros::Time current_time, ini_time;
    speed.data = 0.4 ;
    gantry_speed_pub_.publish(speed);
    ini_time = ros::Time::now();
    while(ros::ok()){
        current_time = ros::Time::now();
        int dt = (current_time-ini_time).toSec();
        if(dt>=13&&dt<14){
            speed.data = 0 ;
            gantry_speed_pub_.publish(speed);
            pwm.data = 50 ;
            valve1_pwm_pub_.publish(pwm);
        }
        else if(dt<25){
            
            speed.data = 0.4 ;
            gantry_speed_pub_.publish(speed);
        } 
        else if(dt>=32&&dt<33){
            pwm.data = 0 ;
            valve1_pwm_pub_.publish(pwm);
        }     
        else if(dt>=34&&dt<35){
            
            speed.data = 0.0 ;
            gantry_speed_pub_.publish(speed);
        } 
        loop_rate.sleep();
    }
    return 0;
}
 
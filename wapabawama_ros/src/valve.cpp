#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
//mode 1 : keep watering (set fixed pwm)
//mode 2 : keep watering (set flow)
//mode 3 : stop watering while not detected
//mode 4 : script
int freq;// command send freq
int watering_mode;
int pwm_all;
int pwm_1;
int pwm_2;
int pwm_3;
float ganrty_speed;
std_msgs::Int8 pwm_all_msg;
std_msgs::Int8 pwm_1_msg;
std_msgs::Int8 pwm_2_msg;
std_msgs::Int8 pwm_3_msg;
std_msgs::Float32 gantry_speed_msg;
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
    pn_.param<int>( "pwm_all", pwm_all, 0);
    pn_.param<int>( "pwm_1", pwm_1, 0);
    pn_.param<int>( "pwm_2", pwm_2, 0);
    pn_.param<int>( "pwm_3", pwm_3, 0);
    pn_.param<float>( "ganrty_speed", ganrty_speed, 0);
    valve1_pwm_pub_ = nh.advertise<std_msgs::Int8>("valve1/pwm",10);  
    valve2_pwm_pub_ = nh.advertise<std_msgs::Int8>("valve2/pwm",10);  
    valve3_pwm_pub_ = nh.advertise<std_msgs::Int8>("valve3/pwm",10); 
    gantry_speed_pub_ = nh.advertise<std_msgs::Float32>("gantry/set_speed",10);
    pwm_all_msg.data = pwm_all ; 
    pwm_1_msg.data = pwm_1 ; 
    pwm_2_msg.data = pwm_2 ; 
    pwm_3_msg.data = pwm_3 ; 
    gantry_speed_msg.data = ganrty_speed ; 
    ros::Rate loop_rate(freq);
    ros::Time current_time, ini_time;
    gantry_speed_pub_.publish(gantry_speed_msg);
    ini_time = ros::Time::now();
    while(ros::ok()){
        if(watering_mode==1){
            if(ganrty_speed != 0){
                valve1_pwm_pub_.publish(pwm_all_msg);
                valve2_pwm_pub_.publish(pwm_all_msg);
                valve3_pwm_pub_.publish(pwm_all_msg);
            }

        }
        else if(watering_mode==2){

        }
        else if(watering_mode==3){
            current_time = ros::Time::now();
            int dt = (current_time-ini_time).toSec();

            if(dt>=13&&dt<14){
                gantry_speed_msg.data = 0 ;
                gantry_speed_pub_.publish(gantry_speed_msg);
                pwm_all_msg.data = pwm_all ;
                valve1_pwm_pub_.publish(pwm_all_msg);
                valve2_pwm_pub_.publish(pwm_all_msg);
                valve3_pwm_pub_.publish(pwm_all_msg);
            }
            else if(dt<25){
                
                gantry_speed_msg.data = ganrty_speed ;
                gantry_speed_pub_.publish(gantry_speed_msg);
            } 
            else if(dt>=32&&dt<33){
                pwm_all_msg.data = 0 ;
                valve1_pwm_pub_.publish(pwm_all_msg);
            }     
            else if(dt>=34&&dt<35){
                
                gantry_speed_msg.data = 0.0 ;
                gantry_speed_pub_.publish(gantry_speed_msg);
            } 
        }
        
        loop_rate.sleep();
    }
    return 0;
}
 
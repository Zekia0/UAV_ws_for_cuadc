#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "uav_pkg/deta_xy.h"
#include <cmath> 
#include <string>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define BUFFER_SIZE 4
int baudrate = 9600;
std::string port("/dev/ttyS0");
int fd = serialOpen(port.c_str(), baudrate);
unsigned char frame[BUFFER_SIZE] = {0x2c, 0x12, 0x02, 0x5b};

float vel_init = 0.80;
int judge_counts = 0;
float camerr_x = 0;
float camerr_y = 0;
float camerr_z = 0;
float hight = 2.5;
float hight_dec = 1.5;
float k_x=0; 
float k_y=0;
float k_z=0;
float k=0.25;
int flag = 0;
int armed_flag = 0;
int offb_flag = 0; 
int sametimes = 0;
int S_to_V = 0;
const float max_speed = 0.2;

void Esp_task(){
	
    int n_written = write(fd, frame, sizeof(frame));
    if (n_written < 0) {
        ROS_ERROR("Failed to write to serial port");
	} else {
		ROS_INFO("Frame sent");
	}
}

float limit_speed(float velocity, float max_speed) {
    return std::min(std::max(velocity, -max_speed), max_speed);
} 

void deta_cb(const uav_pkg::deta_xy::ConstPtr& msg){
    camerr_x = msg -> deta_x;
    camerr_y = msg -> deta_y;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

void duidian_cam_nav(float cam_err_x,float cam_err_y,float cam_err_z){
    k_y = cam_err_x * 0.05 * k;
    k_x = cam_err_y * 0.05 * k;
    k_z = cam_err_z * 0.05 * k;
}

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), last_error(0), integral(0) {}
    void update(float error, float dt) {
        float derivative = (error - last_error) / dt;
        integral += error * dt;
        float output = kp * error + ki * integral + kd * derivative;
        last_error = error;
        // 限制输出在最大速度范围内
        output = limit_speed(output, max_speed);
    }

private:
    float kp;
    float ki;
    float kd;
    float last_error;
    float integral;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ES");
    ros::NodeHandle nh;

    // 初始化PID控制器参数
    float kp = 1.0; // 比例系数
    float ki = 0.1; // 积分系数
    float kd = 0.01; // 微分系数
    PIDController controller(kp, ki, kd);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber img_pos_sub = nh.subscribe<uav_pkg::deta_xy>
            ("/deta_xy",10,deta_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Twist velocity_command;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = hight;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( armed_flag == 0 && current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))){
            ROS_INFO("Vehicle armed");
            last_request = ros::Time::now();
            armed_flag = 1;                
        }  
        if( offb_flag == 0 && current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))){
            ROS_INFO("Offboard enabled");
            last_request = ros::Time::now();
            offb_flag = 1;
        }
        if(flag==0 && current_state.armed && current_state.mode == "OFFBOARD"){
            last_request = ros::Time::now();
            flag=1;
        }
        if(flag==1 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("point1");
            pose.pose.position.x=0;
            pose.pose.position.y=0;
            pose.pose.position.z=hight;
            last_request =ros::Time::now();
            flag = 2;
        }
        if(flag==2 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("point1_x_2.5");
            double distance_to_target = fabs(2.5 - local_pos.pose.position.x);
            double speed_x = limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed));
            pose.pose.position.x = local_pos.pose.position.x + speed_x * (ros::Time::now() - last_request).toSec();
            pose.pose.position.y=0;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.x-2.5)<0.5){
                last_request =ros::Time::now();
                flag = 3;
            }
        }
        if(flag==3 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("point1_y");
            double distance_to_target = fabs(-31.6 - local_pos.pose.position.y);
            double speed_y = -limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed));
            pose.pose.position.y = local_pos.pose.position.y + speed_y * (ros::Time::now() - last_request).toSec();
            pose.pose.position.x=2.5;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.y-(-31.6))<0.5){
                last_request =ros::Time::now();
                flag = 4;
            }
        }
        if(flag==4 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("point1_x_5");
            double distance_to_target = fabs(5 - local_pos.pose.position.x);
            double speed_x = limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed));
            pose.pose.position.x = local_pos.pose.position.x + speed_x * (ros::Time::now() - last_request).toSec();
            pose.pose.position.y=-31.6;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.x-5)<0.5){
                last_request =ros::Time::now();
                flag = 5;
            }
        }
        if(flag==5 && ros::Time::now()-last_request > ros::Duration(0.5)){
            S_to_V = 1;
            ROS_INFO("duidian_nav");
            duidian_cam_nav(camerr_x,camerr_y,0);
            velocity_command.linear.x = k_x * vel_init;  
            velocity_command.linear.y = k_y * vel_init;  
            velocity_command.linear.z = - (local_pos.pose.position.z - hight) * vel_init; 
            last_request =ros::Time::now();
            if(abs(camerr_x)<20 && abs(camerr_y)<20) judge_counts++;
            else{judge_counts = 0;}
            if(judge_counts == 5){/*  */
                ROS_WARN("CUADC");
                Esp_task();
                serialClose(fd);
                last_request =ros::Time::now();
                S_to_V = 0;
                flag = 6;
            }
            ROS_INFO("error_x:%f",camerr_x);
            ROS_INFO("error_y:%f",camerr_y);
            ROS_INFO("judge_counts:%d",judge_counts);
        }
        if(flag==6 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("x_6.87");
            pose.pose.position.x=6.87;
            pose.pose.position.y=-31.6;
            pose.pose.position.z=hight;
            last_request =ros::Time::now();
            flag = 7;
        }
        if(flag==7 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("y-55.27");           
            double distance_to_target = fabs(-55.27 - local_pos.pose.position.x);
            double speed_y = -limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed));
            pose.pose.position.y = local_pos.pose.position.y + speed_y * (ros::Time::now() - last_request).toSec();
            pose.pose.position.x=6.87;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.y-(-55.27))<0.5){
                last_request =ros::Time::now();
                flag = 8;}
        }
        if(flag==8 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("x_8.74");
            pose.pose.position.x=8.74;
            pose.pose.position.y=-55.27;
            pose.pose.position.z=hight;
            last_request =ros::Time::now();
            flag = 9;
        }
        if(flag==9 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("1");
            pose.pose.position.x=9.75;
            pose.pose.position.y=-55.12;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 10;
        }
        if(flag==10 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("2");
            pose.pose.position.x=11.72;
            pose.pose.position.y=-54.80;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 11;
        }
        if(flag==11 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("3");
            pose.pose.position.x=12.03;
            pose.pose.position.y=-56.78;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 12;
        }
        if(flag==12 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("4");
            pose.pose.position.x=10.06;
            pose.pose.position.y=-57.09;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 13;
        }
        if(flag==13 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("5");
            pose.pose.position.x=8.08;
            pose.pose.position.y=-57.38;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 14;
        }
        if(flag==14 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("6");
            pose.pose.position.x=6.11;
            pose.pose.position.y=-57.70;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 15;
        }
        if(flag==15 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("7");
            pose.pose.position.x=5.80;
            pose.pose.position.y=-55.72;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 16;
        }
        if(flag==16 && ros::Time::now()-last_request > ros::Duration(5.0)){
              ROS_INFO("8");
            pose.pose.position.x=7.78;
            pose.pose.position.y=-55.41;
            pose.pose.position.z=hight_dec;
            last_request =ros::Time::now();
            flag = 17;
        }


        if(flag==17 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("9");
            pose.pose.position.x=8.76;
            pose.pose.position.y=-55.27;
            pose.pose.position.z=hight;
            last_request =ros::Time::now();
            flag = 18;
        }
        if(flag==18 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("back1");
            pose.pose.position.x=6.87;
            pose.pose.position.y=-55.27;
            pose.pose.position.z=hight;
            last_request =ros::Time::now();
            flag = 19;
        }
        if(flag==19 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("point1_y");
            double distance_to_target = fabs(-31.6 - local_pos.pose.position.y);
            double speed_y = limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed));
            pose.pose.position.y = local_pos.pose.position.y + speed_y * (ros::Time::now() - last_request).toSec();
            pose.pose.position.x=6.87;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.y-(-31.6))<0.5){
                last_request =ros::Time::now();
                flag = 20;
            }
        }
        if(flag==20 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("back3");
            pose.pose.position.x=5;
            pose.pose.position.y=-31.6;
            pose.pose.position.z=hight;
            last_request =ros::Time::now();
            flag = 21;
        }
        if(flag==21 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("point1_x_2.5");
            double distance_to_target = fabs(2.5 - local_pos.pose.position.x);
            double speed_x = -limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed));
            pose.pose.position.x = local_pos.pose.position.x + speed_x * (ros::Time::now() - last_request).toSec();
            pose.pose.position.y=-31.6;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.x-2.5)<0.5){
                last_request =ros::Time::now();
                flag = 22;
            }
        }
        if(flag==22 && ros::Time::now()-last_request > ros::Duration(5.0)){
            ROS_INFO("point1_y");
            double distance_to_target = fabs(0 - local_pos.pose.position.y);
            double speed_y = limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed));
            pose.pose.position.y = local_pos.pose.position.y + speed_y * (ros::Time::now() - last_request).toSec();
            pose.pose.position.x=2.5;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.y-(0))<0.5){
                last_request =ros::Time::now();
                flag = 23;
            }
        }
        if(flag==23 && ros::Time::now()-last_request > ros::Duration(5.0)){
            double distance_to_target = fabs(0 - local_pos.pose.position.x);
            double speed_x = -(limit_speed(distance_to_target / (ros::Time::now() - last_request).toSec(), static_cast<double>(max_speed)));
            pose.pose.position.x = local_pos.pose.position.x + speed_x * (ros::Time::now() - last_request).toSec();
            pose.pose.position.y=0;
            pose.pose.position.z=hight;
            if(abs(local_pos.pose.position.x-0)<0.5){
                last_request =ros::Time::now();
                flag = 24;}
        }
        if((flag == 24)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
        {
                        offb_set_mode.request.custom_mode = "AUTO.LAND";
            if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > 
            ros::Duration(5.0))){
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                ROS_INFO("AUTO.LAND enabled");
                }
            last_request = ros::Time::now();
            }
        }
        if(S_to_V == 0)local_pos_pub.publish(pose);
        if(S_to_V == 1)velocity_publisher.publish(velocity_command);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

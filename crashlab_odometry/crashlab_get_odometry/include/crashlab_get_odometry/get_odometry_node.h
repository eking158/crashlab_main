#ifndef ODOMETRY_NODE_H
#define ODOMETRY_NODE_H
#include <geometry_msgs/Twist.h>
#include <crashlab_motor_msgs/control_motor.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#define math_pi 3.141592

//Text_Input
void Text_Input(void);
int PWM_range;
int PWM_frequency;
int PWM_limit;
double Control_cycle;
double Wheel_base;
double Wheel_radius;
double Robot_radius;
int Encoder_resolution;
double Wheel_round;
double Robot_round;

ros::Publisher pub_odom;
ros::Subscriber sub_rpm;

//msgs
crashlab_motor_msgs::control_motor rpm_msgs;
geometry_msgs::Pose2D Pose2D_msgs;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;

//변수
float dt;
float velR = 0.0, velL = 0.0;

//time
ros::Time current_time, last_time;

//Callback
void GetRPMCallback(const crashlab_motor_msgs::control_motor &msg);

//Functions
void CalcAblePosition();

#endif // ODOMETRY_NODE_H

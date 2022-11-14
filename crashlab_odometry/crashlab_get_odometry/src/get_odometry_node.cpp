/*
 * motor_control_node.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <crashlab_get_odometry/get_odometry_node.h>
#include <fstream>
#include <cmath>
#include <ros/package.h>
#include <string>
#include <vector>
#include <utility>
#include <set>
#include <stdio.h>

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/crashlab_main/crashlab_settings/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Wheel_base = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GetRPMCallback(const crashlab_motor_msgs::control_motor &msg)
{
    velL = msg.motor1;
    velR = msg.motor2;
}

void CalcAblePosition()
{
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;
    
    float linear_vel = (velL + velR) / 2;
    float angular_vel = (velR - velL) / Wheel_base;

    float distance_delta = linear_vel * 0.1;
    float angular_delta = angular_vel * 0.1;

    Pose2D_msgs.x = Pose2D_msgs.x + distance_delta * cos(Pose2D_msgs.theta + angular_delta / 2);
    Pose2D_msgs.y = Pose2D_msgs.y + distance_delta * sin(Pose2D_msgs.theta + angular_delta / 2);

    Pose2D_msgs.theta += angular_delta;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Pose2D_msgs.theta);

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = Pose2D_msgs.x;
    odom_trans.transform.translation.y = Pose2D_msgs.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = Pose2D_msgs.x;
    odom.pose.pose.position.y = Pose2D_msgs.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x= linear_vel;
    odom.twist.twist.linear.y=0.0;
    odom.twist.twist.angular.z = angular_vel;

    /*
    if(Pose2D_msgs.theta < 0)
    {
        Pose2D_msgs.theta = Pose2D_msgs.theta + 2 * math_pi;
    }
    else if(Pose2D_msgs.theta > 2 * math_pi)
    {
        Pose2D_msgs.theta = Pose2D_msgs.theta - 2 * math_pi;
    }
    if(Pose2D_msgs.theta > math_pi)
    {
        Pose2D_msgs.theta = Pose2D_msgs.theta - 2 * math_pi;
    }
    */
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "get_odometry_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(Control_cycle);

    Text_Input();

    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    sub_rpm = nh.subscribe("/crashlab/rpm", 10, GetRPMCallback);
    tf::TransformBroadcaster odom_broadcaster;


    Pose2D_msgs.x = 0.0;
    Pose2D_msgs.y = 0.0;
    Pose2D_msgs.theta = 0.0;
    
    while (ros::ok())
    {
        CalcAblePosition();
        odom_broadcaster.sendTransform(odom_trans);
        pub_odom.publish(odom);
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

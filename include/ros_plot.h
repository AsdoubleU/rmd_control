#ifndef ROSPLOT_H
#define ROSPLOT_H

#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

#define RAD2DEG 57.2957914

ros::Publisher p_angle;
ros::Publisher p_angular_velocity;
ros::Publisher p_torque;
ros::Publisher p_reference;

std_msgs::Float64 m_angle;
std_msgs::Float64 m_angular_velocity;
std_msgs::Float64 m_torque;
std_msgs::Float64 m_reference;


#endif 
#ifndef ROSPLOT_H
#define ROSPLOT_H

#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

#define RAD2DEG 57.2957914

ros::Publisher p_angle[24];
ros::Publisher p_angular_velocity[24];
ros::Publisher p_torque[24];
ros::Publisher p_reference;

std_msgs::Float64 m_angle[24];
std_msgs::Float64 m_angular_velocity[24];
std_msgs::Float64 m_torque[24];
std_msgs::Float64 m_reference;


#endif 
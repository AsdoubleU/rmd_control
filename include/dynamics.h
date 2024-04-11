#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "motor_controller.h"


#include <eigen3/Eigen/Dense>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

//FILE *Experiment_data;

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;


#define DEG2RAD			0.017453292519943
#define RAD2DEG			57.295779513082323
#define G						9.81
#define PI					3.1415

#define NUM_OF_JOINTS   12

#define inner_dt 0.002


typedef struct Gain_type_
{
    VectorXd old_gain_value         = VectorXd::Zero(12);
    VectorXd new_gain_value         = VectorXd::Zero(12);
    VectorXd real_gain_value        = VectorXd::Zero(12);
    VectorXd callback_gain_value    = VectorXd::Zero(12);    
    VectorXd delta_gain_value      = VectorXd::Zero(12);

    // float old_gain_value{0}, new_gain_value{0}, real_gain_value{0}, callback_gain_value{0}, delta_gain_value{0};
    int count{0}, max_count{2000};
    bool update_flag{false};
} Gain_type;

typedef struct Switch_value_
{   
    VectorXd pre_torque = VectorXd::Zero(12);
    VectorXd pre_th = VectorXd::Zero(12);
    VectorXd pre_th_dot = VectorXd::Zero(12);

    int change_count = 0, changetime = 700;
    bool update_flag{false};

}switch_value;

#endif // DYNAMICS_H
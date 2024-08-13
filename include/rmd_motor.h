#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <linux/types.h>
#include <math.h>
#include "rmd_can.h"
#include "rt_utils.h"

#define PI 3.141592
#define RAD2COMMAND 1031.32

class rmd_motor
{
public:
    rmd_motor();

    bool    motor_run_flag{false};

    unsigned char reference_data[8];
    unsigned char feedback_data[8];

    int     count;
    int     count_92;
    int     count_A1;
    int     unknown_value;

    bool    is_v3;

    int     actuator_gear_ratio;
    int     actuator_direction;
    float   actuator_torque_limit;
    double  initial_theta;
    float   joint_initial_position;
    float   filtered_data;
    float   filtered_torque;
    bool    initialize_position{true};
    float   torque_to_data;
    double  data_to_radian;
    int     direction;
    float   pre_pos;

    void    EnableMotor();
    void    DisableMotor();
    void    StopMotor();
    void    EnableFilter();
    void    UpdateRxData(void);
    void    UpdateRxData2(void);
    void    SetTorqueData(float);
    void    SetVelocityData(float);
    void    SetPositionData(float, float);
    void    SetGainDatas(float);
    void    SetInitialTheta() { initial_theta = joint_theta; }
    float   GetTheta();
    float   GetThetaV3();
    float   GetThetaDot();
    float   GetTorque();

private:
    float   joint_velocity;
    float   joint_theta;
    float   joint_torque;
    float   joint_torque_old;
    float   torque_data_old;
    float   joint_temperature;

    float   motor_theta_last;
    float   temp_encoder_last;
    float   count_overflow;

    float   joint_theta_92;
    float   joint_theta_offset_92;

    bool    is_theta_initialize = false;
};

#endif // RMD_MOTOR_H

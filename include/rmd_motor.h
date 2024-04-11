#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <linux/types.h>
#include <math.h>
#include "rmd_can.h"
#include "rt_utils.h"
#include "dynamics.h"

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
    float   joint_initial_position;
    bool    initialize_position{true};
    float   torque_to_data;
    double   data_to_radian;

    void    EnableMotor();
    void    DisableMotor();
    void    EnableFilter();
    void    UpdateRxData(void);
    void    UpdateRxData2(void);
    void    SetTorqueData(float);
    float   GetTheta();
    float   GetThetaV3();
    float   GetThetaDot();
    float   GetTorque();

private:
    float   joint_velocity;
    float   joint_theta;
    float   joint_torque;
    float   joint_temperature;

    float   motor_theta_last;

    float   joint_theta_92;
    float   joint_theta_offset_92;
};

#endif // RMD_MOTOR_H

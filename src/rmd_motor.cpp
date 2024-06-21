#include "rmd_motor.h"
#include "motor_controller.h"
#include <iostream>


rmd_motor::rmd_motor()
{

}

void rmd_motor::UpdateRxData(void) {

    joint_temperature = (int)(feedback_data[1]);
    
    int temp_torque = (int)(feedback_data[2] | (feedback_data[3]<<8));
    joint_torque = temp_torque / torque_to_data;

    if(joint_torque > 2500) { joint_torque = -(joint_torque - 2675); }
    joint_torque = direction*joint_torque;
    
    int temp_speed = (int)(feedback_data[4] | (feedback_data[5]<<8));
    if(temp_speed > 30000) temp_speed -= 65535;
    if(is_v3) joint_velocity = 0.028 * temp_speed * actuator_direction;
    else joint_velocity = 0.028 * temp_speed * actuator_direction / actuator_gear_ratio;

    int temp_encoder = (int)(feedback_data[6] | (feedback_data[7]<<8));

    float temp_theta = temp_encoder / data_to_radian;
    float incremental_theta{0};
    
    if(initialize_position){
        joint_theta = joint_initial_position;
        initialize_position = false;
    }
    else{
        if(temp_encoder > 15383 && temp_encoder_last < 1000) count_overflow--;
        else if (temp_encoder < 1000 && temp_encoder_last > 15383) count_overflow++;
        temp_theta = temp_encoder + count_overflow * 16383;
    }  

    if (temp_theta > 589788) count_overflow = 0;
    else if (temp_theta < -589788) count_overflow = 0;

    temp_encoder_last = temp_encoder;
    joint_theta = temp_theta * data_to_radian;
   
}

void rmd_motor::UpdateRxData2(void) {
    joint_temperature = (int)(feedback_data[1]);
    
    int temp_torque = (int)(feedback_data[2] | (feedback_data[3]<<8));
    joint_torque = temp_torque / torque_to_data;
    
    int temp_speed = (int)(feedback_data[4] | (feedback_data[5]<<8));
    if(temp_speed > 30000) temp_speed -= 65535;
    if(is_v3) joint_velocity = 0.01 * temp_speed * actuator_direction;
    else joint_velocity = 0.01 * temp_speed * actuator_direction / actuator_gear_ratio;

    int temp_encoder = (int)(feedback_data[6] | (feedback_data[7]<<8));
    
        float temp_theta = temp_encoder / 57.29577951;
        float incremental_theta{0};
        if(initialize_position){
        joint_theta = joint_initial_position;
        motor_theta_last = temp_theta;
        initialize_position = false;
        }
        else{
        incremental_theta = temp_theta - motor_theta_last;
        
        motor_theta_last = temp_theta;
        }    
        if (incremental_theta > 1143) incremental_theta = 0.0174532925;
        else if (incremental_theta < -1143) incremental_theta = -0.0174532925;
    joint_theta += incremental_theta * actuator_direction;

}


float rmd_motor::GetTheta() 
{
    return joint_theta;
}


float rmd_motor::GetThetaDot() 
{
    return joint_velocity;
}


float rmd_motor::GetTorque()
{
    return joint_torque;
}


void rmd_motor::SetTorqueData(float tau)
{
    // if(tau > actuator_torque_limit) tau = actuator_torque_limit;
    // else if(tau < -1 * actuator_torque_limit) tau = -1 * actuator_torque_limit;
    
    long param = actuator_direction * torque_to_data * tau;
    reference_data[0] = 0xA1 & 0xFF;
    reference_data[1] = 0x00 & 0xFF;
    reference_data[2] = 0x00 & 0xFF;
    reference_data[3] = 0x00 & 0xFF;
    reference_data[4] = (param     ) & 0xFF;
    reference_data[5] = (param >> 8) & 0xFF;
    reference_data[6] = 0x00 & 0xFF;
    reference_data[7] = 0x00 & 0xFF;

    if(tau >= 0) { direction = 1; } else { direction = -1; }

}

void rmd_motor::SetVelocityData(float vel)
{

    int32_t param = static_cast<int32_t>(vel*100.0);
    reference_data[0] = 0xA2 & 0xFF;
    reference_data[1] = 0x00 & 0xFF;
    reference_data[2] = 0x00 & 0xFF;
    reference_data[3] = 0x00 & 0xFF;
    reference_data[4] = (param     ) & 0xFF;
    reference_data[5] = (param >> 8) & 0xFF;
    reference_data[6] = (param >> 16) & 0xFF;
    reference_data[7] = (param >> 24) & 0xFF;

}

void rmd_motor::SetPositionData(float pos)
{

    int32_t param = static_cast<int32_t>(pos*100.0);
    reference_data[0] = 0xA3 & 0xFF;
    reference_data[1] = 0x00 & 0xFF;
    reference_data[2] = 0x00 & 0xFF;
    reference_data[3] = 0x00 & 0xFF;
    reference_data[4] = (param     ) & 0xFF;
    reference_data[5] = (param >> 8) & 0xFF;
    reference_data[6] = (param >> 16) & 0xFF;
    reference_data[7] = (param >> 24) & 0xFF;

}

// ID 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00
void rmd_motor::EnableFilter()
{
    reference_data[0] = 0x20 & 0xFF;
    reference_data[1] = 0x02 & 0xFF;
    reference_data[2] = 0x00 & 0xFF;
    reference_data[3] = 0x00 & 0xFF;
    reference_data[4] = 0x01 & 0xFF;
    reference_data[5] = 0x00 & 0xFF;
    reference_data[6] = 0x00 & 0xFF;
    reference_data[7] = 0x00 & 0xFF;
}


void rmd_motor::EnableMotor()
{
    motor_run_flag = true;
    initialize_position = true;

    reference_data[0] = 0x88 & 0xFF;
    reference_data[1] = 0x00 & 0xFF;
    reference_data[2] = 0x00 & 0xFF;
    reference_data[3] = 0x00 & 0xFF;
    reference_data[4] = 0x00 & 0xFF;
    reference_data[5] = 0x00 & 0xFF;
    reference_data[6] = 0x00 & 0xFF;
    reference_data[7] = 0x00 & 0xFF;
}

void rmd_motor::SetGainDatas(float gain)
{
    reference_data[0] = 0x32 & 0xFF;
    reference_data[1] = 0x00 & 0xFF;
    reference_data[2] = 0x00 & 0xFF;
    reference_data[3] = 0x00 & 0xFF;
    reference_data[4] = 0x00 & 0xFF;
    reference_data[5] = 0x00 & 0xFF;
    reference_data[6] = 0x00 & 0xFF;
    reference_data[7] = 0x00 & 0xFF;
}

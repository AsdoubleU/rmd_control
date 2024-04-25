#include "motor_controller.h"

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[NUM_OF_RMD];

Motor_Controller::Motor_Controller()
{
  _DEV_MC[0].actuator_direction =  1.;
  _DEV_MC[0].actuator_gear_ratio = 1.;   
  _DEV_MC[0].joint_initial_position = 0;     
  _DEV_MC[0].torque_to_data = 225.;    
  _DEV_MC[0].actuator_torque_limit = 3.5*2;     
  _DEV_MC[0].data_to_radian = DATA_TO_RADIAN;
  _DEV_MC[0].is_v3 = true; 

  // _DEV_MC[1].actuator_direction =  -1;     _DEV_MC[1].actuator_gear_ratio = 9;   _DEV_MC[1].joint_initial_position = -0.31420;     _DEV_MC[1].torque_to_data = 24.5;     _DEV_MC[1].actuator_torque_limit = 18*2;       _DEV_MC[1].data_to_radian = DATA_TO_RADIAN_V3;      _DEV_MC[1].is_v3 = true;  //-0.22959;
  // _DEV_MC[2].actuator_direction =  1;     _DEV_MC[2].actuator_gear_ratio = 36;  _DEV_MC[2].joint_initial_position = 0.31455;     _DEV_MC[2].torque_to_data = 24.5;     _DEV_MC[2].actuator_torque_limit = 9*2;       _DEV_MC[2].data_to_radian = DATA_TO_RADIAN_V3_20b;      _DEV_MC[2].is_v3 = true;
  // _DEV_MC[3].actuator_direction =  -1;     _DEV_MC[3].actuator_gear_ratio = 36;  _DEV_MC[3].joint_initial_position = -0.897273;     _DEV_MC[3].torque_to_data = 24.5;     _DEV_MC[3].actuator_torque_limit = 18*2;      _DEV_MC[3].data_to_radian = DATA_TO_RADIAN_V3_20b;      _DEV_MC[3].is_v3 = true; //
  // _DEV_MC[4].actuator_direction = -1;     _DEV_MC[4].actuator_gear_ratio = 36;  _DEV_MC[4].joint_initial_position = -1.88496;     _DEV_MC[4].torque_to_data = 24.5;     _DEV_MC[4].actuator_torque_limit = 13*2;      _DEV_MC[4].data_to_radian = DATA_TO_RADIAN_V3_20b;      _DEV_MC[4].is_v3 = true; 
  // _DEV_MC[5].actuator_direction =  -1;     _DEV_MC[5].actuator_gear_ratio = 8;   _DEV_MC[5].joint_initial_position = 0;     _DEV_MC[5].torque_to_data = 140;    _DEV_MC[5].actuator_torque_limit = 5*2;       _DEV_MC[5].data_to_radian = DATA_TO_RADIAN_V3;      _DEV_MC[5].is_v3 = true;
  

  

  // _DEV_MC[6].actuator_direction =  1;     _DEV_MC[6].actuator_gear_ratio = 9;   _DEV_MC[6].joint_initial_position = 0.0;     _DEV_MC[6].torque_to_data = 90;     _DEV_MC[6].actuator_torque_limit = 5;       _DEV_MC[6].data_to_radian = DATA_TO_RADIAN_V3;      _DEV_MC[6].is_v3 = true;    //direction =  1; 9, 220
  // _DEV_MC[7].actuator_direction =  1;     _DEV_MC[7].actuator_gear_ratio = 9;   _DEV_MC[7].joint_initial_position = -0.325;     _DEV_MC[7].torque_to_data = 90;     _DEV_MC[7].actuator_torque_limit = 9;       _DEV_MC[7].data_to_radian = DATA_TO_RADIAN_V3;      _DEV_MC[7].is_v3 = true;    //direction =  1;
  // _DEV_MC[8].actuator_direction =  1;     _DEV_MC[8].actuator_gear_ratio = 9;   _DEV_MC[8].joint_initial_position = -0.127;     _DEV_MC[8].torque_to_data = 90;     _DEV_MC[8].actuator_torque_limit = 9;       _DEV_MC[8].data_to_radian = DATA_TO_RADIAN_V3;      _DEV_MC[8].is_v3 = true;    //direction =  1;
  // _DEV_MC[9].actuator_direction =  -1;     _DEV_MC[9].actuator_gear_ratio =36;   _DEV_MC[9].joint_initial_position = 0.00213;     _DEV_MC[9].torque_to_data = 35;     _DEV_MC[9].actuator_torque_limit = 18;       _DEV_MC[9].data_to_radian = DATA_TO_RADIAN_V3;      _DEV_MC[9].is_v3 = true;    //direction = -1;
  // _DEV_MC[10].actuator_direction = -1;    _DEV_MC[10].actuator_gear_ratio = 6;  _DEV_MC[10].joint_initial_position = 1.312;    _DEV_MC[10].torque_to_data = 85;    _DEV_MC[10].actuator_torque_limit = 9;      _DEV_MC[10].data_to_radian = DATA_TO_RADIAN_V3;     _DEV_MC[10].is_v3 = false;    //direction = -1;
  // _DEV_MC[11].actuator_direction = 1;    _DEV_MC[11].actuator_gear_ratio = 9;  _DEV_MC[11].joint_initial_position = 0.04196;    _DEV_MC[11].torque_to_data = 90;    _DEV_MC[11].actuator_torque_limit = 9;      _DEV_MC[11].data_to_radian = DATA_TO_RADIAN_V3;     _DEV_MC[11].is_v3 = true;    //direction =  1;
}

void Motor_Controller::EnableMotor()
{
  // SPI 1.0  CH A
  sharedData->rmd_motor_run_flag[0] = true;  
  sharedData->rmd_motor_run_flag[1] = false;  
  sharedData->rmd_motor_run_flag[2] = false; 
  //          CH B
  sharedData->rmd_motor_run_flag[3] = false;     
  sharedData->rmd_motor_run_flag[4] = false;     
  sharedData->rmd_motor_run_flag[5] = false;     
  // SPI 1.1  CH c
  sharedData->rmd_motor_run_flag[6] = false;     
  sharedData->rmd_motor_run_flag[7] = false;     
  sharedData->rmd_motor_run_flag[8] = false;     
  //          CH D
  sharedData->rmd_motor_run_flag[9] = false;     
  sharedData->rmd_motor_run_flag[10] = false;    
  sharedData->rmd_motor_run_flag[11] = false;    
  for(uint8_t i=0; i<NUM_OF_RMD; i++) 
  {
    _DEV_MC[i].EnableMotor();
  }
}


VectorXd Motor_Controller::GetJointTheta(){
  for(uint8_t i=0; i<NUM_OF_RMD; i++)
  {
    th_joint[i] = _DEV_MC[i].GetTheta();
  }
  return th_joint;
}


VectorXd Motor_Controller::GetThetaDot()
{
  for(uint8_t i=0; i<NUM_OF_RMD; i++) 
  {
    th_dot[i] = _DEV_MC[i].GetThetaDot();

  }  
  return th_dot;
}

// Simple Moving Average filtered Joint Velocity
VectorXd Motor_Controller::GetThetaDotSMAF()
{
  for(uint8_t i=0; i<NUM_OF_RMD; i++) 
  {
    th_dot[i] = _DEV_MC[i].GetThetaDot();
  }
  sma << sma.block<FILTER_WINDOW_SIZE-1, NUM_OF_RMD>(1, 0), 
      th_dot[0], th_dot[1], th_dot[2], th_dot[3], th_dot[4], th_dot[5], 
      th_dot[6], th_dot[7], th_dot[8], th_dot[9], th_dot[10], th_dot[11];

  th_dot_sma_filtered = sma.colwise().mean();

  return th_dot_sma_filtered;
}

VectorXd Motor_Controller::GetTorque()
{
  VectorXd tau(NUM_OF_RMD); 
  for(uint8_t i=0; i<NUM_OF_RMD; i++)
  {
    tau[i] = _DEV_MC[i].GetTorque();
  }
  return tau;
}


void Motor_Controller::SetTorque(VectorXd tau)
{
  for(uint8_t i=0; i<NUM_OF_RMD; i++) 
  {
    _DEV_MC[i].SetTorqueData(tau[i]);
  }
}

void Motor_Controller::SetTorque(float tau)
{
  for(uint8_t i=0; i<NUM_OF_RMD; i++) 
  {
    _DEV_MC[i].SetTorqueData(tau);
  }
}


void Motor_Controller::EnableFilter(){
  for(uint8_t i=0; i<NUM_OF_RMD; i++) 
  {
    _DEV_MC[i].EnableFilter();
  }
}
#include "motor_controller.h"

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[NUM_OF_RMD];

Motor_Controller::Motor_Controller()
{
  _DEV_MC[0].actuator_direction =  1.;  _DEV_MC[0].actuator_gear_ratio = 1.;      _DEV_MC[0].joint_initial_position = 0; 
  _DEV_MC[0].torque_to_data = 24.5;     _DEV_MC[0].actuator_torque_limit = 3.5*2; _DEV_MC[0].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[1].actuator_direction =  1.;  _DEV_MC[1].actuator_gear_ratio = 1.;      _DEV_MC[1].joint_initial_position = 0;     
  _DEV_MC[1].torque_to_data = 24.5;     _DEV_MC[1].actuator_torque_limit = 3.5*2; _DEV_MC[1].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[2].actuator_direction =  1.;  _DEV_MC[2].actuator_gear_ratio = 1.;      _DEV_MC[2].joint_initial_position = 0;     
  _DEV_MC[2].torque_to_data = 24.5;     _DEV_MC[2].actuator_torque_limit = 3.5*2; _DEV_MC[2].data_to_radian = DATA_TO_RADIAN; 
  
  _DEV_MC[3].actuator_direction =  1.;  _DEV_MC[3].actuator_gear_ratio = 1.;      _DEV_MC[3].joint_initial_position = 0;     
  _DEV_MC[3].torque_to_data = 24.5;     _DEV_MC[3].actuator_torque_limit = 3.5*2; _DEV_MC[3].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[4].actuator_direction =  1.;  _DEV_MC[4].actuator_gear_ratio = 1.;      _DEV_MC[4].joint_initial_position = 0;     
  _DEV_MC[4].torque_to_data = 24.5;     _DEV_MC[4].actuator_torque_limit = 3.5*2; _DEV_MC[4].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[5].actuator_direction =  1.;  _DEV_MC[5].actuator_gear_ratio = 1.;      _DEV_MC[5].joint_initial_position = 0;     
  _DEV_MC[5].torque_to_data = 24.5;     _DEV_MC[5].actuator_torque_limit = 3.5*2; _DEV_MC[5].data_to_radian = DATA_TO_RADIAN; 
  
  _DEV_MC[6].actuator_direction =  1.;  _DEV_MC[6].actuator_gear_ratio = 1.;      _DEV_MC[6].joint_initial_position = 0;     
  _DEV_MC[6].torque_to_data = 24.5;     _DEV_MC[6].actuator_torque_limit = 3.5*2; _DEV_MC[6].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[7].actuator_direction =  1.;  _DEV_MC[7].actuator_gear_ratio = 1.;      _DEV_MC[7].joint_initial_position = 0;     
  _DEV_MC[7].torque_to_data = 24.5;     _DEV_MC[7].actuator_torque_limit = 3.5*2; _DEV_MC[7].data_to_radian = DATA_TO_RADIAN;

  _DEV_MC[8].actuator_direction =  1.;  _DEV_MC[8].actuator_gear_ratio = 1.;      _DEV_MC[8].joint_initial_position = 0;     
  _DEV_MC[8].torque_to_data = 24.5;     _DEV_MC[8].actuator_torque_limit = 3.5*2; _DEV_MC[8].data_to_radian = DATA_TO_RADIAN;

  _DEV_MC[9].actuator_direction =  1.;  _DEV_MC[9].actuator_gear_ratio = 1.;      _DEV_MC[9].joint_initial_position = 0;     
  _DEV_MC[9].torque_to_data = 24.5;     _DEV_MC[9].actuator_torque_limit = 3.5*2; _DEV_MC[9].data_to_radian = DATA_TO_RADIAN;

  _DEV_MC[10].actuator_direction =  1.; _DEV_MC[10].actuator_gear_ratio = 1.;      _DEV_MC[10].joint_initial_position = 0;     
  _DEV_MC[10].torque_to_data = 24.5;    _DEV_MC[10].actuator_torque_limit = 3.5*2; _DEV_MC[10].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[11].actuator_direction =  1.; _DEV_MC[11].actuator_gear_ratio = 1.;      _DEV_MC[11].joint_initial_position = 0;     
  _DEV_MC[11].torque_to_data = 24.5;    _DEV_MC[11].actuator_torque_limit = 3.5*2; _DEV_MC[11].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[12].actuator_direction =  1.; _DEV_MC[12].actuator_gear_ratio = 1.;      _DEV_MC[12].joint_initial_position = 0;  
  _DEV_MC[12].torque_to_data = 24.5;    _DEV_MC[12].actuator_torque_limit = 3.5*2; _DEV_MC[12].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[13].actuator_direction =  1.; _DEV_MC[13].actuator_gear_ratio = 1.;      _DEV_MC[13].joint_initial_position = 0;   
  _DEV_MC[13].torque_to_data = 24.5;    _DEV_MC[13].actuator_torque_limit = 3.5*2; _DEV_MC[13].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[14].actuator_direction =  1.; _DEV_MC[14].actuator_gear_ratio = 1.;      _DEV_MC[14].joint_initial_position = 0;     
  _DEV_MC[14].torque_to_data = 24.5;    _DEV_MC[14].actuator_torque_limit = 3.5*2; _DEV_MC[14].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[15].actuator_direction =  1.; _DEV_MC[15].actuator_gear_ratio = 1.;      _DEV_MC[15].joint_initial_position = 0;     
  _DEV_MC[15].torque_to_data = 24.5;    _DEV_MC[15].actuator_torque_limit = 3.5*2; _DEV_MC[15].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[16].actuator_direction =  1.; _DEV_MC[16].actuator_gear_ratio = 1.;      _DEV_MC[16].joint_initial_position = 0;     
  _DEV_MC[16].torque_to_data = 24.5;    _DEV_MC[16].actuator_torque_limit = 3.5*2; _DEV_MC[16].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[17].actuator_direction =  1.; _DEV_MC[17].actuator_gear_ratio = 1.;      _DEV_MC[17].joint_initial_position = 0;  
  _DEV_MC[17].torque_to_data = 24.5;    _DEV_MC[17].actuator_torque_limit = 3.5*2; _DEV_MC[17].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[18].actuator_direction =  1.; _DEV_MC[18].actuator_gear_ratio = 1.;      _DEV_MC[18].joint_initial_position = 0;     
  _DEV_MC[18].torque_to_data = 24.5;    _DEV_MC[18].actuator_torque_limit = 3.5*2; _DEV_MC[18].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[19].actuator_direction =  1.; _DEV_MC[19].actuator_gear_ratio = 1.;      _DEV_MC[19].joint_initial_position = 0;     
  _DEV_MC[19].torque_to_data = 24.5;    _DEV_MC[19].actuator_torque_limit = 3.5*2; _DEV_MC[19].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[20].actuator_direction =  1.; _DEV_MC[20].actuator_gear_ratio = 1.;      _DEV_MC[20].joint_initial_position = 0;     
  _DEV_MC[20].torque_to_data = 24.5;    _DEV_MC[20].actuator_torque_limit = 3.5*2; _DEV_MC[20].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[21].actuator_direction =  1.; _DEV_MC[21].actuator_gear_ratio = 1.;      _DEV_MC[21].joint_initial_position = 0;     
  _DEV_MC[21].torque_to_data = 24.5;    _DEV_MC[21].actuator_torque_limit = 3.5*2; _DEV_MC[21].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[22].actuator_direction =  1.; _DEV_MC[22].actuator_gear_ratio = 1.;      _DEV_MC[22].joint_initial_position = 0;     
  _DEV_MC[22].torque_to_data = 24.5;    _DEV_MC[22].actuator_torque_limit = 3.5*2; _DEV_MC[22].data_to_radian = DATA_TO_RADIAN; 

  _DEV_MC[23].actuator_direction =  1.; _DEV_MC[23].actuator_gear_ratio = 1.;      _DEV_MC[23].joint_initial_position = 0;   
  _DEV_MC[23].torque_to_data = 24.5;    _DEV_MC[23].actuator_torque_limit = 3.5*2; _DEV_MC[23].data_to_radian = DATA_TO_RADIAN; 

  for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].is_v3 = true; }
  
}

void Motor_Controller::EnableMotor()
{
  // SPI 1.0  CH A
  sharedData->rmd_motor_run_flag[0] = true;  
  sharedData->rmd_motor_run_flag[1] = true;  
  sharedData->rmd_motor_run_flag[2] = true; 
  sharedData->rmd_motor_run_flag[3] = true; 
  sharedData->rmd_motor_run_flag[4] = true;     
  sharedData->rmd_motor_run_flag[5] = true;     
  // SPI 1.1  CH B
  sharedData->rmd_motor_run_flag[6] = true;     
  sharedData->rmd_motor_run_flag[7] = true;
  sharedData->rmd_motor_run_flag[8] = true;
  sharedData->rmd_motor_run_flag[9] = true;     
  sharedData->rmd_motor_run_flag[10] = true;    
  sharedData->rmd_motor_run_flag[11] = true;    
  // SPI 1.1  CH C
  sharedData->rmd_motor_run_flag[12] = true; 
  sharedData->rmd_motor_run_flag[13] = true; 
  sharedData->rmd_motor_run_flag[14] = true; 
  sharedData->rmd_motor_run_flag[15] = true; 
  sharedData->rmd_motor_run_flag[16] = true; 
  sharedData->rmd_motor_run_flag[17] = true; 
  //          CH D
  sharedData->rmd_motor_run_flag[18] = true; 
  sharedData->rmd_motor_run_flag[19] = true; 
  sharedData->rmd_motor_run_flag[20] = true; 
  sharedData->rmd_motor_run_flag[21] = true; 
  sharedData->rmd_motor_run_flag[22] = true; 
  sharedData->rmd_motor_run_flag[23] = true; 


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
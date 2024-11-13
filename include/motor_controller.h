#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "shared_memory.h"

#define   DATA_TO_RADIAN 1.065329236946157e-5
#define   FILTER_WINDOW_SIZE  20
#define   NUM_OF_ACTUATORS    24
#define   NUM_OF_RMD          24

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

extern pRBCORE_SHM sharedData;
extern rmd_motor _DEV_MC[NUM_OF_RMD];

class Motor_Controller{

public:   
  int count;
  bool first_loop = true;

  VectorXd th_joint = VectorXd::Zero(NUM_OF_ACTUATORS);
  VectorXd th_dot = VectorXd::Zero(NUM_OF_ACTUATORS);
  VectorXd old_th_dot = VectorXd::Zero(NUM_OF_ACTUATORS);
  VectorXd th_dot_sma_filtered = VectorXd::Zero(NUM_OF_ACTUATORS);
  VectorXd filtered_th_dot = VectorXd::Zero(NUM_OF_ACTUATORS);
  MatrixXd sma = MatrixXd::Zero(FILTER_WINDOW_SIZE, NUM_OF_ACTUATORS);

  Motor_Controller();
  //~Motor_Controller();

  VectorXd GetTheta();
  VectorXd GetJointTheta();
  VectorXd GetThetaDot();
  VectorXd GetThetaDotSMAF();
  VectorXd GetTorque();
  void ReadTheta();    
  void SetTorque(VectorXd tau)                 { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].SetTorqueData(tau[i]); } }
  void SetTorque(float tau)                    { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].SetTorqueData(tau); } }
  void SetPosition(float max_speed, float pos) { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].SetPositionData(max_speed, pos); } }
  void EnableMotor();
  void DisableMotor();
  void StopMotor()                             { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].StopMotor(); } }
  void EnableFilter()                          { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].EnableFilter(); } }
  void SetInitialTheta()                       { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].SetInitialTheta(); } }
  void ReadGainDatas()                         { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].ReadGainDatas(); } }
  void ReadMultiturnAngle()                    { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].ReadMultiturnAngle(); } }
  void PrintGainDatas()                        { for(uint8_t i=0; i<NUM_OF_RMD; i++) { _DEV_MC[i].PrintGainDatas(); } }
};


#endif // MOTOR_CONTROLLER_H
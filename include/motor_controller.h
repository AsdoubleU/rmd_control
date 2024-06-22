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
  void SetTorque(VectorXd tau);
  void SetTorque(float tau);
  void SetPosition(VectorXd theta);  
  void EnableMotor();
  void EnableFilter();
};


#endif // MOTOR_CONTROLLER_H
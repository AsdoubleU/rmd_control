#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "shared_memory.h"

#define   DATA_TO_RADIAN_V3_20b   41721.5134018818109
#define   DATA_TO_RADIAN_V3   2607.435432674516
#define   DATA_TO_RADIAN_V2   10430.21970545193
#define   FILTER_WINDOW_SIZE  20
#define   NUM_OF_ACTUATORS    1
#define   NUM_OF_RMD          1

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

  VectorXd GetThetaX();
  VectorXd GetThetaL();
  VectorXd GetTheta();
  VectorXd GetJointTheta();
  VectorXd GetThetaDot();
  VectorXd GetThetaDotEst();
  VectorXd GetThetaDotSMAF();
  VectorXd GetTorque();
  void ReadTheta();    
  void SetTorque(VectorXd tau);
  void SetTorque(float tau);
  void SetPosition(VectorXd theta);  
  void EnableMotor();
  void EnableFilter();
  void ReadCurrentLoopPI();
};


#endif // MOTOR_CONTROLLER_H
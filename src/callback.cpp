#include "callback.h"

extern rmd_motor _DEV_MC[NUM_OF_RMD];
extern Dynamics::RobotDynamics dynamics;


Callback::Callback(){}


void Callback::SwitchMode(const std_msgs::Int32ConstPtr &msg)
{
  dynamics.SwitchMode(msg);
}

void Callback::SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  dynamics.gain_p_task_space[0] = msg->data.at(0);
  dynamics.gain_p_task_space[1] = msg->data.at(1);
  dynamics.gain_p_task_space[2] = msg->data.at(2);
  dynamics.gain_p_task_space[3] = msg->data.at(3);
  dynamics.gain_p_task_space[4] = msg->data.at(4);
  dynamics.gain_p_task_space[5] = msg->data.at(5);

}

void Callback::SwitchGainTaskSpaceD(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  dynamics.gain_d_task_space[0] = msg->data.at(0);
  dynamics.gain_d_task_space[1] = msg->data.at(1);
  dynamics.gain_d_task_space[2] = msg->data.at(2);
  dynamics.gain_d_task_space[3] = msg->data.at(3);
  dynamics.gain_d_task_space[4] = msg->data.at(4);
  dynamics.gain_d_task_space[5] = msg->data.at(5);

}


void Callback::SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  dynamics.gain_w_task_space[0] = msg->data.at(0);
  dynamics.gain_w_task_space[1] = msg->data.at(1);
  dynamics.gain_w_task_space[2] = msg->data.at(2);
}

void Callback::SwitchGainP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < NUM_OF_ACTUATORS; i++)
  {
    //dynamics.gain_p_joint_space[i] = msg -> data.at(i);
    dynamics.js_kp.callback_gain_value(i) = msg -> data.at(i);
    dynamics.js_kp.update_flag = true;
  } 
}

void Callback::SwitchGainD(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < NUM_OF_ACTUATORS; i++) 
  {
    dynamics.gain_d_joint_space[i] = msg -> data.at(i);
    // dynamics.js_kd.callback_gain_value(i) = msg -> data.at(i);
    // dynamics.js_kd.update_flag = true;

  }
}

void Callback::SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < NUM_OF_ACTUATORS; i++) 
  {
      dynamics.gain_r[i] = msg -> data.at(i);
   // dynamics.js_gp.callback_gain_value(i) = msg -> data.at(i);
   // dynamics.js_gp.update_flag = true;
  }
}

void Callback::InitializePose(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data) for(uint8_t i=0; i<NUM_OF_ACTUATORS; i++) _DEV_MC[i].initialize_position = true;
  std::cout << "Initialized Pose" << std::endl;
}

void Callback::StartRecording(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data)
  {
    dynamics.SwitchRecording();
  } 
  
}

void Callback::Pushedcallback(const std_msgs::BoolConstPtr &msg)
{
  bool is_pushed = false;
  is_pushed = msg->data;
  std_msgs::Int32Ptr dmsg= boost::make_shared<std_msgs::Int32>();
  if(is_pushed)
  { 
    dmsg->data = 7;
    dynamics.SwitchMode(dmsg);
    std::cout << "Recieve Torque 0 command" << std::endl;
  }
  

}

void Callback::SwitchGainCTCKP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  dynamics.gain_p_CTC[0] = msg->data.at(0);
  dynamics.gain_p_CTC[1] = msg->data.at(1);
  dynamics.gain_p_CTC[2] = msg->data.at(2);
  dynamics.gain_p_CTC[3] = msg->data.at(3);
  dynamics.gain_p_CTC[4] = msg->data.at(4);
  dynamics.gain_p_CTC[5] = msg->data.at(5);

}

void Callback::SwitchGainCTCKV(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  dynamics.gain_d_CTC[0] = msg->data.at(0);
  dynamics.gain_d_CTC[1] = msg->data.at(1);
  dynamics.gain_d_CTC[2] = msg->data.at(2);
  dynamics.gain_d_CTC[3] = msg->data.at(3);
  dynamics.gain_d_CTC[4] = msg->data.at(4);
  dynamics.gain_d_CTC[5] = msg->data.at(5);

}

void Callback::FrictionKv(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  dynamics.friction_Kv[0] = msg->data.at(0);
  dynamics.friction_Kv[1] = msg->data.at(1);
  dynamics.friction_Kv[2] = msg->data.at(2);
  dynamics.friction_Kv[3] = msg->data.at(3);
  dynamics.friction_Kv[4] = msg->data.at(4);
  dynamics.friction_Kv[5] = msg->data.at(5);
}

void Callback::TestTorque(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  dynamics.msg_torque[0] = msg->data.at(0);
  dynamics.msg_torque[1] = msg->data.at(1);
  dynamics.msg_torque[2] = msg->data.at(2);
  dynamics.msg_torque[3] = msg->data.at(3);
  dynamics.msg_torque[4] = msg->data.at(4);
  dynamics.msg_torque[5] = msg->data.at(5);
}

void Callback::WriteInnerGainP(const std_msgs::Float32ConstPtr &msg)
{
  dynamics.inner_gain_p = msg->data;
}

void Callback::WriteInnerGainI(const std_msgs::Float32ConstPtr &msg)
{
  dynamics.inner_gain_i = msg->data;
}
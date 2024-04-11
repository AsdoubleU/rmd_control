#include "callback.h"

extern rmd_motor _DEV_MC[NUM_OF_RMD];

Callback::Callback(){}


void Callback::InitializePose(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data) for(uint8_t i=0; i<NUM_OF_ACTUATORS; i++) _DEV_MC[i].initialize_position = true;
  std::cout << "Initialized Pose" << std::endl;
}

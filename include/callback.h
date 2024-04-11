#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include <unistd.h>
#include "spi2can.h"
#include "rt_utils.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "motor_controller.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

class Callback
{
public:
  Callback();

private:

};

#endif // CALLBACK_H
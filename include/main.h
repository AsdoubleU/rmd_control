#include "spi2can.h"
#include "rt_utils.h"
#include "motor_controller.h"
#include "callback.h"
#include "TrajectoryGenerator.h"
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#define RAD2DEG 57.2957914
#define DEG2RAD 0.01745329
#define DATAPATH "/home/sthexa/data/"

float reference;
float ddot_old;
float filter_old;
size_t mode = 0;

static void *rt_motion_thread(void *arg);
pRBCORE_SHM sharedData;
rmd_motor _DEV_MC[NUM_OF_RMD];
Motor_Controller motor_ctrl;
Callback callback;
TrajectoryGenerator traj[NUM_OF_RMD];

// FILE *REFERENCE;
// FILE *ANGLE;
// FILE *ANGULAR_VELOCITY;
// FILE *TORQUE;

ros::Publisher p_reference;
ros::Publisher p_joint_state;
ros::Publisher p_multiturn_angle;

std_msgs::Float64MultiArray m_multiturn_angle;
std_msgs::Float64 m_reference;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rmd_control");
    ros::Time::init();
    ros::Rate loop_rate(500);
    ros::NodeHandle node_handle_;

    spi2can::getInstance();

    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));

    pthread_t thread_motion;

    int thread_id_motion = generate_rt_thread(thread_motion, rt_motion_thread, "motion_thread", 3, 95, NULL);

    p_reference = node_handle_.advertise<std_msgs::Float64>("/reference/",1);
    p_joint_state = node_handle_.advertise<sensor_msgs::JointState>("/rmd_joint_states",1);
    p_multiturn_angle = node_handle_.advertise<std_msgs::Float64MultiArray>("/rmd_multiturn_angle",1);

    sensor_msgs::JointState m_joint_state;
    m_joint_state.name.resize(NUM_OF_RMD);
    m_joint_state.position.resize(NUM_OF_RMD);
    m_joint_state.velocity.resize(NUM_OF_RMD);
    m_joint_state.effort.resize(NUM_OF_RMD);
    m_multiturn_angle.data.resize(NUM_OF_RMD);
    for(size_t i=0;i<NUM_OF_RMD;i++){ m_joint_state.name[i] = "joint_" + std::to_string(i+1); }

    const char* data_path = DATAPATH;

    // REFERENCE = fopen( ((std::string(data_path)) + "reference.dat").c_str(),"w");
    // ANGLE = fopen(((std::string(data_path)) + "angle.dat").c_str(),"w");
    // ANGULAR_VELOCITY = fopen(((std::string(data_path)) + "angular_velocity.dat").c_str(),"w"); 
    // TORQUE = fopen(((std::string(data_path)) + "torque.dat").c_str(),"w");

    while(ros::ok())
    {
        ros::Time current_time = ros::Time::now(); 
        m_joint_state.header.stamp = current_time;

        m_reference.data = reference;
        p_reference.publish(m_reference);

        // fprintf(REFERENCE, "%lf\n", reference);

        for(size_t i=0;i<NUM_OF_RMD;i++){
            // fprintf(ANGLE, "%lf ", _DEV_MC[i].GetTheta()*RAD2DEG );
            // fprintf(ANGULAR_VELOCITY, "%lf ", _DEV_MC[i].GetThetaDot()*RAD2DEG );
            // fprintf(TORQUE, "%lf ", _DEV_MC[i].GetTorque() );

            m_joint_state.position[i] = _DEV_MC[i].GetTheta()*RAD2DEG;
            m_joint_state.velocity[i] = _DEV_MC[i].GetThetaDot()*RAD2DEG;
            m_joint_state.effort[i] = _DEV_MC[i].GetTorque();
            m_multiturn_angle.data[i] = _DEV_MC[i].GetMultiturnTheta();
        }
        
        // fprintf(ANGLE, "\n");
        // fprintf(ANGULAR_VELOCITY, "\n");
        // fprintf(TORQUE, "\n");

        p_joint_state.publish(m_joint_state);
        p_multiturn_angle.publish(m_multiturn_angle);

        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
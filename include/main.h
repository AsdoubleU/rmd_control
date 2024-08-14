#include "spi2can.h"
#include "rt_utils.h"
#include "motor_controller.h"
#include "callback.h"
#include "TrajectoryGenerator.h"
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

#define RAD2DEG 57.2957914

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

FILE *REFERENCE;
FILE *ANGLE[24];
FILE *ANGULAR_VELOCITY[24];
FILE *TORQUE[24];

ros::Publisher p_reference;
ros::Publisher p_joint_state;

std_msgs::Float64 m_angle[24];
std_msgs::Float64 m_angular_velocity[24];
std_msgs::Float64 m_torque[24];
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

    sensor_msgs::JointState m_joint_state;
    m_joint_state.name.resize(NUM_OF_RMD);
    m_joint_state.position.resize(NUM_OF_RMD);
    m_joint_state.velocity.resize(NUM_OF_RMD);
    m_joint_state.effort.resize(NUM_OF_RMD);
    for(size_t i=0;i<NUM_OF_RMD;i++){ m_joint_state.name[i] = "joint_" + std::to_string(i+1); }

    REFERENCE = fopen("/home/sthexa/data/reference.dat","w");

    ANGLE[0] = fopen("/home/sthexa/data/angle1.dat","w"); ANGLE[1] = fopen("/home/sthexa/data/angle2.dat","w");
    ANGLE[2] = fopen("/home/sthexa/data/angle3.dat","w"); ANGLE[3] = fopen("/home/sthexa/data/angle4.dat","w");
    ANGLE[4] = fopen("/home/sthexa/data/angle5.dat","w"); ANGLE[5] = fopen("/home/sthexa/data/angle6.dat","w");
    ANGLE[6] = fopen("/home/sthexa/data/angle7.dat","w"); ANGLE[7] = fopen("/home/sthexa/data/angle8.dat","w");
    ANGLE[8] = fopen("/home/sthexa/data/angle9.dat","w"); ANGLE[9] = fopen("/home/sthexa/data/angle10.dat","w");
    ANGLE[10] = fopen("/home/sthexa/data/angle11.dat","w"); ANGLE[11] = fopen("/home/sthexa/data/angle12.dat","w");
    ANGLE[12] = fopen("/home/sthexa/data/angle13.dat","w"); ANGLE[13] = fopen("/home/sthexa/data/angle14.dat","w");
    ANGLE[14] = fopen("/home/sthexa/data/angle15.dat","w"); ANGLE[15] = fopen("/home/sthexa/data/angle16.dat","w");
    ANGLE[16] = fopen("/home/sthexa/data/angle17.dat","w"); ANGLE[17] = fopen("/home/sthexa/data/angle18.dat","w");
    ANGLE[18] = fopen("/home/sthexa/data/angle19.dat","w"); ANGLE[19] = fopen("/home/sthexa/data/angle20.dat","w");
    ANGLE[20] = fopen("/home/sthexa/data/angle21.dat","w"); ANGLE[21] = fopen("/home/sthexa/data/angle22.dat","w");
    ANGLE[22] = fopen("/home/sthexa/data/angle23.dat","w"); ANGLE[23] = fopen("/home/sthexa/data/angle24.dat","w");

    ANGULAR_VELOCITY[0] = fopen("/home/sthexa/data/angular_velocity1.dat","w"); ANGULAR_VELOCITY[1] = fopen("/home/sthexa/data/angular_velocity2.dat","w");
    ANGULAR_VELOCITY[2] = fopen("/home/sthexa/data/angular_velocity3.dat","w"); ANGULAR_VELOCITY[3] = fopen("/home/sthexa/data/angular_velocity4.dat","w");
    ANGULAR_VELOCITY[4] = fopen("/home/sthexa/data/angular_velocity5.dat","w"); ANGULAR_VELOCITY[5] = fopen("/home/sthexa/data/angular_velocity6.dat","w");
    ANGULAR_VELOCITY[6] = fopen("/home/sthexa/data/angular_velocity7.dat","w"); ANGULAR_VELOCITY[7] = fopen("/home/sthexa/data/angular_velocity8.dat","w");
    ANGULAR_VELOCITY[8] = fopen("/home/sthexa/data/angular_velocity9.dat","w"); ANGULAR_VELOCITY[9] = fopen("/home/sthexa/data/angular_velocity10.dat","w");
    ANGULAR_VELOCITY[10] = fopen("/home/sthexa/data/angular_velocity11.dat","w"); ANGULAR_VELOCITY[11] = fopen("/home/sthexa/data/angular_velocity12.dat","w");
    ANGULAR_VELOCITY[12] = fopen("/home/sthexa/data/angular_velocity13.dat","w"); ANGULAR_VELOCITY[13] = fopen("/home/sthexa/data/angular_velocity14.dat","w");
    ANGULAR_VELOCITY[14] = fopen("/home/sthexa/data/angular_velocity15.dat","w"); ANGULAR_VELOCITY[15] = fopen("/home/sthexa/data/angular_velocity16.dat","w");
    ANGULAR_VELOCITY[16] = fopen("/home/sthexa/data/angular_velocity17.dat","w"); ANGULAR_VELOCITY[17] = fopen("/home/sthexa/data/angular_velocity18.dat","w");
    ANGULAR_VELOCITY[18] = fopen("/home/sthexa/data/angular_velocity19.dat","w"); ANGULAR_VELOCITY[19] = fopen("/home/sthexa/data/angular_velocity20.dat","w");
    ANGULAR_VELOCITY[20] = fopen("/home/sthexa/data/angular_velocity21.dat","w"); ANGULAR_VELOCITY[21] = fopen("/home/sthexa/data/angular_velocity22.dat","w");
    ANGULAR_VELOCITY[22] = fopen("/home/sthexa/data/angular_velocity23.dat","w"); ANGULAR_VELOCITY[23] = fopen("/home/sthexa/data/angular_velocity24.dat","w");

    TORQUE[0] = fopen("/home/sthexa/data/torque1.dat","w"); TORQUE[1] = fopen("/home/sthexa/data/torque2.dat","w");
    TORQUE[2] = fopen("/home/sthexa/data/torque3.dat","w"); TORQUE[3] = fopen("/home/sthexa/data/torque4.dat","w");
    TORQUE[4] = fopen("/home/sthexa/data/torque5.dat","w"); TORQUE[5] = fopen("/home/sthexa/data/torque6.dat","w");
    TORQUE[6] = fopen("/home/sthexa/data/torque7.dat","w"); TORQUE[7] = fopen("/home/sthexa/data/torque8.dat","w");
    TORQUE[8] = fopen("/home/sthexa/data/torque9.dat","w"); TORQUE[9] = fopen("/home/sthexa/data/torque10.dat","w");
    TORQUE[10] = fopen("/home/sthexa/data/torque11.dat","w"); TORQUE[11] = fopen("/home/sthexa/data/torque12.dat","w");
    TORQUE[12] = fopen("/home/sthexa/data/torque13.dat","w"); TORQUE[13] = fopen("/home/sthexa/data/torque14.dat","w");
    TORQUE[14] = fopen("/home/sthexa/data/torque15.dat","w"); TORQUE[15] = fopen("/home/sthexa/data/torque16.dat","w");
    TORQUE[16] = fopen("/home/sthexa/data/torque17.dat","w"); TORQUE[17] = fopen("/home/sthexa/data/torque18.dat","w");
    TORQUE[18] = fopen("/home/sthexa/data/torque19.dat","w"); TORQUE[19] = fopen("/home/sthexa/data/torque20.dat","w");
    TORQUE[20] = fopen("/home/sthexa/data/torque21.dat","w"); TORQUE[21] = fopen("/home/sthexa/data/torque22.dat","w");
    TORQUE[22] = fopen("/home/sthexa/data/torque23.dat","w"); TORQUE[23] = fopen("/home/sthexa/data/torque24.dat","w");

    while(ros::ok())
    {
        ros::Time current_time = ros::Time::now(); 
        m_joint_state.header.stamp = current_time;

        // m_reference.data = reference*30/1080;
        m_reference.data = reference;
        p_reference.publish(m_reference);

        fprintf(REFERENCE, "%lf\n", reference);

        for(size_t i=0;i<NUM_OF_RMD;i++){
            fprintf(ANGLE[i], "%lf\n", _DEV_MC[i].GetTheta()*RAD2DEG );
            fprintf(ANGULAR_VELOCITY[i], "%lf\n", _DEV_MC[i].GetThetaDot() );
            fprintf(TORQUE[i], "%lf\n", _DEV_MC[i].GetTorque() );

            m_joint_state.position[i] = _DEV_MC[i].GetTheta()*RAD2DEG;
            m_joint_state.velocity[i] = _DEV_MC[i].GetThetaDot();
            m_joint_state.effort[i] = _DEV_MC[i].GetTorque();
        }

        p_joint_state.publish(m_joint_state);

        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
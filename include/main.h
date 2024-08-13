#include "spi2can.h"
#include "rt_utils.h"
#include "motor_controller.h"
#include "callback.h"
#include "ros_plot.h"
#include "TrajectoryGenerator.h"

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

    p_angle[0] = node_handle_.advertise<std_msgs::Float64>("/angle_1/",1);
    p_angle[1] = node_handle_.advertise<std_msgs::Float64>("/angle_2/",1);
    p_angle[2] = node_handle_.advertise<std_msgs::Float64>("/angle_3/",1);
    p_angle[3] = node_handle_.advertise<std_msgs::Float64>("/angle_4/",1);
    p_angle[4] = node_handle_.advertise<std_msgs::Float64>("/angle_5/",1);
    p_angle[5] = node_handle_.advertise<std_msgs::Float64>("/angle_6/",1);
    p_angle[6] = node_handle_.advertise<std_msgs::Float64>("/angle_7/",1);
    p_angle[7] = node_handle_.advertise<std_msgs::Float64>("/angle_8/",1);
    p_angle[8] = node_handle_.advertise<std_msgs::Float64>("/angle_9/",1);
    p_angle[9] = node_handle_.advertise<std_msgs::Float64>("/angle_10/",1);
    p_angle[10] = node_handle_.advertise<std_msgs::Float64>("/angle_11/",1);
    p_angle[11] = node_handle_.advertise<std_msgs::Float64>("/angle_12/",1);
    p_angle[12] = node_handle_.advertise<std_msgs::Float64>("/angle_13/",1);
    p_angle[13] = node_handle_.advertise<std_msgs::Float64>("/angle_14/",1);
    p_angle[14] = node_handle_.advertise<std_msgs::Float64>("/angle_15/",1);
    p_angle[15] = node_handle_.advertise<std_msgs::Float64>("/angle_16/",1);
    p_angle[16] = node_handle_.advertise<std_msgs::Float64>("/angle_17/",1);
    p_angle[17] = node_handle_.advertise<std_msgs::Float64>("/angle_18/",1);
    p_angle[18] = node_handle_.advertise<std_msgs::Float64>("/angle_19/",1);
    p_angle[19] = node_handle_.advertise<std_msgs::Float64>("/angle_20/",1);
    p_angle[20] = node_handle_.advertise<std_msgs::Float64>("/angle_21/",1);
    p_angle[21] = node_handle_.advertise<std_msgs::Float64>("/angle_22/",1);
    p_angle[22] = node_handle_.advertise<std_msgs::Float64>("/angle_23/",1);
    p_angle[23] = node_handle_.advertise<std_msgs::Float64>("/angle_24/",1);

    p_angular_velocity[0] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_1/",1);
    p_angular_velocity[1] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_2/",1);
    p_angular_velocity[2] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_3/",1);
    p_angular_velocity[3] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_4/",1);
    p_angular_velocity[4] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_5/",1);
    p_angular_velocity[5] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_6/",1);
    p_angular_velocity[6] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_7/",1);
    p_angular_velocity[7] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_8/",1);
    p_angular_velocity[8] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_9/",1);
    p_angular_velocity[9] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_10/",1);
    p_angular_velocity[10] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_11/",1);
    p_angular_velocity[11] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_12/",1);
    p_angular_velocity[12] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_13/",1);
    p_angular_velocity[13] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_14/",1);
    p_angular_velocity[14] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_15/",1);
    p_angular_velocity[15] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_16/",1);
    p_angular_velocity[16] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_17/",1);
    p_angular_velocity[17] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_18/",1);
    p_angular_velocity[18] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_19/",1);
    p_angular_velocity[19] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_20/",1);
    p_angular_velocity[20] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_21/",1);
    p_angular_velocity[21] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_22/",1);
    p_angular_velocity[22] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_23/",1);
    p_angular_velocity[23] = node_handle_.advertise<std_msgs::Float64>("/angular_velocity_24/",1);

    p_torque[0] = node_handle_.advertise<std_msgs::Float64>("/torque_1/",1);
    p_torque[1] = node_handle_.advertise<std_msgs::Float64>("/torque_2/",1);
    p_torque[2] = node_handle_.advertise<std_msgs::Float64>("/torque_3/",1);
    p_torque[3] = node_handle_.advertise<std_msgs::Float64>("/torque_4/",1);
    p_torque[4] = node_handle_.advertise<std_msgs::Float64>("/torque_5/",1);
    p_torque[5] = node_handle_.advertise<std_msgs::Float64>("/torque_6/",1);
    p_torque[6] = node_handle_.advertise<std_msgs::Float64>("/torque_7/",1);
    p_torque[7] = node_handle_.advertise<std_msgs::Float64>("/torque_8/",1);
    p_torque[8] = node_handle_.advertise<std_msgs::Float64>("/torque_9/",1);
    p_torque[9] = node_handle_.advertise<std_msgs::Float64>("/torque_10/",1);
    p_torque[10] = node_handle_.advertise<std_msgs::Float64>("/torque_11/",1);
    p_torque[11] = node_handle_.advertise<std_msgs::Float64>("/torque_12/",1);
    p_torque[12] = node_handle_.advertise<std_msgs::Float64>("/torque_13/",1);
    p_torque[13] = node_handle_.advertise<std_msgs::Float64>("/torque_14/",1);
    p_torque[14] = node_handle_.advertise<std_msgs::Float64>("/torque_15/",1);
    p_torque[15] = node_handle_.advertise<std_msgs::Float64>("/torque_16/",1);
    p_torque[16] = node_handle_.advertise<std_msgs::Float64>("/torque_17/",1);
    p_torque[17] = node_handle_.advertise<std_msgs::Float64>("/torque_18/",1);
    p_torque[18] = node_handle_.advertise<std_msgs::Float64>("/torque_19/",1);
    p_torque[19] = node_handle_.advertise<std_msgs::Float64>("/torque_20/",1);
    p_torque[20] = node_handle_.advertise<std_msgs::Float64>("/torque_21/",1);
    p_torque[21] = node_handle_.advertise<std_msgs::Float64>("/torque_22/",1);
    p_torque[22] = node_handle_.advertise<std_msgs::Float64>("/torque_23/",1);
    p_torque[23] = node_handle_.advertise<std_msgs::Float64>("/torque_24/",1);

    p_reference = node_handle_.advertise<std_msgs::Float64>("/reference/",1);

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

        for(size_t i=0;i<NUM_OF_RMD;i++){
            m_angle[i].data = _DEV_MC[i].GetTheta()*RAD2DEG;
            m_angular_velocity[i].data = _DEV_MC[i].GetThetaDot();
            m_torque[i].data = _DEV_MC[i].GetTorque();
            p_angle[i].publish(m_angle[i]);
            p_angular_velocity[i].publish(m_angular_velocity[i]);
            p_torque[i].publish(m_torque[i]);
        }
        // m_reference.data = reference*30/1080;
        m_reference.data = reference/1080*30;
        p_reference.publish(m_reference);

        fprintf(REFERENCE, "%lf\n", reference);

        for(size_t i=0;i<NUM_OF_ACTUATORS;i++){
            fprintf(ANGLE[i], "%lf\n", _DEV_MC[i].GetTheta()*RAD2DEG );
            fprintf(ANGULAR_VELOCITY[i], "%lf\n", _DEV_MC[i].GetThetaDot() );
            fprintf(TORQUE[i], "%lf\n", _DEV_MC[i].GetTorque() );
        }

        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
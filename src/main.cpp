#include "spi2can.h"
#include "rt_utils.h"
#include "motor_controller.h"
#include "callback.h"
#include "ros_plot.h"

float reference;
float ddot_old;
float filter_old;

static void *rt_motion_thread(void *arg);
pRBCORE_SHM sharedData;
rmd_motor _DEV_MC[NUM_OF_RMD];
Motor_Controller motor_ctrl;
Callback callback;


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
        m_reference.data = reference;
        p_reference.publish(m_reference);

        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}


void *rt_motion_thread(void *arg){
    const long PERIOD_US = RT_MS * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    int thread_loop_count = 0;
    int motion_count = 0;
    int motion_count_time_sec = 0;
    double control_time = 0.;
    bool is_print_comm_frequency = true;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    bool is_first_loop = true;

    while(true){
        control_time = thread_loop_count/500.;

        if(is_first_loop){
            motor_ctrl.EnableMotor();

            timespec_add_us(&TIME_NEXT, 4 * 1000 * 1000);
            is_first_loop = false;
            thread_loop_count++;
        }

        else if(thread_loop_count > 1000){

            motor_ctrl.SetTorque( 2.*sin(control_time/0.3) );
            // motor_ctrl.SetPosition(8000, 4000*sin(control_time));
            // _DEV_MC[0].SetTorqueData( 2.0 );
            // _DEV_MC[0].SetVelocityDta( 8000.*sin(control_time/0.3) );

            if(motion_count > 500 && is_print_comm_frequency) {
                motion_count = 1;
                if(motion_count_time_sec < 120)
                {
                    motion_count_time_sec++;
                    ROS_INFO("Reception Count for the nth motor");
                    for(int i = 0; i < NUM_OF_RMD; i++ )
                    {
                        ROS_INFO("%dth --> %d times",i,_DEV_MC[i].count);
                        _DEV_MC[i].count = 0; _DEV_MC[i].count_A1 = 0; 
                    }
                    std::cout<<std::endl;
                }
                else is_print_comm_frequency = false;
            }
            if(is_print_comm_frequency) motion_count++;

            thread_loop_count++;
        }

        else thread_loop_count++;

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) ROS_ERROR("RT Deadline Miss, main controller : %.3f ms",timediff_us(&TIME_NEXT, &TIME_NOW)*0.001); 
    }
}
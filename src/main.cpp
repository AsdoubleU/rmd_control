#include "spi2can.h"
#include "rt_utils.h"
#include "motor_controller.h"
#include "callback.h"
#include "ros_plot.h"

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

    p_angle = node_handle_.advertise<std_msgs::Float64>("/Angle/",1);
    p_angular_velocity = node_handle_.advertise<std_msgs::Float64>("/Angular_Velocity/",1);
    p_torque = node_handle_.advertise<std_msgs::Float64>("/Torque/",1);

    while(ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        m_angle.data = _DEV_MC[0].GetTheta()*RAD2DEG;
        m_angular_velocity.data = _DEV_MC[0].GetThetaDot();

        p_angle.publish(m_angle);
        p_angular_velocity.publish(m_angular_velocity);

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
        
        control_time = thread_loop_count/500;

        if(is_first_loop){
            motor_ctrl.EnableMotor();

            timespec_add_us(&TIME_NEXT, 4 * 1000 * 1000);
            is_first_loop = false;
            thread_loop_count++;
        }

        else if(thread_loop_count > 1000){

            // motor_ctrl.SetTorque( 10.*sin(control_time/0.3) );
            // _DEV_MC[0].SetVelocityData(100000*sin(control_time));

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
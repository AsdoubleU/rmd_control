#include "spi2can.h"
#include "rt_utils.h"
#include "dynamics.h"
#include "motor_controller.h"
#include "callback.h"

static void *rt_motion_thread(void *arg);
pRBCORE_SHM sharedData;
rmd_motor _DEV_MC[NUM_OF_RMD];
Dynamics::RobotDynamics dynamics;
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

    std::vector<std::string> joints_name = {"j1", "j2", "j3", "j4", "j5", "j6", "j1", "j2", "j3", "j4", "j5", "j6"};

    while(ros::ok())
    {
        sensor_msgs::JointState msg;
        geometry_msgs::Pose ee_pose_msg;
        geometry_msgs::Pose ref_ee_pose_msg;
       
        msg.header.stamp = ros::Time::now();

        for (uint8_t i = 0; i<NUM_OF_RMD; i ++)
        {
            msg.name.push_back(joints_name.at(i));
            msg.position.push_back(dynamics.th[i]);//actual angle
            msg.velocity.push_back(dynamics.filtered_th_dot[i]);//filtered angular speed * RAD2DEG
            msg.effort.push_back(dynamics.joint_torque[i]);//reference torque
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


void *rt_motion_thread(void *arg){
    const long PERIOD_US = RT_MS * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    int loop_count = 0;
    int comm_loop_count = 0;
    int comm_loop_count_time_sec = 0;
    bool is_print_comm_frequency = true;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    bool is_first_loop = true;

    while(true){

        if(is_first_loop){
            motor_ctrl.EnableMotor();
            // motor_ctrl.SetTorque(10);
            // motor_ctrl.EnableFilter();//first motor setting function

            timespec_add_us(&TIME_NEXT, 4 * 1000 * 1000);
            is_first_loop = false;
            loop_count++;
        }
        else if(loop_count > 1000){
            loop_count++;
            // dynamics.Loop();
            // motor_ctrl.EnableFilter();//first motor setting function

            if(comm_loop_count > 500 && is_print_comm_frequency) {
                comm_loop_count = 1;
                if(comm_loop_count_time_sec < 120)
                {
                    std::cout<<"[HRRLab Hexapod Info] : "<<"Reception Count for the nth motor"<<std::endl<<std::endl;
                    comm_loop_count_time_sec++;
                    for(int i = 0; i < NUM_OF_RMD; i++ )
                    {
                        std::cout << i << ": " << _DEV_MC[i].count << "     " ;
                        _DEV_MC[i].count = 0; _DEV_MC[i].count_A1 = 0;
                        if(i%6 == 0){ std::cout<<std::endl; }
                    }
                    std::cout << " " << std::endl;
                }
                else is_print_comm_frequency = false;
            }
            if(is_print_comm_frequency) comm_loop_count++;
        }
        else loop_count++;

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
            std::cout << "RT Deadline Miss, main controller :  " << timediff_us(&TIME_NEXT, &TIME_NOW)*0.001<<" ms"<< std::endl;
        }
    }
}
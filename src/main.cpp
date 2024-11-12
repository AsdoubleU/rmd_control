#include "main.h"

void *rt_motion_thread(void *arg){

    const long PERIOD_US = RT_MS * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    int thread_loop_count = 0;
    int motion_count = 0;
    int motion_count_time_sec = 0;
    double control_time = 0.;
    double motion_time = 0.;
    bool is_print_comm_frequency = true;

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    bool is_first_loop = true;

    while(true){
        control_time = thread_loop_count/500.;

        if(is_first_loop){

            for (size_t i=0;i<NUM_OF_ACTUATORS;i++){
                traj[i].init(0.002,1);
                traj[i].isEnd = false;
                traj[i].t_ = 0;
            }
            timespec_add_us(&TIME_NEXT, 4 * 1000 * 1000);
            is_first_loop = false;
            motion_time = 0;
            thread_loop_count++;

        }

        else if(thread_loop_count < 200 ){
            motor_ctrl.DisableMotor();
            thread_loop_count++;
        }

        else if(thread_loop_count < 300 && thread_loop_count > 200){
            motor_ctrl.EnableMotor();
            thread_loop_count++;
        }
        

        else if(thread_loop_count < 500 && thread_loop_count > 300){
            motor_ctrl.SetTorque(0);
            motor_ctrl.SetInitialTheta();
            thread_loop_count++;
        }

        else if(thread_loop_count < 1000 && thread_loop_count > 500){
            motor_ctrl.ReadGainDatas();
            thread_loop_count++;
        }

        else if(thread_loop_count > 1000 && thread_loop_count < 4000){

            for(size_t i=0;i<NUM_OF_ACTUATORS;i++) { _DEV_MC[i].SetPositionData(100,0); }
            
            if(thread_loop_count > 3998) {
                ROS_INFO("Motor is zero position");
                // motor_ctrl.PrintGainDatas();
                for(size_t i=0;i<NUM_OF_ACTUATORS;i++){
                    traj[i].isEnd = false;
                    traj[i].t_ = 0;
                }
            }

            thread_loop_count++;
        }

        else if(thread_loop_count > 4000) {

            reference = 20*sin((motion_time));
            motor_ctrl.SetPosition(8000, reference*DEG2RAD);
            // reference = 1*sin((motion_time)/0.3);
            // _DEV_MC[0].SetTorqueData( _DEV_MC[0].JointSpacePD(10.0,0.,reference/RAD2DEG) );
            // motor_ctrl.SetTorque( reference );
            // motor_ctrl.SetTorque( 0 );
            // _DEV_MC[0].SetVelocityDta( 8000.*sin(control_time/0.3) );

            // // Trajectory fixed // //
            // for(size_t i=0;i<NUM_OF_ACTUATORS;i++) { 
            //     traj[i].SetSinusoidalTrajectory(60., 0., 1.0);
            //     reference = traj[0].GetRefvar();
            //     if(!traj[i].isEnd) _DEV_MC[i].SetPositionData(8000,traj[i].GetRefvar()*DEG2RAD);
            //     // _DEV_MC[i].JointSpacePD(3,0.03,traj[i].GetRefvar()*DEG2RAD, traj[i].GetRefvardot()*DEG2RAD);
            // }

            // // Trajectory tracking // //

            // if(mode == 0) {
            //     for(size_t i=0;i<NUM_OF_ACTUATORS;i++) { 
            //         traj[i].SetSinusoidalTrajectory(60., 0., 1.0);
            //         reference = traj[0].GetRefvar();
            //         if(!traj[i].isEnd) _DEV_MC[i].SetPositionData(8000,traj[i].GetRefvar()*DEG2RAD);
            //         // if(!traj[i].isEnd) _DEV_MC[i].JointSpacePD(3,0.03,traj[i].GetRefvar()*DEG2RAD, traj[i].GetRefvardot()*DEG2RAD);
            //     }

            //     if(motion_time > 1.0) {
            //         mode++;
            //         for(size_t i=0;i<NUM_OF_ACTUATORS;i++){
            //                 traj[i].isEnd = false;
            //                 traj[i].t_ = 0;
            //         }
            //     }
            // }

            // else if(mode == 1) {
            //     for(size_t i=0;i<NUM_OF_ACTUATORS;i++) { 
            //         traj[i].SetSinusoidalTrajectory(30., 60., 1.0);
            //         reference = traj[0].GetRefvar();
            //         if(!traj[i].isEnd) _DEV_MC[i].SetPositionData(8000,traj[i].GetRefvar()*DEG2RAD);
            //         // if(!traj[i].isEnd) _DEV_MC[i].JointSpacePD(3,0.03,traj[i].GetRefvar()*DEG2RAD, traj[i].GetRefvardot()*DEG2RAD);
            //     }

            //     if(motion_time > 2.0) {
            //         mode++;
            //         for(size_t i=0;i<NUM_OF_ACTUATORS;i++){
            //             traj[i].isEnd = false;
            //             traj[i].t_ = 0;
            //         }
            //     }
            // }

            // else if(mode == 2) {
            //     for(size_t i=0;i<NUM_OF_ACTUATORS;i++) { 
            //         traj[i].SetSinusoidalTrajectory(0., 30., 1.0);
            //         reference = traj[0].GetRefvar();
            //         if(!traj[i].isEnd) _DEV_MC[i].SetPositionData(8000,traj[i].GetRefvar()*DEG2RAD);
            //         // if(!traj[i].isEnd) _DEV_MC[i].JointSpacePD(3,0.03,traj[i].GetRefvar()*DEG2RAD, traj[i].GetRefvardot()*DEG2RAD);
            //     }

            //     if(motion_time > 3.0) {
            //         mode = 0;
            //         motion_time = 0;
            //         for(size_t i=0;i<NUM_OF_ACTUATORS;i++){
            //             traj[i].isEnd = false;
            //             traj[i].t_ = 0;
            //         }
            //     }
            // }

            motion_time += 0.002;

            if(motion_count > 500 && is_print_comm_frequency) {
                motion_count = 1;
                if(motion_count_time_sec < 120)
                {
                    motion_count_time_sec++;
                    // ROS_INFO("Reception Count for the nth motor");
                    for(int i = 0; i < NUM_OF_ACTUATORS; i++ )
                    {
                        // ROS_INFO("%dth --> %d times",i,_DEV_MC[i].count);
                        _DEV_MC[i].count = 0; _DEV_MC[i].count_A1 = 0; 
                    }
                    // std::cout<<std::endl;
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
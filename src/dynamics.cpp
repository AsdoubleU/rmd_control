#include "dynamics.h"

extern Motor_Controller motor_ctrl;

namespace Dynamics
{
    RobotDynamics::RobotDynamics()
    {
        // InitializeRBDLVariables();
        for (uint8_t i=0; i<NUM_OF_JOINTS; i++)
        {
            joint_torque[i] = 0;
        }
        std::cout<<"set Torque "<<std::endl;
    }

    void RobotDynamics::Loop()
    {
        SetTheta(motor_ctrl.GetJointTheta());
        SetThetaDotSMAF(motor_ctrl.GetThetaDotSMAF());
        motor_ctrl.GeThetaDofLP();
        SetThetaDot(motor_ctrl.GetThetaDot());
        SetRBDLVariables();
        motor_ctrl.SetTorque(GetTorque());
        PrintToFile();
    }

    void RobotDynamics::PostureGeneration()
    {
        switch(control_mode)
        {
            case gravity_compensation:
                GenerateTorqueGravityCompensation();
                break;
            case joint_space:
                GenerateTrajectory();
                GenerateTorqueJointSpacePD();
                break;
            case joint_space2:
                GenerateTrajectory();
                GenerateTorqueJointSpacePD2();
                break;
            case task_space:
                GenerateTorqueTaskSpacePD();
                break;    
            case TASKSPACEMOTION:
                TaskSpaceMotion();
                break;     
            case CTCMOTION:
                CTCMotion();
                break;            
            case clear_torque:
                ClearTorque();
                break;
            case motor_test:
                GenerateTorqueJointSpacePDForMotorTest();
                break;            
        }
    }

    void RobotDynamics::PrintToFile()
    {
        if (record_data)
        {
            if (count_for_print_to_file < 5000)
            {
                //fprintf(Experiment_data, "%lf, %lf, %lf, %lf \n", joint_torque[2],th_dot_sma_filtered[2],ref_th[2],th[2]);
                fprintf(Experiment_data, "%lf, %lf, %lf \n", ref_th[4],th[4],joint_torque[4]);
            }
            else if (count_for_print_to_file == 5000)
            {
                fclose(Experiment_data);
                record_data = false;
                std::cout << "record data finished" << std::endl;
            }
            count_for_print_to_file++;
        }
    }

    void RobotDynamics::SwitchRecording()
    {
        Experiment_data= fopen("/home/rainbow/catkin_ws/src/Experiment_data.dat","w");
        record_data = true;
        count_for_print_to_file = 0;
        std::cout << "Recording started" << std::endl;
    }

    void RobotDynamics::SwitchMode(const std_msgs::Int32ConstPtr & msg)
    {
        cnt = 0;
        if(msg -> data == 0) 
        {
            control_mode = gravity_compensation;
            std::cout << "change variable ref_th=th " << std::endl;
        }
            
        else if (msg -> data == 1)
        {
            control_mode = joint_space;

            ref_angle[0] = ref_th[0];
            ref_angle[1] = ref_th[1];
            ref_angle[2] = ref_th[2];
            ref_angle[3] = ref_th[3];
            ref_angle[4] = ref_th[4];
            ref_angle[5] = ref_th[5];

            ref_angle[6] = ref_th[6];
            ref_angle[7] = ref_th[7];
            ref_angle[8] = ref_th[8];
            ref_angle[9] = ref_th[9];
            ref_angle[10] = ref_th[10];
            ref_angle[11] = ref_th[11];
            
            count = 0;

            std::cout << "Changed motion to joint space PD" << std::endl;
        } 
        else if (msg -> data == 2)
        {
            control_mode = joint_space2;

            ref_angle[0] = ref_th[0];
            ref_angle[1] = ref_th[1];
            ref_angle[2] = ref_th[2];
            ref_angle[3] = ref_th[3];
            ref_angle[4] = ref_th[4];
            ref_angle[5] = ref_th[5];

            ref_angle[6] = ref_th[6];
            ref_angle[7] = ref_th[7];
            ref_angle[8] = ref_th[8];
            ref_angle[9] = ref_th[9];
            ref_angle[10] = ref_th[10];
            ref_angle[11] = ref_th[11];
            
            count = 0;
            cnt = 0; 
            std::cout << "Changed motion to joint space PD2" << std::endl;
        } 
        else if (msg -> data == 3) 
        {
            control_mode = task_space;
            count = 0;
            cnt = 0;
            taskspace_count = 0;
            std::cout << "Changed motion to task space PD" << std::endl;
            for(int i =0; i < 3; i++)
            {
                std::cout << initial_ee_position(i)<< std::endl;
            }
            //Position_data = fopen("/home/rainbow/catkin_ws/src/Taskspace_position.dat","w");
        }
        else if (msg -> data == 4)
        {
            control_mode = gravity_compensation;
            std::cout << "Changed motion to gravity compensation" << std::endl;
        }  
        else if (msg -> data == 5)
        {
            cnt = 0;
            control_mode = TASKSPACEMOTION;
            for(int i =0; i < 6; i++)
            {
                std::cout << swing_L.Init_PR(i)<< std::endl;
            }
            std::cout << "Changed motion to TaskSpaceMotion" << std::endl;
        }  
        else if (msg -> data == 6)
        {
            cnt = 0;
            chg_cnt=0;
            control_mode = CTCMOTION;
            for(int i =0; i < 6; i++)
            {
                std::cout << swing_L.Init_PR(i)<< std::endl;
            }
            std::cout << "Changed motion to CTCMotion" << std::endl;
        }  
        else if (msg -> data == 7)
        {
            control_mode = clear_torque;
            std::cout << "Torque clear" << std::endl;
        } 
        else if (msg -> data == 10)
        {
            control_mode = motor_test;
            std::cout << "Mode is switched to Motor test" << std::endl;
            motor_test_count = 0;
            cnt = 0;
        }
        
    }

    void RobotDynamics::SetTheta(VectorXd a_theta)
    {
        for(uint8_t i=0; i<NUM_OF_JOINTS; i++) th[i] = a_theta[i];
    }

    void RobotDynamics::SetThetaDot(VectorXd a_theta_dot)
    {
        for(uint8_t i=0; i<NUM_OF_JOINTS; i++) th_dot[i] = a_theta_dot[i]; 
    }

    void RobotDynamics::SetThetaDotSMAF(VectorXd a_theta_dot)
    {
        for(uint8_t i=0; i<NUM_OF_JOINTS; i++) filtered_th_dot[i] = a_theta_dot[i];

        th_d_dot = (filtered_th_dot - last_th_dot) / dt;
        last_th_dot = filtered_th_dot;
    }

    VectorXd RobotDynamics::GetTorque()
    {   
        return joint_torque;
        //return tau_gravity_compensation;
    }

    void RobotDynamics::GenerateTrajectory()
    {
        float count_time = count * dt;
        float count_time2 = count * dt;
        
        if(count_time <= step_time)
        {
            trajectory = 0.5* (1 - cos(PI * (count_time/step_time)));
        }
       
        trajectory2 = 0.5 * (1 - cos(PI * (count_time/step_time)));//unlimited trajectory
        //ref_th[i]= amplitude * sin( PI * ( frequency * count_time));
        
        count++;
        
    }

    void RobotDynamics::UpdateGainValues()
    {
        if(js_kp.update_flag)
        {
            if(js_kp.count == js_kp.max_count-1) js_kp.real_gain_value = js_kp.old_gain_value;
            js_kp.count = 0;
            js_kp.old_gain_value = js_kp.real_gain_value;
            js_kp.new_gain_value = js_kp.callback_gain_value;
            js_kp.delta_gain_value = (js_kp.new_gain_value - js_kp.old_gain_value) / js_kp.max_count;
            js_kp.update_flag = false;
        }
        if(js_kp.count < js_kp.max_count)
        {
            js_kp.real_gain_value += js_kp.delta_gain_value;
            js_kp.count++;
        }

        if(js_kd.update_flag)
        {
            if(js_kd.count == js_kd.max_count-1) js_kd.real_gain_value = js_kd.old_gain_value;
            js_kd.count = 0;
            js_kd.old_gain_value = js_kd.real_gain_value;
            js_kd.new_gain_value = js_kd.callback_gain_value;
            js_kd.delta_gain_value = (js_kd.new_gain_value - js_kd.old_gain_value) / js_kd.max_count;
            js_kd.update_flag = false;
        }
        if(js_kd.count < js_kd.max_count)
        {
            js_kd.real_gain_value += js_kd.delta_gain_value;
            js_kd.count++;
        }

        if(js_gp.update_flag)
        {
            if(js_gp.count == js_gp.max_count-1) js_gp.real_gain_value = js_gp.old_gain_value;
            js_gp.count = 0;
            js_gp.old_gain_value = js_gp.real_gain_value;
            js_gp.new_gain_value = js_gp.callback_gain_value;
            js_gp.delta_gain_value = (js_gp.new_gain_value - js_gp.old_gain_value) / js_gp.max_count;
            js_gp.update_flag = false;
        }
        if(js_gp.count < js_gp.max_count)
        {
            js_gp.real_gain_value += js_gp.delta_gain_value;
            js_gp.count++;
        }

    }

    void RobotDynamics::ClearTorque()
    {
        for (uint8_t i=0; i<NUM_OF_JOINTS; i++)
        {
            joint_torque[i] = 0;
            torque_F(i) = 0;
        }
    }

    void RobotDynamics::SetRBDLVariables()
    {

            swing_L.q(0) = th(0);
            swing_L.q(1) = th(1);
            swing_L.q(2) = th(2);
            swing_L.q(3) = th(3);
            swing_L.q(4) = PI/2+th(4)-th(3);
            swing_L.q(5) = th(5);

            swing_R.q(0) = th(6);
            swing_R.q(1) = th(7);
            swing_R.q(2) = th(8);
            swing_R.q(3) = th(9);
            swing_R.q(4) = PI/2+th(10)-th(9);
            swing_R.q(5) = th(11);


            for(uint8_t i=0; i < 6; i++)
            {
                swing_L.q_dot(i) = filtered_th_dot(i);
                swing_L.q_d_dot(i) = th_d_dot(i);

                swing_R.q_dot(i) = filtered_th_dot(i+6);
                swing_R.q_d_dot(i) = th_d_dot(i+6);
            }
           
       
    }

    void RobotDynamics::InitializeRBDLVariables()
    {
        rbdl_check_api_version(RBDL_API_VERSION);
        std::cout << "Checked RBDL API VERSION" << std::endl;
        
        swing_L.rbdl_model = new RBDLModel();
        swing_L.rbdl_model->gravity = RBDL::Math::Vector3d(0., 0., -9.81);
        
        L_Air_Model(swing_L);
        rbdl_variable_init(swing_L);
        std::cout<<"DoF of swing_L : "<<swing_L.rbdl_model->dof_count<<std::endl;
        
        ////////////////////////////////////////////////////////////////right leg///////////////////////////////////////////////////////

        swing_R.rbdl_model = new RBDLModel();
        swing_R.rbdl_model->gravity = RBDL::Math::Vector3d(0., 0., -9.81);
        
        R_Air_Model(swing_R);
        rbdl_variable_init(swing_R);
        std::cout<<"DoF of swing_R : "<<swing_R.rbdl_model->dof_count<<std::endl;

        std::cout << "RBDL Initialize function success" << std::endl;
    }
    
    void RobotDynamics::rbdl_variable_init(Air_RBDL &rbdl)
    {
        rbdl.q = RBDLVectorNd::Zero(6);
        rbdl.q_dot = RBDLVectorNd::Zero(6);
        rbdl.q_d_dot = RBDLVectorNd::Zero(6);
        rbdl.prevQ = RBDLVectorNd::Zero(rbdl.rbdl_model->dof_count);
        rbdl.prevQDot = RBDLVectorNd::Zero(rbdl.rbdl_model->dof_count);
        rbdl.tau = RBDLVectorNd::Zero(6); 

        rbdl.Init_PR = RBDLVectorNd::Zero(6);
        rbdl.Foot_Pos = RBDLVectorNd::Zero(6);
        rbdl.Foot_Pos_dot = RBDLVectorNd::Zero(6);
        rbdl.A_Jacobian = RBDLMatrixNd::Zero(6,6);
        rbdl.prev_A_Jacobian = RBDLMatrixNd::Zero(6,6);
        rbdl.A_Jacobian_dot = RBDLMatrixNd::Zero(6,6);
        rbdl.Inv_A_Jacobian = RBDLMatrixNd::Zero(6,6);
        rbdl.Des_X = RBDLVectorNd::Zero(6);
        rbdl.refTask = RBDLVectorNd::Zero(6);
        rbdl.torque_Task = RBDLVectorNd::Zero(6);


        rbdl.Des_XDot = RBDLVectorNd::Zero(6);
        rbdl.Des_XDDot = RBDLVectorNd::Zero(6);
        rbdl.torque_CTC = RBDLVectorNd::Zero(6);
        rbdl.Old_Des_X = RBDLVectorNd::Zero(6);
        rbdl.Old_Des_XDot = RBDLVectorNd::Zero(6);
        rbdl.Old_Des_XDDot = RBDLVectorNd::Zero(6);
        rbdl.New_Des_X = RBDLVectorNd::Zero(6);
        rbdl.New_Des_XDot = RBDLVectorNd::Zero(6);
        rbdl.New_Des_XDDot = RBDLVectorNd::Zero(6); 

        rbdl.Kp = RBDLVectorNd::Zero(6);
        rbdl.Kv = RBDLVectorNd::Zero(6); // 계산한 결과값들을 저장하는 변수
    }

    void RobotDynamics::L_Air_Model(Air_RBDL &rbdl)
    {

        rbdl.hip_yaw_inertia = RBDLMatrix3d(0.0008989,   1.17e-05,     2.3e-06, 
                                                     1.17e-05,   0.0006077,       7e-07, 
                                                     2.3e-06,        7e-07,   0.0004679);

        rbdl.hip_roll_inertia = RBDLMatrix3d(0.0003885,   -1.06e-05,     1.5e-06, 
                                                     -1.06e-05,    0.0005126,    -3.4e-06, 
                                                      1.5e-06,      -3.4e-06,   0.0004593);

        rbdl.hip_pitch_inertia = RBDLMatrix3d(0.0134848,    -2.17e-05,       1.85e-05, 
                                                       -2.17e-05,      0.0125048,    0.0016397, 
                                                        1.85e-05,      0.0016397,    0.0021808);

        rbdl.knee_pitch_inertia = RBDLMatrix3d(0.0022136,         6e-07, -1.16e-05, 
                                                            6e-07,     0.0021655,   4.5e-06, 
                                                        -1.16e-05,       4.5e-06,  0.000122);

        rbdl.ankle_pitch_inertia = RBDLMatrix3d(1.45e-05,     5e-07,     8e-07, 
                                                         5e-07,     5.67e-05,         0, 
                                                         8e-07,            0,   5.85e-05);

        rbdl.ankle_roll_inertia = RBDLMatrix3d(0.0013997,      4e-07,   0.0004991, 
                                                            4e-07,  0.0026224,    -5.4e-06, 
                                                        0.0004991,   -5.4e-06,   0.0026109);

        rbdl.foot_inertia = RBDLMatrix3d(  0,0,0,
                                                    0,0,0,
                                                    0,0,0);


        rbdl.hip_yaw_link = RBDLBody(0.75394, RBDLVector3d(0.001248,-0.000607, -0.000365), rbdl.hip_yaw_inertia); //기존 무게중심 위치
        rbdl.hip_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 0, 1)); // pelvis yaw
        rbdl.hip_yaw_id = rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0, 0.075, -0.096)), rbdl.hip_yaw_joint, rbdl.hip_yaw_link);
        
        rbdl.hip_roll_link = RBDLBody(0.69039, RBDLVector3d(-0.061767 ,0.00154 ,-8.2e-05), rbdl.hip_roll_inertia); // 기존
        rbdl.hip_roll_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(1, 0, 0)); // pelvis roll
        rbdl.hip_roll_id = rbdl.rbdl_model->RBDLModel::AddBody(1, RBDL::Math::Xtrans(RBDLVector3d(0, 0, 0)), rbdl.hip_roll_joint, rbdl.hip_roll_link);

        rbdl.hip_pitch_link = RBDLBody(2.0087, RBDLVector3d(0.000349 ,0.084719 ,-0.072582), rbdl.hip_pitch_inertia); // 기존
        rbdl.hip_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 1, 0)); // pelvis pitch
        rbdl.hip_pitch_id = rbdl.rbdl_model->RBDLModel::AddBody(2, RBDL::Math::Xtrans(RBDLVector3d(-0.067, 0, 0)), rbdl.hip_pitch_joint, rbdl.hip_pitch_link);
    
        rbdl.knee_pitch_link = RBDLBody(0.39278, RBDLVector3d(-0.000615, -0.000375, -0.11931), rbdl.knee_pitch_inertia); // 기존
        rbdl.knee_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 1, 0));
        rbdl.knee_pitch_id = rbdl.rbdl_model->RBDLModel::AddBody(3, RBDL::Math::Xtrans(RBDLVector3d(0, 0.0565, -0.25)), rbdl.knee_pitch_joint, rbdl.knee_pitch_link);

        rbdl.ankle_pitch_link = RBDLBody(0.078575, RBDLVector3d(0.002517, 0.0003, -0.000387), rbdl.ankle_pitch_inertia); // 기존
        rbdl.ankle_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 1, 0));
        rbdl.ankle_pitch_id = rbdl.rbdl_model->RBDLModel::AddBody(4, RBDL::Math::Xtrans(RBDLVector3d(0, 0, -0.25)), rbdl.ankle_pitch_joint, rbdl.ankle_pitch_link);

        rbdl.ankle_roll_link = RBDLBody(0.96035, RBDLVector3d(0.03881 ,-0.000273, -0.019958), rbdl.ankle_roll_inertia);
        rbdl.ankle_roll_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(1, 0, 0));
        rbdl.ankle_roll_id = rbdl.rbdl_model->RBDLModel::AddBody(5, RBDL::Math::Xtrans(RBDLVector3d(0, 0, 0)), rbdl.ankle_roll_joint, rbdl.ankle_roll_link);

        rbdl.foot_link = RBDLBody(0, RBDLVector3d(0, 0, 0), rbdl.foot_inertia);
        rbdl.foot_joint = RBDLJoint(RBDL::JointType::JointTypeFixed);
        rbdl.foot_id = rbdl.rbdl_model->RBDLModel::AddBody(rbdl.ankle_roll_id, RBDL::Math::Xtrans(RBDLVector3d(0, 0, -0.051)), rbdl.foot_joint, rbdl.foot_link);
        std::cout << "set left leg" << std::endl;

    }

    void RobotDynamics::R_Air_Model(Air_RBDL &rbdl)
    {
        rbdl.hip_yaw_inertia = RBDLMatrix3d(0.0008989,   -1.17e-05,        2.3e-06, 
                                                      -1.17e-05,   0.0006077,         -7e-07, 
                                                        2.3e-06,      -7e-07,      0.0004679);

        rbdl.hip_roll_inertia = RBDLMatrix3d(0.0003885, 1.06e-05,  1.5e-06, 
                                                       1.06e-05,  0.0005126, 3.4e-06, 
                                                       1.5e-06,   3.4e-06,   0.0004593);

        rbdl.hip_pitch_inertia = RBDLMatrix3d( 0.0134846, 2.18e-05,   1.8e-05, 
                                                         2.18e-05,  0.0125046, -0.0016397,
                                                         1.8e-05,  -0.0016397,  0.0021808);

        rbdl.knee_pitch_inertia = RBDLMatrix3d(0.0022052,          -6e-07,    -1.16e-05,
                                                            -6e-07,       0.0021571,     -4.4e-06,
                                                         -1.16e-05,        -4.4e-06,    0.0001219);

        rbdl.ankle_pitch_inertia = RBDLMatrix3d(1.46e-05,       -5e-07,           8e-07, 
                                                            -5e-07,     5.67e-05,               0, 
                                                             8e-07,            0,        5.86e-05);

        rbdl.ankle_roll_inertia = RBDLMatrix3d( 0.0014,       -3e-07,   0.000498, 
                                                          -3e-07,    0.0026268,    5.4e-06,
                                                        0.000498,      5.4e-06,   0.002615);

        rbdl.foot_inertia = RBDLMatrix3d( 0,0,0,
                                                    0,0,0,
                                                    0,0,0);

        rbdl.hip_yaw_link = RBDLBody(0.72894, RBDLVector3d(0.001248, 0.000607, -0.000365), rbdl.hip_yaw_inertia); //기존 무게중심 위치
        rbdl.hip_yaw_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 0, 1)); // pelvis yaw
        rbdl.hip_yaw_id = rbdl.rbdl_model->RBDLModel::AddBody(0, RBDL::Math::Xtrans(RBDLVector3d(0, -0.075, -0.096)), rbdl.hip_yaw_joint, rbdl.hip_yaw_link);
        
        rbdl.hip_roll_link = RBDLBody(0.69039, RBDLVector3d(-0.061767, -0.00154, -8.2e-05), rbdl.hip_roll_inertia); // 기존
        rbdl.hip_roll_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(1, 0, 0)); // pelvis roll
        rbdl.hip_roll_id = rbdl.rbdl_model->RBDLModel::AddBody(1, RBDL::Math::Xtrans(RBDLVector3d(0, 0, 0)), rbdl.hip_roll_joint, rbdl.hip_roll_link);

        rbdl.hip_pitch_link = RBDLBody(2.0087, RBDLVector3d(0.00035, -0.084719, -0.072582), rbdl.hip_pitch_inertia); // 기존
        rbdl.hip_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 1, 0)); // pelvis pitch
        rbdl.hip_pitch_id = rbdl.rbdl_model->RBDLModel::AddBody(2, RBDL::Math::Xtrans(RBDLVector3d(-0.05, 0, 0)), rbdl.hip_pitch_joint, rbdl.hip_pitch_link);
    
        rbdl.knee_pitch_link = RBDLBody(0.3923, RBDLVector3d(-0.000616 ,0.000374 ,-0.11915), rbdl.knee_pitch_inertia); // 기존
        rbdl.knee_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 1, 0));
        rbdl.knee_pitch_id = rbdl.rbdl_model->RBDLModel::AddBody(3, RBDL::Math::Xtrans(RBDLVector3d(0, -0.0565, -0.25)), rbdl.knee_pitch_joint, rbdl.knee_pitch_link);

        rbdl.ankle_pitch_link = RBDLBody(0.079061, RBDLVector3d(0.002501, -0.000298 ,-0.000384), rbdl.ankle_pitch_inertia); // 기존
        rbdl.ankle_pitch_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(0, 1, 0));
        rbdl.ankle_pitch_id = rbdl.rbdl_model->RBDLModel::AddBody(4, RBDL::Math::Xtrans(RBDLVector3d(0, 0, -0.25)), rbdl.ankle_pitch_joint, rbdl.ankle_pitch_link);

        rbdl.ankle_roll_link = RBDLBody(0.96035, RBDLVector3d(0.03881, 0.000273 ,-0.019958), rbdl.ankle_roll_inertia);
        rbdl.ankle_roll_joint = RBDLJoint(RBDL::JointType::JointTypeRevolute, RBDLVector3d(1, 0, 0));
        rbdl.ankle_roll_id = rbdl.rbdl_model->RBDLModel::AddBody(5, RBDL::Math::Xtrans(RBDLVector3d(0, 0, 0)), rbdl.ankle_roll_joint, rbdl.ankle_roll_link);

        rbdl.foot_link = RBDLBody(0, RBDLVector3d(0, 0, 0), rbdl.foot_inertia);
        rbdl.foot_joint = RBDLJoint(RBDL::JointType::JointTypeFixed);
        rbdl.foot_id = rbdl.rbdl_model->RBDLModel::AddBody(rbdl.ankle_roll_id, RBDL::Math::Xtrans(RBDLVector3d(0, 0, -0.051)), rbdl.foot_joint, rbdl.foot_link);
   
        std::cout << "set right leg" << std::endl;
    }

    void RobotDynamics::CalcFeedbackPose(Air_RBDL &rbdl)
    {
    
        RBDLVector3d FootPose;
        RBDLMatrix3d R, R_Tmp;
        RBDLMatrixNd EndJacobian, EndJacobian_tmp, EndBmatrix;

        EndJacobian = RBDLMatrixNd::Zero(6,6);
        EndJacobian_tmp = RBDLMatrixNd::Zero(6,6);
        EndBmatrix = RBDLMatrixNd::Zero(6,6);        

        FootPose= CalcBodyToBaseCoordinates(*rbdl.rbdl_model, rbdl.q, rbdl.foot_id, RBDLVector3d(0, 0, 0), true);
    
        R_Tmp = CalcBodyWorldOrientation(*rbdl.rbdl_model, rbdl.q, rbdl.foot_id, true);

        R = R_Tmp.transpose();

        // Get the Euler Angle - pi, theta, psi
        pi = atan2(R(1,0),R(0,0));
        theta = atan2(-R(2,0), cos(pi)*R(0,0) + sin(pi)*R(1,0));
        psi = atan2(sin(pi)*R(0,2) - cos(pi)*R(1,2), -sin(pi)*R(0,1) + cos(pi)*R(1,1));

    
        CalcPointJacobian6D(*rbdl.rbdl_model, rbdl.q, rbdl.foot_id, RBDLVector3d(0, 0, 0), EndJacobian_tmp, true);
        
        // Chage the Row
        for(int j = 0; j < 6; j++)
        {
            for(int i = 0; i < 3; i++)
            {
                EndJacobian(i,j) = EndJacobian_tmp(i+3,j);  // linear
            }
            for(int i = 3; i < 6; i++)
            {
                EndJacobian(i,j) = EndJacobian_tmp(i-3,j);  // angular
            }
        }
        
        rbdl.EndJacobian_T = EndJacobian.transpose();
        
        
        // Calculate the Analytical Jacobian & Inverse of Analytical Jacobian
        EndBmatrix << 1, 0, 0, 0, 0, 0, 
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0, 
                    0, 0, 0, -sin(pi), cos(pi), 0, 
                    0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1;

        rbdl.A_Jacobian = EndBmatrix*EndJacobian;
        rbdl.Inv_A_Jacobian = rbdl.A_Jacobian.inverse();

        // Calculate the Jacobian dot
        for(int i = 0; i < 6; i++)
        {
            for(int j = 0 ; j< 6; j++)
            {
                rbdl.A_Jacobian_dot(i,j) = (rbdl.A_Jacobian(i,j) - rbdl.prev_A_Jacobian(i,j)) / inner_dt;
                rbdl.prev_A_Jacobian(i,j) = rbdl.A_Jacobian(i,j);
            }
        }
        
        rbdl.Foot_Pos(0) = FootPose(0);
        rbdl.Foot_Pos(1) = FootPose(1);   
        rbdl.Foot_Pos(2) = FootPose(2);
        rbdl.Foot_Pos(3) = psi;
        rbdl.Foot_Pos(4) = theta;
        rbdl.Foot_Pos(5) = pi;

        rbdl.Foot_Pos_dot = rbdl.A_Jacobian*rbdl.q_dot;

    }

    void RobotDynamics::CalcTaskSpaceTorque(Air_RBDL &rbdl)
    {
        
        RBDLVectorNd TaskspaceTorque, MapJointTorque; 
        TaskspaceTorque = RBDLVectorNd::Zero(6);
        MapJointTorque = RBDLVectorNd::Zero(6);
        

        for(uint8_t i=0; i<6; i++)
        {
            TaskspaceTorque(i) = gain_p_task_space(i) * (rbdl.refTask(i) - rbdl.Foot_Pos(i));
        }
        MapJointTorque = rbdl.A_Jacobian.transpose() * TaskspaceTorque; 
        NonlinearEffects(*rbdl.rbdl_model, rbdl.q, rbdl.q_dot, rbdl.tau, NULL);

        
        rbdl.torque_Task = MapJointTorque ; 


    }

    void RobotDynamics::GenerateTorqueJointSpacePD()//go to walkready 
    {
        RBDL::NonlinearEffects(*swing_L.rbdl_model, swing_L.q, swing_L.q_dot, swing_L.tau, NULL);
        RBDL::NonlinearEffects(*swing_R.rbdl_model, swing_R.q, swing_R.q_dot, swing_R.tau, NULL);
        
       

        for (uint8_t i=0; i<6; i++)
        {
            tau_gravity_compensation(i) = swing_L.tau(i);
            tau_gravity_compensation(i+6) = swing_R.tau(i);

            // tau_viscous_damping[i] = js_kd.real_gain_value[i] * th_dot_sma_filtered[i];
            // tau_viscous_damping[i+6] = js_kd.real_gain_value[i+6] * th_dot_sma_filtered[i+6];
        }
        for (uint8_t i=0; i<NUM_OF_JOINTS; i++) 
        {
            ref_th[0] = 0;
            ref_th[1] = ref_angle[1] + 0.325 * trajectory; ///--> 0
            ref_th[2] = ref_angle[2] + 0.650 * trajectory; ///-->0.523
            ref_th[3] = ref_angle[3] - 0.8 * trajectory; ///--> -0.798
            ref_th[4] = ref_angle[4] - 1.1 * trajectory; ///--> 0.412
            ref_th[5] = 0;


            ref_th[6] = 0;
            ref_th[7] = ref_angle[7] + 0.325 * trajectory; ///--> 0
            ref_th[8] = ref_angle[8] + 0.650 * trajectory; ///-->0.523
            ref_th[9] = ref_angle[9] - 0.8 * trajectory; ///--> -0.798
            ref_th[10] = ref_angle[10] - 1.1* trajectory; ///--> 0.312
            ref_th[11] = 0;
            
            
            for(int i =0; i<6; i++)
            {
                
               joint_torque[i] = js_kp.real_gain_value[i] * (ref_th[i] - th[i]) + gain_r[i]* tau_gravity_compensation[i] - gain_d_joint_space[i] * th_dot_sma_filtered[i]; //gravity compansation joint torque

            }
            
            // joint_torque[i] = js_kp.real_gain_value[i] * (ref_th[i] - th[i]) - js_kd.real_gain_value[i] * th_dot_sma_filtered[i]; //non-gravity compensation joint torque

            for (uint8_t i=0; i<NUM_OF_JOINTS; i++) 
            {
                pre_motion.pre_torque[i] = joint_torque[i];

            }
        }
        
    }

     void RobotDynamics::GenerateTorqueJointSpacePD2()
    {
        RBDL::NonlinearEffects(*swing_L.rbdl_model, swing_L.q, swing_L.q_dot, swing_L.tau, NULL);
        RBDL::NonlinearEffects(*swing_R.rbdl_model, swing_R.q, swing_R.q_dot, swing_R.tau, NULL);
       

        for (uint8_t i=0; i<6; i++)
        {
            tau_gravity_compensation(i) = 2.5 * sin(th[i]);
            //tau_gravity_compensation(i) = swing_L.tau(i);
            //tau_gravity_compensation(i+6) = swing_R.tau(i);

            // tau_viscous_damping[i] = js_kd.real_gain_value[i] * th_dot_sma_filtered[i];
            // tau_viscous_damping[i+6] = js_kd.real_gain_value[i+6] * th_dot_sma_filtered[i+6];
        }
        for (uint8_t i=0; i<NUM_OF_JOINTS; i++) 
        {
            // ref_th[0] = 0;
            // // ref_th[1] = ref_angle[1] - 0.7 * trajectory2;
            // // ref_th[1] = ref_angle[1] + trajectory2 * (DEG2RAD * -90);
            // ref_th[1] = 0;
            // ref_th[2] = ref_angle[2] + 1.396 * trajectory2; //-->0.872 , 0.650 , 0~80
            // ref_th[2] = ref_angle[2] + 1.396 * trajectory2; //-->0.872 , 0.650 , 0~80
            // ref_th[3] = ref_angle[3] - 2.3 * trajectory2; // -->1.459
            // ref_th[4] = ref_angle[4] - 1 * trajectory2; //-->1.412
            // ref_th[5] = 0;


            // ref_th[1] = ref_angle[1] - 0.7 * trajectory2;
            // ref_th[1] = ref_angle[1] + trajectory2 * (DEG2RAD * -90);
            ref_th[0] = trajectory2 * (DEG2RAD * 90);
            ref_th[1] = trajectory2 * (DEG2RAD * 90);
            ref_th[2] = trajectory2 * (DEG2RAD * 90);
            ref_th[3] = trajectory2 * (DEG2RAD * -90);
            ref_th[4] = trajectory2 * (DEG2RAD * 90);
            ref_th[5] = trajectory2 * (DEG2RAD * 90);


            //ref_th[6] = 0;
            //ref_th[7] = 0;
            //ref_th[8] = ref_angle[8] + 0.650 * trajectory2;
            //ref_th[9] = ref_angle[9] - 0.8 * trajectory2;
            //ref_th[10] = ref_angle[10] - 1.3 * trajectory2;
            //ref_th[11] = 0;

            ref_th[6] = trajectory2 ;
            ref_th[7] = trajectory2 ;
            ref_th[8] = trajectory2;
            ref_th[9] = trajectory2;
            ref_th[10] = trajectory2;
            ref_th[11] = trajectory2 ;
            
            
            joint_torque[i] = js_kp.real_gain_value[i] * (ref_th[i] - th[i]) +  gain_r[i]* tau_gravity_compensation[i] - gain_d_joint_space[i] * th_dot_sma_filtered[i]; //gravity compansation joint torque
            // joint_torque[i] = js_kp.real_gain_value[i] * (ref_th[i] - th[i]) - js_kd.real_gain_value[i] * th_dot_sma_filtered[i]; //non-gravity compensation joint torque
            
            
        }
        for (uint8_t i=0; i<NUM_OF_JOINTS; i++) 
        {
            pre_motion.pre_torque[i] = joint_torque[i];

        }
    

    }

    void RobotDynamics::GenerateTorqueJointSpacePDForMotorTest()
    {        


        float step_time = 6;
        float count_time = motor_test_count * dt;
        
        int i = 0; // Motor id for testing 
        float tau_gravity_compensation = 5 * sin(th[i]); // -G * KG * 0.25m // 
        ref_th[i] = sin(PI*(count_time/step_time)) * PI * 0.5 ;
      
        // joint_torque[i] =  gain_r[i] * tau_gravity_compensation 
        //                 + js_kp.real_gain_value[i] * (ref_th[i] - th[i]) 
        //                 - gain_d_joint_space[i] * filtered_th_dot[i]; 

         joint_torque[i] =  gain_r[i] * tau_gravity_compensation 
                        + js_kp.real_gain_value[i] * (ref_th[i] - th[i]) 
                        - gain_d_joint_space[i] * filtered_th_dot[i]; 

        motor_test_count++;
        std::cout<<"gain:r" <<gain_r[i]<<std::endl;
        std::cout<<"tau_gravity" <<tau_gravity_compensation<<std::endl;
        std::cout<<"gain_d" <<gain_d_joint_space[i]<<std::endl;
        std::cout<<"LPF_velocity" <<filtered_th_dot[i]<<std::endl;

        
    }

    void RobotDynamics::GenerateTorqueGravityCompensation()
    {
        NonlinearEffects(*swing_L.rbdl_model, swing_L.q, swing_L.q_dot, swing_L.tau, NULL);
        NonlinearEffects(*swing_R.rbdl_model, swing_R.q, swing_R.q_dot, swing_R.tau, NULL);
        
        
        for(uint8_t i = 0; i < 6; i++)
        {
            tau_gravity_compensation(i) = swing_L.tau(i);
            tau_gravity_compensation(i+6) = swing_R.tau(i);
            
            tau_viscous_damping[i] = gain_d_joint_space[i] * filtered_th_dot[i];
            tau_viscous_damping[i+6] = gain_d_joint_space[i+6] * filtered_th_dot[i+6];
        }
        for(int i = 0; i<6; i++)
        {
            joint_torque(i) = gain_r(i) * tau_gravity_compensation(i)-tau_viscous_damping(i) + torque_F(i); 
            joint_torque(i+6) = gain_r(i+6) * tau_gravity_compensation(i+6)-tau_viscous_damping(i+6); 
        }    
        std::cout<< swing_L.q(4)<<std::endl;
            
        
    }

    void RobotDynamics::GenerateTorqueTaskSpacePD()
    {
        cnt_time = cnt*inner_dt;

        A0 <<   1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        A1 <<  0, -1, 0,      0,
               1,  0, 0,  0.075,
               0,  0, 1, -0.096,
               0,  0, 0,      1;
        
        A2 <<  cos(swing_L.q(0)),     0,     sin(swing_L.q(0)),      0,
               sin(swing_L.q(0)),     0,    -cos(swing_L.q(0)),      0,
                        0,            1,              0,             0,
                        0,            0,              0,             1;
        
        A3 <<  -sin(swing_L.q(1)),    0,       cos(swing_L.q(1)),    0,
                cos(swing_L.q(1)),    0,       sin(swing_L.q(1)),    0,
                         0,           1,                0,           -0.067,
                         0,           0,                0,           1;


        A4 <<   cos(swing_L.q(2)), -sin(swing_L.q(2)),  0,   (-0.25)*cos(swing_L.q(2)),
                sin(swing_L.q(2)),  cos(swing_L.q(2)),  0,   (-0.25)*sin(swing_L.q(2)),
                         0,           0,                1,              0.0565,
                         0,           0,                0,                 1;

        A5 <<   cos(swing_L.q(3)), -sin(swing_L.q(3)),  0,   (-0.25)*cos(swing_L.q(3)),
                sin(swing_L.q(3)),  cos(swing_L.q(3)),  0,   (-0.25)*sin(swing_L.q(3)),
                          0,          0,                1,                 0,
                          0,          0,                0,                 1;     

        A6 <<   cos(swing_L.q(4)),   0,    -sin(swing_L.q(4)),        0,
                sin(swing_L.q(4)),   0,     cos(swing_L.q(4)),        0,
                         0,         -1,              0,               0,
                         0,          0,              0,               1;

        A7 <<   cos(swing_L.q(5)),   0,      sin(swing_L.q(5)),      (-0.051)*cos(swing_L.q(5)),
                sin(swing_L.q(5)),   0,     -cos(swing_L.q(5)),      (-0.051)*sin(swing_L.q(5)),
                         0,          1,               0,                         0,
                         0,          0,               0,                         1;

        A8 <<  0,0,1,0,
               1,0,0,0,
               0,1,0,0,
               0,0,0,1;

        // std::cout << "HT matrix half success." << std::endl;    

        T00 = A0*A1;
        T01 = T00*A2;
        T02 = T01*A3;
        T03 = T02*A4;
        T04 = T03*A5;
        T05 = T04*A6;
        T06 = T05*A7*A8;
    
        a0 << T00(0,2), T00(1,2), T00(2,2);  //z axis unit vector  
        a1 << T01(0,2), T01(1,2), T01(2,2);
        a2 << T02(0,2), T02(1,2), T02(2,2);
        a3 << T03(0,2), T03(1,2), T03(2,2);
        a4 << T04(0,2), T04(1,2), T04(2,2);
        a5 << T05(0,2), T05(1,2), T05(2,2);

        P6_P0 << T06(0,3)-T00(0,3), T06(1,3)-T00(1,3), T06(2,3)-T00(2,3); 
        P6_P1 << T06(0,3)-T01(0,3), T06(1,3)-T01(1,3), T06(2,3)-T01(2,3);
        P6_P2 << T06(0,3)-T02(0,3), T06(1,3)-T02(1,3), T06(2,3)-T02(2,3);
        P6_P3 << T06(0,3)-T03(0,3), T06(1,3)-T03(1,3), T06(2,3)-T03(2,3);
        P6_P4 << T06(0,3)-T04(0,3), T06(1,3)-T04(1,3), T06(2,3)-T04(2,3);
        P6_P5 << T06(0,3)-T05(0,3), T06(1,3)-T05(1,3), T06(2,3)-T05(2,3);  

        J1 << a0.cross(P6_P0), a0;
        J2 << a1.cross(P6_P1), a1;
        J3 << a2.cross(P6_P2), a2;
        J4 << a3.cross(P6_P3), a3;
        J5 << a4.cross(P6_P4), a4;
        J6 << a5.cross(P6_P5), a5;

        Jacobian << J1, J2, J3, J4, J5, J6;

        ee_position << T06(0,3), T06(1,3), T06(2,3);

        ref_ee_position = footpad_position; //footpad position is trajectory position 
        
        ref_ee_quaternion.w() = qw; ref_ee_quaternion.x() = qx; ref_ee_quaternion.y() = qy; ref_ee_quaternion.z() = qz;      

        // std::cout << "Trajectory success." << std::endl;          

        ee_force(0) = gain_p_task_space(0) * (ref_ee_position(0) - ee_position(0));
        ee_force(1) = gain_p_task_space(1) * (ref_ee_position(1) - ee_position(1));
        ee_force(2) = gain_p_task_space(2) * (ref_ee_position(2) - ee_position(2));

        ee_rotation = T06.block<3,3>(0,0);
        ee_rotation_x = ee_rotation.block<3,1>(0,0); 
        ee_rotation_y = ee_rotation.block<3,1>(0,1); 
        ee_rotation_z = ee_rotation.block<3,1>(0,2);
        
        ref_ee_rotation = ref_ee_quaternion.normalized().toRotationMatrix();    

        ref_ee_rotation_x = ref_ee_rotation.block<3,1>(0,0); 
        ref_ee_rotation_y = ref_ee_rotation.block<3,1>(0,1); 
        ref_ee_rotation_z = ref_ee_rotation.block<3,1>(0,2);

        ee_orientation_error = ee_rotation_x.cross(ref_ee_rotation_x) + 
                               ee_rotation_y.cross(ref_ee_rotation_y) + 
                               ee_rotation_z.cross(ref_ee_rotation_z);

        ee_momentum << gain_p_task_space(3) * ee_orientation_error(0), 
                       gain_p_task_space(4) * ee_orientation_error(1), 
                       gain_p_task_space(5) * ee_orientation_error(2);

        virtual_spring << ee_force(0), ee_force(1), ee_force(2), ee_momentum(0), ee_momentum(1), ee_momentum(2);
    
        tau = Jacobian.transpose()*virtual_spring;

        NonlinearEffects(*swing_L.rbdl_model, swing_L.q, swing_L.q_dot, swing_L.tau, NULL);

        double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));
        
        for(uint8_t i=0; i<6; i++) 
        {
            tau_gravity_compensation(i) = swing_L.tau(i);
            tau_viscous_damping[i] = gain_d_task_space[i] * filtered_th_dot[i]; 
        
        }
        if(cnt_time == 0 )
        {
            for(int i=0; i<3; i++)
            {
             initial_ee_position[i] = ee_position(i);
            }
        }
        
        if(cnt_time <= step_time * 16)
        {
            footpad_position(0) = initial_ee_position[0];
            footpad_position(1) = initial_ee_position[1];
            footpad_position(2) = initial_ee_position[2]; //+ (0.1)*new_trajectory;
            cnt++;
        }

        for(int i =0; i<6; i++)
        {
            joint_torque(i) = tau(i) - tau_viscous_damping(i) + gain_r(i) * tau_gravity_compensation(i);      
            
            
        }   
        
      
  
 
        
    }

    void RobotDynamics::CalcCTCTorque(Air_RBDL &rbdl)
    {
        

        RBDLVectorNd Task_ddot,Joint_ddot;
        Task_ddot = RBDLVectorNd::Zero(6);
        Joint_ddot = RBDLVectorNd::Zero(6);

        RBDLMatrixNd I_Matrix;
        I_Matrix = RBDLMatrixNd::Zero(6,6);

    

        for(int i = 0; i < 6; i++)
        {
            Task_ddot(i) = rbdl.Des_XDDot(i) + gain_p_CTC(i) * (rbdl.Des_X(i) - rbdl.Foot_Pos(i)) + gain_d_CTC(i) * (rbdl.Des_XDot(i) - rbdl.Foot_Pos_dot(i));
        }
        

        Joint_ddot = rbdl.Inv_A_Jacobian * (Task_ddot - rbdl.A_Jacobian_dot*rbdl.q_dot); 
        
    
        NonlinearEffects(*rbdl.rbdl_model, rbdl.q, rbdl.q_dot, rbdl.tau, NULL);


        CompositeRigidBodyAlgorithm(*rbdl.rbdl_model, rbdl.q, I_Matrix, true);
        
    
        rbdl.torque_CTC = I_Matrix * Joint_ddot ; 
    }

    void RobotDynamics::TaskSpaceMotion() 
    {
        
        cnt_time = cnt*inner_dt;
        RBDLVectorNd ActualJointDamping;
        ActualJointDamping = RBDLVectorNd :: Zero(12);

        double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

        CalcFeedbackPose(swing_L);
        CalcFeedbackPose(swing_R);

        if(cnt_time==0)
        {
            for(int i = 0; i < 6; i++)
            {
                swing_R.Init_PR(i) = swing_R.Foot_Pos(i);
                swing_L.Init_PR(i) = swing_L.Foot_Pos(i);
                std::cout<<swing_L.Init_PR(i)<<std::endl;
            }
        }
        

        if(cnt_time <=step_time*100)
        {
        
            swing_L.refTask(0) = swing_L.Init_PR(0);// + 0.2*(sin(PI*(cnt_time/step_time)));
            swing_L.refTask(1) = swing_L.Init_PR(1);
            swing_L.refTask(2) = swing_L.Init_PR(2) + 0.07*new_trajectory;
            swing_L.refTask(3) = swing_L.Init_PR(3);
            swing_L.refTask(4) = swing_L.Init_PR(4);
            swing_L.refTask(5) = swing_L.Init_PR(5);


            swing_R.refTask(0) = swing_R.Init_PR(0);// - 0.2*(sin(PI*(cnt_time/step_time)));
            swing_R.refTask(1) = swing_R.Init_PR(1);
            swing_R.refTask(2) = swing_R.Init_PR(2);// + 0.1*new_trajectory;
            swing_R.refTask(3) = swing_R.Init_PR(3);
            swing_R.refTask(4) = swing_R.Init_PR(4);
            swing_R.refTask(5) = swing_R.Init_PR(5);
            cnt++;
        }
        CalcTaskSpaceTorque(swing_L);
        CalcTaskSpaceTorque(swing_R);


        for(uint8_t i =0; i<6; i++ )
        {
            ActualJointDamping(i) = gain_d_task_space(i) * (0 - filtered_th_dot[i]);
            ActualJointDamping(i+6) = gain_d_task_space(i) * (0 - filtered_th_dot[i+6]);
            joint_torque(i) = (swing_L.torque_Task(i) + ActualJointDamping(i))+ gain_r(i)*swing_L.tau(i);
            joint_torque(i+6) = swing_R.torque_Task(i) + ActualJointDamping(i+6);

        }

        Ref_ee_position(0) = swing_L.refTask(0);
        Ref_ee_position(1) = swing_L.refTask(1);
        Ref_ee_position(2) = swing_L.refTask(2);
        Ref_ee_orientation(0) = swing_L.refTask(3);
        Ref_ee_orientation(1) = swing_L.refTask(4);
        Ref_ee_orientation(2) = swing_L.refTask(5);

        Actual_ee_position(0) = swing_L.Foot_Pos(0);
        Actual_ee_position(1) = swing_L.Foot_Pos(1);   
        Actual_ee_position(2) = swing_L.Foot_Pos(2);
        Actual_ee_orientation(0) = swing_L.Foot_Pos(3); 
        Actual_ee_orientation(1) = swing_L.Foot_Pos(4); 
        Actual_ee_orientation(2) = swing_L.Foot_Pos(5); 


    }

    void RobotDynamics::CTCMotion() 
    {
        double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));
        cnt_time = cnt*inner_dt;
        chg_time = chg_cnt*inner_dt; 

        CalcFeedbackPose(swing_L);
        CalcFeedbackPose(swing_R);

        if(cnt_time==0)
        {
            for(int i = 0; i < 6; i++)
            {
                swing_R.Init_PR(i) = swing_R.Foot_Pos(i);
                swing_L.Init_PR(i) = swing_L.Foot_Pos(i);
                std::cout<<swing_L.Init_PR(i)<<std::endl;

                swing_R.Des_XDDot(i) = 0;
                swing_R.Des_XDot(i) = 0;
                swing_L.Des_XDDot(i) = 0;
                swing_L.Des_XDot(i) = 0;
                // ref_th[1] = th(1);
            }
        }
        

        if(cnt_time <=step_time*100)
        {
        
            swing_L.Des_X(0) = swing_L.Init_PR(0);// - 0.07*new_trajectory;//- 0.1*(sin(PI*(cnt_time/step_time)));
            swing_L.Des_X(1) = swing_L.Init_PR(1);//- 0.07*new_trajectory;
            swing_L.Des_X(2) = swing_L.Init_PR(2) + 0.07*new_trajectory;
            swing_L.Des_X(3) = swing_L.Init_PR(3);
            swing_L.Des_X(4) = swing_L.Init_PR(4);
            swing_L.Des_X(5) = swing_L.Init_PR(5);


            swing_R.Des_X(0) = swing_R.Init_PR(0);// - 0.1*(sin(PI*(cnt_time/step_time)));
            swing_R.Des_X(1) = swing_R.Init_PR(1);
            swing_R.Des_X(2) = swing_R.Init_PR(2);// + 0.1*new_trajectory;
            swing_R.Des_X(3) = swing_R.Init_PR(3);
            swing_R.Des_X(4) = swing_R.Init_PR(4);
            swing_R.Des_X(5) = swing_R.Init_PR(5);
            cnt++;
        }
    

        CalcCTCTorque(swing_L);
        CalcCTCTorque(swing_R);

        for(uint8_t i =0; i<6; i++ )
        {
            tau_viscous_damping[i] = gain_d_task_space[i] * filtered_th_dot[i];
            //tau_viscous_damping[i+6] = gain_d_task_space[i] * filtered_th_dot[i+6];

            joint_torque(i) = (swing_L.torque_CTC(i))*tanh(5*chg_time/chg_step_time) + gain_r(i)*swing_L.tau(i) - tau_viscous_damping[i] + torque_F(i); 
            joint_torque(i+6) = (swing_R.torque_CTC(i))*tanh(5*chg_time/chg_step_time) + gain_r(i+6)*swing_R.tau(i);
        }
        chg_cnt++;
    
        
    }

    void RobotDynamics::FrictionTorque()
    {
        friction_ << 0.2, 0, 2.2, 1.2, 1.4, 0;

        for(uint8_t i=0; i<6; i++)
        {
            if(abs(filtered_th_dot(i))<friction_gap)
            {
                torque_F(i) = 0; 
                torque_F(i+6) = 0;
            }
            else
            {
                torque_F(i) = friction_(i) * tanh(filtered_th_dot(i) * friction_Kv(i)); 
                torque_F(i+6) = 0;
            }
        }
            


    }

}
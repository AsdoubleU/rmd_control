#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "motor_controller.h"


#include <eigen3/Eigen/Dense>

#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <rbdl/rbdl.h>
// #include <stob_control/CustomJointState.h>

//FILE *Experiment_data;

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;


namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

using RBDLModel = RBDL::Model;
using RBDLBody = RBDL::Body;
using RBDLVector3d = RBDL::Math::Vector3d;
using RBDLVectorNd = RBDL::Math::VectorNd;
using RBDLMatrixNd = RBDL::Math::MatrixNd;
using RBDLMatrix3d = RBDL::Math::Matrix3d;
using RBDLJoint = RBDL::Joint;


#define DEG2RAD			0.017453292519943
#define RAD2DEG			57.295779513082323
#define G						9.81
#define PI					3.1415

#define NUM_OF_JOINTS   12

#define inner_dt 0.002



#define M_Robot  19.320
#define M_base  4.9133
#define M_L_HIP_Y  0.9518
#define M_L_HIP_R  0.8580
#define M_L_HIP_P  2.4902
#define M_L_KNEE_P  1.5349
#define M_L_ANKLE_P  0.3672
#define M_L_ANKLE_R  1.226
#define M_R_HIP_Y  0.9518
#define M_R_HIP_R  0.8580
#define M_R_HIP_P  2.4902
#define M_R_KNEE_P  1.5349
#define M_R_ANKLE_P  0.3672
#define M_R_ANKLE_R  1.226

typedef struct Air_rbdl_model_
{
  RBDLModel* rbdl_model;
  RBDLVectorNd q, q_dot, q_d_dot, prevQ, prevQDot, tau, Init_PR,Foot_Pos,  Foot_Pos_dot, Foot_Pos_dot_old, Des_X, Des_XDot, Des_XDDot, torque_CTC, 
               Old_Des_X, Old_Des_XDot, Old_Des_XDDot, New_Des_X, New_Des_XDot, New_Des_XDDot, Kp, Kv, Friction_Torque,refTask, torque_Task;
  RBDLMatrixNd A_Jacobian, prev_A_Jacobian, A_Jacobian_dot, Inv_A_Jacobian,EndJacobian_T;

  unsigned int  hip_yaw_id, hip_roll_id, hip_pitch_id, knee_pitch_id, ankle_pitch_id, ankle_roll_id, foot_id;//id have information of the body
  RBDLBody /*body_Base,*/ hip_yaw_link, hip_roll_link, hip_pitch_link, knee_pitch_link,  ankle_pitch_link, ankle_roll_link, foot_link;//make body.
  RBDLJoint /*joint_Base,*/ hip_yaw_joint, hip_roll_joint, hip_pitch_joint, knee_pitch_joint, ankle_pitch_joint, ankle_roll_joint, foot_joint; //make joint
  RBDLMatrix3d /*bodyI_Base,*/ hip_yaw_inertia, hip_roll_inertia, hip_pitch_inertia, knee_pitch_inertia, ankle_pitch_inertia, ankle_roll_inertia, foot_inertia;//Inertia of Body
} Air_RBDL;



typedef struct Gain_type_
{
    VectorXd old_gain_value         = VectorXd::Zero(12);
    VectorXd new_gain_value         = VectorXd::Zero(12);
    VectorXd real_gain_value        = VectorXd::Zero(12);
    VectorXd callback_gain_value    = VectorXd::Zero(12);    
    VectorXd delta_gain_value      = VectorXd::Zero(12);

    // float old_gain_value{0}, new_gain_value{0}, real_gain_value{0}, callback_gain_value{0}, delta_gain_value{0};
    int count{0}, max_count{2000};
    bool update_flag{false};
} Gain_type;

typedef struct Switch_value_
{   
    VectorXd pre_torque = VectorXd::Zero(12);
    VectorXd pre_th = VectorXd::Zero(12);
    VectorXd pre_th_dot = VectorXd::Zero(12);

    int change_count = 0, changetime = 700;
    bool update_flag{false};

}switch_value;

namespace Dynamics
{
    class RobotDynamics
    {


        VectorXd R_Pos_HIP_Y_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_HIP_R_CoM = VectorXd::Zero(3);
        VectorXd R_Pos_HIP_P_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_KNEE_P_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_ANKLE_P_CoM = VectorXd::Zero(3); 
        VectorXd R_Pos_ANKLE_R_CoM = VectorXd::Zero(3);

        VectorXd L_Pos_HIP_Y_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_HIP_R_CoM = VectorXd::Zero(3);
        VectorXd L_Pos_HIP_P_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_KNEE_P_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_ANKLE_P_CoM = VectorXd::Zero(3); 
        VectorXd L_Pos_ANKLE_R_CoM = VectorXd::Zero(3); 

        VectorXd Base_Pos_CoM = VectorXd::Zero(3); 

        VectorXd Robot_Pos_CoM = VectorXd::Zero(3); 

        MatrixXd L_T00 = MatrixXd::Zero(4,4);
        MatrixXd L_T01 = MatrixXd::Zero(4,4);
        MatrixXd L_T02 = MatrixXd::Zero(4,4);
        MatrixXd L_T03 = MatrixXd::Zero(4,4);
        MatrixXd L_T04 = MatrixXd::Zero(4,4);
        MatrixXd L_T05 = MatrixXd::Zero(4,4);
        MatrixXd L_T06 = MatrixXd::Zero(4,4);

        MatrixXd R_T00 = MatrixXd::Zero(4,4);
        MatrixXd R_T01 = MatrixXd::Zero(4,4);
        MatrixXd R_T02 = MatrixXd::Zero(4,4);
        MatrixXd R_T03 = MatrixXd::Zero(4,4);
        MatrixXd R_T04 = MatrixXd::Zero(4,4);
        MatrixXd R_T05 = MatrixXd::Zero(4,4);
        MatrixXd R_T06 = MatrixXd::Zero(4,4);

        Eigen::Matrix3d L_T00_rot;
        Eigen::Matrix3d L_T01_rot;
        Eigen::Matrix3d L_T02_rot;
        Eigen::Matrix3d L_T03_rot;
        Eigen::Matrix3d L_T04_rot;
        Eigen::Matrix3d L_T05_rot;
        Eigen::Matrix3d L_T06_rot;

        Eigen::Matrix3d R_T00_rot;
        Eigen::Matrix3d R_T01_rot;
        Eigen::Matrix3d R_T02_rot;
        Eigen::Matrix3d R_T03_rot;
        Eigen::Matrix3d R_T04_rot;
        Eigen::Matrix3d R_T05_rot;
        Eigen::Matrix3d R_T06_rot;

        VectorXd L_T00_pos = VectorXd::Zero(3); 
        VectorXd L_T01_pos = VectorXd::Zero(3); 
        VectorXd L_T02_pos = VectorXd::Zero(3); 
        VectorXd L_T03_pos = VectorXd::Zero(3); 
        VectorXd L_T04_pos = VectorXd::Zero(3);
        VectorXd L_T05_pos = VectorXd::Zero(3); 

        VectorXd R_T00_pos = VectorXd::Zero(3);
        VectorXd R_T01_pos = VectorXd::Zero(3);
        VectorXd R_T02_pos = VectorXd::Zero(3);
        VectorXd R_T03_pos = VectorXd::Zero(3);
        VectorXd R_T04_pos = VectorXd::Zero(3);
        VectorXd R_T05_pos = VectorXd::Zero(3);        

       

        MatrixXd R_HIP_Y_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_HIP_R_CoM = MatrixXd::Zero(4,4);
        MatrixXd R_HIP_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_KNEE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_ANKLE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd R_ANKLE_R_CoM = MatrixXd::Zero(4,4); 
        MatrixXd BASE_CoM  = MatrixXd::Zero(4,4);
        MatrixXd L_HIP_Y_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_HIP_R_CoM = MatrixXd::Zero(4,4);
        MatrixXd L_HIP_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_KNEE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_ANKLE_P_CoM = MatrixXd::Zero(4,4); 
        MatrixXd L_ANKLE_R_CoM = MatrixXd::Zero(4,4);

        MatrixXd L_A1 = MatrixXd::Zero(4,4); 
        MatrixXd L_A2 = MatrixXd::Zero(4,4); 
        MatrixXd L_A3 = MatrixXd::Zero(4,4);
        MatrixXd L_A4 = MatrixXd::Zero(4,4);
        MatrixXd L_A5 = MatrixXd::Zero(4,4);
        MatrixXd L_A6 = MatrixXd::Zero(4,4);
        MatrixXd L_A7 = MatrixXd::Zero(4,4);
        MatrixXd L_A8 = MatrixXd::Zero(4,4);
        MatrixXd L_A9 = MatrixXd::Zero(4,4);
        MatrixXd L_A10 = MatrixXd::Zero(4,4);  

        MatrixXd R_A1 = MatrixXd::Zero(4,4); 
        MatrixXd R_A2 = MatrixXd::Zero(4,4); 
        MatrixXd R_A3 = MatrixXd::Zero(4,4);
        MatrixXd R_A4 = MatrixXd::Zero(4,4);
        MatrixXd R_A5 = MatrixXd::Zero(4,4);
        MatrixXd R_A6 = MatrixXd::Zero(4,4);
        MatrixXd R_A7 = MatrixXd::Zero(4,4);
        MatrixXd R_A8 = MatrixXd::Zero(4,4);
        MatrixXd R_A9 = MatrixXd::Zero(4,4);
        MatrixXd R_A10 = MatrixXd::Zero(4,4);

        VectorXd J1 = VectorXd::Zero(6); 
        VectorXd J2 = VectorXd::Zero(6); 
        VectorXd J3 = VectorXd::Zero(6); 
        VectorXd J4 = VectorXd::Zero(6); 
        VectorXd J5 = VectorXd::Zero(6); 
        VectorXd J6 = VectorXd::Zero(6); 
        VectorXd J11 = VectorXd::Zero(6); 
        VectorXd J12 = VectorXd::Zero(6); 
        VectorXd J13 = VectorXd::Zero(6); 
        VectorXd J14 = VectorXd::Zero(6); 
        VectorXd J15 = VectorXd::Zero(6); 
        VectorXd J16 = VectorXd::Zero(6); 

        MatrixXd Jacobian = MatrixXd::Zero(6,6); 
        MatrixXd Jacobian_R = MatrixXd::Zero(6,6); 

        Eigen::Vector3d r_ee_rotation_x, r_ee_rotation_y, r_ee_rotation_z, 
                r_ref_ee_rotation_x, r_ref_ee_rotation_y, r_ref_ee_rotation_z,
                r_ee_orientation_error, r_ee_force, r_ee_momentum;

        Eigen::Matrix3d ee_rotation, ref_ee_rotation;
        Eigen::Matrix3d r_ee_rotation, r_ref_ee_rotation;

        Quaterniond ref_ee_quaternion;  
        Quaterniond ee_quaternion;  
        Quaterniond r_ref_ee_quaternion;  
        Quaterniond r_ee_quaternion; 

        MatrixXd A0 = MatrixXd::Zero(4,4);  
        MatrixXd A1 = MatrixXd::Zero(4,4); 
        MatrixXd A2 = MatrixXd::Zero(4,4); 
        MatrixXd A3 = MatrixXd::Zero(4,4);
        MatrixXd A4 = MatrixXd::Zero(4,4);
        MatrixXd A5 = MatrixXd::Zero(4,4);
        MatrixXd A6 = MatrixXd::Zero(4,4);
        MatrixXd A7 = MatrixXd::Zero(4,4);
        MatrixXd A8 = MatrixXd::Zero(4,4);
        
        MatrixXd A10 = MatrixXd::Zero(4,4); 
        MatrixXd A11 = MatrixXd::Zero(4,4); 
        MatrixXd A12 = MatrixXd::Zero(4,4);
        MatrixXd A13 = MatrixXd::Zero(4,4);
        MatrixXd A14 = MatrixXd::Zero(4,4);
        MatrixXd A15 = MatrixXd::Zero(4,4);
        MatrixXd A16 = MatrixXd::Zero(4,4);
        MatrixXd A17 = MatrixXd::Zero(4,4);
        MatrixXd A18 = MatrixXd::Zero(4,4);

        MatrixXd T00 = MatrixXd::Zero(4,4); 
        MatrixXd T01 = MatrixXd::Zero(4,4); 
        MatrixXd T02 = MatrixXd::Zero(4,4); 
        MatrixXd T03 = MatrixXd::Zero(4,4);
        MatrixXd T04 = MatrixXd::Zero(4,4);
        MatrixXd T05 = MatrixXd::Zero(4,4);
        MatrixXd T06 = MatrixXd::Zero(4,4);
        MatrixXd T07 = MatrixXd::Zero(4,4);
        MatrixXd T08 = MatrixXd::Zero(4,4);
        MatrixXd T09 = MatrixXd::Zero(4,4);
        MatrixXd T10 = MatrixXd::Zero(4,4);

        Eigen::Vector3d a0, a1, a2, a3, a4, a5;
        Eigen::Vector3d a11, a12, a13, a14, a15;
        Eigen::Vector3d P6_P0, P6_P1, P6_P2, P6_P3, P6_P4, P6_P5;
        Eigen::Vector3d P16_P0, P16_P11, P16_P12, P16_P13, P16_P14, P16_P15;


        VectorXd virtual_spring = VectorXd::Zero(6);
        VectorXd r_virtual_spring = VectorXd::Zero(6);


        double dt = 0.002;
        double time = 0;
        double trajectory = 0;
        double trajectory2 = 0;

        double cnt_time = 0;
        unsigned int cnt = 0;   
        double chg_step_time = 0.04;
        double chg_cnt_time = 0;
        double chg_time = 0;
        unsigned int chg_cnt = 0;

        Vector3d ee_rotation_x, ee_rotation_y, ee_rotation_z, ref_ee_rotation_x, ref_ee_rotation_y, ref_ee_rotation_z;
        Vector3d ee_orientation_error, ee_force, ee_momentum;


        VectorXd tau = VectorXd::Zero(6);
        VectorXd r_tau = VectorXd::Zero(6);

        VectorXd ref_angle = VectorXd::Zero(12);


        MatrixXd joint_limit = MatrixXd::Zero(2,NUM_OF_JOINTS);
        MatrixXd torque_limit = MatrixXd::Zero(2,NUM_OF_JOINTS);

        VectorXd threshold = VectorXd(NUM_OF_JOINTS);

        

        std::vector<double> present_joint_angle_;

        enum ControlMode
        {
            gravity_compensation = 4,
            joint_space = 1,
            joint_space2 = 2,
            task_space = 3,
            TASKSPACEMOTION = 5,
            CTCMOTION = 6,
            clear_torque =7,
            motor_test = 10
        };
        enum ControlMode control_mode;

    public:
        RobotDynamics();
        //~RobotDynamics();

        Air_RBDL swing_L, swing_R;
    
        float qw = 1;
        float qx = 0;
        float qy = 0;
        float qz = 0;
        float inner_gain_p = 100;
        float inner_gain_i = 10;
        float friction_gap = 0.012;

        double pi, theta, psi=0;
       
        VectorXd th = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd ref_th = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd th_dot = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd th_dot_sma_filtered = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd filtered_th_dot = VectorXd::Zero(NUM_OF_JOINTS);

        VectorXd last_th_dot = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd th_d_dot = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd tau_gravity_compensation = VectorXd::Zero(NUM_OF_JOINTS);

        Eigen::Vector3d ee_position, ee_velocity, pre_ee_position, ref_ee_position, initial_ee_position, footpad_position, ee_position_last;
        Eigen::Vector3d r_ee_position, r_ee_velocity, r_pre_ee_position, r_ref_ee_position, r_initial_ee_position, r_footpad_position, r_ee_position_last;

        Eigen::Vector3d Ref_ee_position,Ref_ee_orientation,Actual_ee_position, Actual_ee_orientation;

    

        int motor_test_count = 0;
        int count = 0;
        int count_for_print_to_file = 0;
        double step_time = 3;
        int taskspace_time = 8;
        int taskspace_count = 0;

        bool record_data = false;

        Gain_type js_kp, js_kd,js_gp;
        switch_value pre_motion;

        Vector3d gain_p, gain_d, gain_w;
        VectorXd gain_r = VectorXd(NUM_OF_JOINTS);
        VectorXd zero_gain = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd msg_torque = VectorXd::Zero(6);

        VectorXd joint_torque = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd torque_F = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd tau_viscous_damping = VectorXd::Zero(NUM_OF_JOINTS);


        VectorXd gain_p_joint_space = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd gain_d_joint_space = VectorXd::Zero(NUM_OF_JOINTS);

        VectorXd gain_Kp = VectorXd::Zero(NUM_OF_JOINTS);
        VectorXd gain_Kv = VectorXd::Zero(NUM_OF_JOINTS);

        VectorXd gain_p_task_space = VectorXd::Zero(6);
        VectorXd gain_d_task_space = VectorXd::Zero(6);
        Vector3d gain_w_task_space;

        VectorXd gain_p_CTC = VectorXd::Zero(6);
        VectorXd gain_d_CTC = VectorXd::Zero(6);

        VectorXd friction_Kv = VectorXd::Zero(6);

      
        Eigen::VectorXd friction_ = VectorXd::Zero(6);


        VectorXd GetTorque();
        void SetTheta(VectorXd thetas);
        void SetThetaDot(VectorXd);
        void SetThetaDotSMAF(VectorXd);
        void ClearTorque();
        void CalculateFK();
        void GenerateTorqueJointSpacePD();
        void GenerateTorqueJointSpacePD2();
        void GenerateTorqueJointSpacePDForMotorTest();
        void GenerateTaskSpaceJointSpacePD();
        void GenerateTorqueTaskSpacePD();
        void GenerateTrajectory();
        void GenerateTorqueGravityCompensation();
        void PostureGeneration();
        void Loop();
        void SwitchMode(const std_msgs::Int32ConstPtr & msg);

        void SetRBDLVariables();
        void InitializeRBDLVariables();
        void UpdateGainValues();
        void PrintToFile();
        void SwitchRecording();
        void L_Air_Model(Air_RBDL &rbdl);
        void R_Air_Model(Air_RBDL &rbdl);
        void rbdl_variable_init(Air_RBDL &rbdl);
        void CalcFeedbackPose(Air_RBDL &rbdl);
        void CalcTaskSpaceTorque(Air_RBDL &rbdl);
        void CalcCTCTorque(Air_RBDL &rbdl);
        void CTCMotion();
        void TaskSpaceMotion();
        void FrictionTorque();


        FILE *Position_data;
        FILE *Experiment_data;
    };
}


#endif // DYNAMICS_H
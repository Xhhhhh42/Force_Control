#pragma once

#include <Eigen/Dense>
#include <memory>

#include <force_control/state_in_joint_space.hpp>
#include <force_control/state_in_task_space.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>

// // 预定义的控制参数
// const Eigen::Matrix3d Kd = Eigen::Matrix3d::Identity() * 100.0;  // 刚度矩阵
// const Eigen::Matrix3d Dd = Eigen::Matrix3d::Identity() * 20.0;   // 阻尼矩阵
// const Eigen::Matrix3d Lambda_d = Eigen::Matrix3d::Identity() * 0.1;    // 惯性矩阵
// const Eigen::Matrix<double, 7, 7> K_D = Eigen::Matrix<double, 7, 7>::Identity() * 1;

namespace force_control {

class Task_Space_Controller
{
public:
    Task_Space_Controller( double &Lambda_d_ts, double &K_d_ts, double &D_d_ts, double &K_D_ts );

    ~Task_Space_Controller() {};

    bool init( std::shared_ptr<State_in_Joint_Space> &state_in_JS );

    void update_Task_State();

    Eigen::Matrix<double, 7, 1> update();

public:
    bool updateTaskSpaceControl(
        const Eigen::Matrix3d& Lambda, const Eigen::Vector3d& x, const Eigen::Vector3d& xd, 
        const Eigen::Vector3d& x_dot, const Eigen::Vector3d& x_dot_d, 
        const Eigen::Vector3d& x_ddot_d, const Eigen::Vector3d& Fext );

    inline Eigen::Vector3d calculateError(const Eigen::Vector3d& x, const Eigen::Vector3d& xd) 
        { return x - xd; }

    Eigen::Matrix<double, 7, 1> control_torque_in_JointSpace( Eigen::Matrix<double, 3, 7> &jacobian_, Eigen::Vector3d &F_c );

    Eigen::Matrix<double, 7, 1> add_null_space_damping( Eigen::Matrix<double, 7, 1> &tau_c, const Eigen::Matrix<double, 7, 7> &K_D, Eigen::Matrix<double, 7, 3> &jacobian_pseudo_inverse );

    // State
    std::shared_ptr<State_in_Joint_Space> state_in_JS_;
    std::shared_ptr<State_in_Task_Space> state_in_TS_;

    Eigen::Vector3d x_; 
    Eigen::Vector3d x_dot_; 
    Eigen::Vector3d x_ddot_; 
    Eigen::Vector3d xd_; // 设定期望位置
    Eigen::Vector3d x_dot_d_; // 期望速度
    Eigen::Vector3d x_ddot_d_; // 期望加速度
    Eigen::Quaterniond orientation_d_;
    Eigen::Quaterniond orientation_;

    //
    Eigen::Vector3d print1;
    Eigen::Vector3d print2;
    Eigen::Matrix3d print_lambda;
    Eigen::Vector3d print1_1;
    Eigen::Vector3d print1_2;
    Eigen::Vector3d print1_3;

    bool inited_;

    std::ofstream out;

    Eigen::Vector3d last_Fex_;
    Eigen::Matrix3d Lambda_d_;  // 刚度矩阵
    Eigen::Matrix3d Kd_;   // 阻尼矩阵
    Eigen::Matrix3d Dd_;    // 惯性矩阵
    Eigen::Matrix<double, 7, 7> K_D_;
};


}  // namespace force_control
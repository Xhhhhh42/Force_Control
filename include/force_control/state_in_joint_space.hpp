#pragma once

#include <Eigen/Dense>

namespace force_control {


class State_in_Joint_Space
{
public:
    State_in_Joint_Space();

    ~State_in_Joint_Space() {};

    bool update( Eigen::Matrix<double, 7, 7> &joint_mass_matrix, Eigen::Matrix<double, 7, 1> &joint_coriolis_force, 
                 Eigen::Matrix<double, 7, 1> &joint_gravity_force, Eigen::Matrix<double, 4, 4> &pose, 
                 Eigen::Matrix<double, 6, 7> &zero_jacobian, Eigen::Matrix<double, 3, 7> &jacobian );

    bool update_Pseudo_inverse( Eigen::Matrix<double, 7, 3> &jacobian_pseudo_inverse );

    bool joint_state_init( Eigen::Matrix<double, 7, 1> &initial_q, Eigen::Matrix<double, 4, 4> &init_pose );

    bool update_joint_state( Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq );

    bool update_wrench( Eigen::Matrix<double, 6, 1> &wrench_ext );

    bool update_base_T_EE( Eigen::Matrix<double, 4, 4> &O_T_EE );

    bool init_base_T_EE( Eigen::Matrix<double, 4, 4> &init_O_T_EE );

public:
    // Joint_State
    Eigen::Matrix<double, 7, 7> joint_mass_matrix_;
    Eigen::Matrix<double, 7, 1> joint_coriolis_force_;
    Eigen::Matrix<double, 7, 1> joint_gravity_force_;
    // Eigen::Matrix<double, 7, 1> joint_friction_force_; 
    Eigen::Matrix<double, 4, 4> pose_;
    Eigen::Matrix<double, 4, 4> init_pose_;
    Eigen::Matrix<double, 6, 7> zero_jacobian_;
    Eigen::Matrix<double, 3, 7> jacobian_;
    Eigen::Matrix<double, 7, 3> jacobian_pseudo_inverse_;

    Eigen::Matrix<double, 7, 1> q_;
    Eigen::Matrix<double, 7, 1> dq_;
    Eigen::Matrix<double, 7, 1> initial_q_;

    // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame.
    Eigen::Matrix<double, 6, 1> wrench_ext_;

    Eigen::Matrix<double, 4, 4> O_T_EE_;
    Eigen::Matrix<double, 4, 4> init_O_T_EE_;
};

}  // namespace force_control
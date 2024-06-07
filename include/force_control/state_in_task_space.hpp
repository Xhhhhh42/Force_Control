#pragma once

#include <Eigen/Dense>

namespace force_control {

using Vector7d = Eigen::Matrix<double, 7, 1>;

class State_in_Task_Space
{
public:
    State_in_Task_Space();

    ~State_in_Task_Space() {};

    bool update( Eigen::Matrix<double, 3, 3> &task_lambda_matrix, Eigen::Vector3d &task_coriolis_force, Eigen::Vector3d &task_gravity_force,
                 Eigen::Vector3d &task_friction_force, Eigen::Vector3d &F_c );

    bool update_Fc( Eigen::Vector3d &F_c );

public:
    // Task_State
    Eigen::Matrix<double, 3, 3> task_lambda_matrix_;
    Eigen::Vector3d task_coriolis_force_;
    Eigen::Vector3d task_gravity_force_; 
    Eigen::Vector3d task_friction_force_;
    Eigen::Vector3d F_c_;
};

}  // namespace force_control
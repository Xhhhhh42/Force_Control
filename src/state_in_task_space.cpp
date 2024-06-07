#include <force_control/state_in_task_space.hpp>

namespace force_control {

State_in_Task_Space::State_in_Task_Space()
    : task_lambda_matrix_(Eigen::Matrix<double, 3, 3>::Zero()),
      task_coriolis_force_(Eigen::Vector3d::Zero()),
      task_gravity_force_(Eigen::Vector3d::Zero()), 
      task_friction_force_(Eigen::Vector3d::Zero()),
      F_c_(Eigen::Vector3d::Zero())
{}


/// @brief 
/// @param task_lambda_matrix 
/// @param task_coriolis_force 
/// @param task_gravity_force 
/// @param task_friction_force 
/// @param F_c 
/// @return 
bool State_in_Task_Space::update( Eigen::Matrix<double, 3, 3> &task_lambda_matrix, Eigen::Vector3d &task_coriolis_force, Eigen::Vector3d &task_gravity_force,
                                  Eigen::Vector3d &task_friction_force, Eigen::Vector3d &F_c )
{
    task_lambda_matrix_ = task_lambda_matrix;
    task_coriolis_force_ = task_coriolis_force;
    task_gravity_force_ = task_gravity_force;
    task_friction_force_ = task_friction_force;
    F_c_ = F_c;

    return true;
}

bool State_in_Task_Space::update_Fc( Eigen::Vector3d &F_c )
{
    F_c_ = F_c;
    return true;
}

}  // namespace force_control





#include <force_control/state_in_joint_space.hpp>

namespace force_control {

using Vector7d = Eigen::Matrix<double, 7, 1>;

State_in_Joint_Space::State_in_Joint_Space()
    : joint_mass_matrix_(Eigen::Matrix<double, 7, 7>::Zero()),
      joint_coriolis_force_(Vector7d::Zero()),
      joint_gravity_force_(Vector7d::Zero()),
    //   joint_friction_force_(Vector7d::Zero()),
      pose_(Eigen::Matrix<double, 4, 4>::Identity()),
      init_pose_(Eigen::Matrix<double, 4, 4>::Identity()),
      zero_jacobian_(Eigen::Matrix<double, 6, 7>::Zero()),
      jacobian_(Eigen::Matrix<double, 3, 7>::Zero()),
      jacobian_pseudo_inverse_(Eigen::Matrix<double, 7, 3>::Zero()),
      q_(Vector7d::Zero()),
      dq_(Vector7d::Zero()),
      initial_q_(Vector7d::Zero())
{}


/// @brief 
/// @param joint_mass_matrix 
/// @param joint_coriolis_force 
/// @param joint_gravity_force 
/// @param pose 
/// @param zero_jacobian 
/// @param jacobian 
/// @return 
bool State_in_Joint_Space::update( Eigen::Matrix<double, 7, 7> &joint_mass_matrix, Eigen::Matrix<double, 7, 1> &joint_coriolis_force, 
                                   Eigen::Matrix<double, 7, 1> &joint_gravity_force, Eigen::Matrix<double, 4, 4> &pose, 
                                   Eigen::Matrix<double, 6, 7> &zero_jacobian, Eigen::Matrix<double, 3, 7> &jacobian )
{
  joint_mass_matrix_ = joint_mass_matrix;
  joint_coriolis_force_ = joint_coriolis_force;
  joint_gravity_force_ = joint_gravity_force;
  // joint_friction_force_ = joint_friction_force; 
  pose_ = pose;
  zero_jacobian_ = zero_jacobian;
  jacobian_ = jacobian;

  return true;
}


/// @brief 
/// @param jacobian_pseudo_inverse 
/// @return 
bool State_in_Joint_Space::update_Pseudo_inverse( Eigen::Matrix<double, 7, 3> &jacobian_pseudo_inverse )
{
  jacobian_pseudo_inverse_ = jacobian_pseudo_inverse;
  return true;
}


bool State_in_Joint_Space::joint_state_init( Eigen::Matrix<double, 7, 1> &initial_q, Eigen::Matrix<double, 4, 4> &init_pose )
{
  initial_q_ = initial_q;
  init_pose_ = init_pose;
  return true;
}


bool State_in_Joint_Space::update_joint_state( Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq )
{
  q_ = q;
  dq_ = dq;
  return true;
}


bool State_in_Joint_Space::update_wrench( Eigen::Matrix<double, 6, 1> &wrench_ext )
{
  wrench_ext_ = wrench_ext;
  return true;
}


bool State_in_Joint_Space::update_base_T_EE( Eigen::Matrix<double, 4, 4> &O_T_EE )
{
  O_T_EE_ = O_T_EE;
  return true;
}


bool State_in_Joint_Space::init_base_T_EE( Eigen::Matrix<double, 4, 4> &init_O_T_EE )
{
  init_O_T_EE_ = init_O_T_EE;
  return true;
}

}  // namespace force_control





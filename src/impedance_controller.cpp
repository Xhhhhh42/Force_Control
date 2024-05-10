#include <force_control/impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace force_control {

/// @brief output: torque/effort
/// @return 
controller_interface::InterfaceConfiguration
Impedance_Controller::command_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


/// @brief input: position & velocity
/// @return 
controller_interface::InterfaceConfiguration
Impedance_Controller::state_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }

  return config;
}


/// @brief 
/// @param Time& time
/// @param period 
/// @return 
controller_interface::return_type Impedance_Controller::update( const rclcpp::Time& ,
                                                                const rclcpp::Duration& period ) 
{
  updateJointStates();
  Vector7d q_goal = compute_new_position( period );

  //test
  // Eigen::Vector3d new_position = compute_new_position();
  // Vector7d joint_positions_desired_eigen(joint_positions_desired_.data());
  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

  auto tau_d_calculated = compute_torque_command(
      q_goal, joint_positions_current_eigen, joint_velocities_current_eigen);
  
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}


Vector7d Impedance_Controller::compute_torque_command( const Vector7d& joint_positions_desired,
                                                       const Vector7d& joint_positions_current,
                                                       const Vector7d& joint_velocities_current ) 
{
  //test
  std::array<double, 49> mass_matrix_array = franka_robot_model_->getMassMatrix();
  // Convert std::array to Eigen::Matrix
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(mass_matrix_array.data());
  

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  Vector7d q_error = joint_positions_desired - joint_positions_current;

  Vector7d mass_compenent = mass_matrix * q_error;

  // Vector7d tau_d_calculated =
  //     k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + mass_compenent;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct( q_error ) + d_gains_.cwiseProduct(-dq_filtered_);

  return tau_d_calculated;
}


CallbackReturn Impedance_Controller::on_init() 
{
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}


/// @brief 
/// @param  State& previous_state
/// @return 
CallbackReturn Impedance_Controller::on_configure( const rclcpp_lifecycle::State& ) 
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  dq_filtered_.setZero();

  //test
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));

  return CallbackReturn::SUCCESS;
}


/// @brief 
/// @param State& previous_state
/// @return 
CallbackReturn Impedance_Controller::on_activate( const rclcpp_lifecycle::State& ) 
{
  updateJointStates();
  dq_filtered_.setZero();
  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  initial_q_ = joint_positions_current_eigen;
  elapsed_time_ = 0.0;

  //test
  joint_positions_desired_.reserve(num_joints_);
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}


void Impedance_Controller::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) 
  {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_current_[i] = position_interface.get_value();
    joint_velocities_current_[i] = velocity_interface.get_value();
  }
}


Eigen::Vector3d Impedance_Controller::compute_new_position() 
{
  elapsed_time_ = elapsed_time_ + trajectory_period_;
  double radius = 0.1;

  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_));

  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);

  Eigen::Vector3d new_position = position_;
  new_position.x() -= delta_x;
  new_position.z() -= delta_z;

  return new_position;
}


Vector7d Impedance_Controller::compute_new_position( const rclcpp::Duration& period )
{
  Vector7d q_goal = initial_q_;
  elapsed_time_ = elapsed_time_ + period.seconds();

  double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * elapsed_time_));
  q_goal(3) += delta_angle;
  q_goal(4) += delta_angle;

  return q_goal;
}


bool Impedance_Controller::assign_parameters() 
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }
  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints_, k_gains.size());
    return false;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }
  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints_, d_gains.size());
    return false;
  }
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  return true;
}

}  // namespace force_control

#include "pluginlib/class_list_macros.hpp"

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS( force_control::Impedance_Controller,
                        controller_interface::ControllerInterface )

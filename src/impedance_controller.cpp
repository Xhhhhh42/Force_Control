#include <force_control/impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace force_control {

using Vector7d = Eigen::Matrix<double, 7, 1>;

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
    // config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
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
  update_Joint_States();
  update_state_in_JS_();

  // std::ofstream out("/home/forcecontrol/Downloads/output.text", std::ios::app);
  // if (out.is_open()) {
  //   out<<std::fixed<<std::setprecision(4) << "q_goal:   " <<q_goal[0]<<","<<q_goal[1]<<","<<q_goal[2] 
  //                                                                   <<","<<q_goal[3]<<","<<q_goal[4]<<","<<q_goal[5]
  //                                                                   <<","<<q_goal[6]<< std::endl;
  //   out<<std::fixed<<std::setprecision(4) << "state_in_JS_->q_:    " <<state_in_JS_->q_[0]<<","<<state_in_JS_->q_[1]<<","<<state_in_JS_->q_[2] 
  //                                                                   <<","<<state_in_JS_->q_[3]<<","<<state_in_JS_->q_[4]<<","<<state_in_JS_->q_[5]
  //                                                                   <<","<<state_in_JS_->q_[6]<< std::endl;
  //   out<<std::fixed<<std::setprecision(4) << "joint_controller:   " <<tau_d_calculated[0]<<","<<tau_d_calculated[1]<<","<<tau_d_calculated[2] 
  //                                                                   <<","<<tau_d_calculated[3]<<","<<tau_d_calculated[4]<<","<<tau_d_calculated[5]
  //                                                                   <<","<<tau_d_calculated[6]<< std::endl;
  //   out << "-------------------------------------------------------------------------" << std::endl;
  //   out.close(); // Close the file after logging data
  // } else {
  //   std::cerr << "Unable to open log file\n";
  // }

  if( controller_type_ == "task_space_controller" ) {
    // Option 1: Use Task Sapce Controller
    auto tau_d_tsc = task_space_controller_->update();
    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(tau_d_tsc(i));
    }
  } else if( controller_type_ == "joint_space_controller" ) {
    // Option 2: Use Joint Sapce Controller
    auto tau_d_jsc = joint_space_controller_->update( period );
    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(tau_d_jsc(i));
    }  
  }  

  return controller_interface::return_type::OK;
}


CallbackReturn Impedance_Controller::on_init() 
{
  try {
    auto_declare<std::string>( "arm_id", "panda" );
    auto_declare<std::string>( "controller", "task_space_controller" );
    auto_declare<double>( "Lambda_d_ts", 1.0 );
    auto_declare<double>( "K_d_ts", 100.0 );
    auto_declare<double>( "D_d_ts", 20.0 );
    auto_declare<double>( "K_D_ts", 1.0 );
    auto_declare<std::vector<double>>("k_gains_js", {});
    auto_declare<std::vector<double>>("d_gains_js", {});
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

  franka_robot_model_ = std::make_unique<FrankaRobotModelExtended>(
                          FrankaRobotModelExtended(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));

  state_in_JS_ = std::make_shared<State_in_Joint_Space>();
  task_space_controller_ = std::make_shared<Task_Space_Controller>( Lambda_d_ts_, K_d_ts_, D_d_ts_, K_D_ts_ );
  joint_space_controller_ = std::make_shared<Joint_Space_Controller>( k_gains_js_, d_gains_js_ );

  return CallbackReturn::SUCCESS;
}


/// @brief 
/// @param State& previous_state
/// @return 
CallbackReturn Impedance_Controller::on_activate( const rclcpp_lifecycle::State& ) 
{
  update_Joint_States();
  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  Vector7d initial_q = joint_positions_current_eigen;

  //test
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  update_state_in_JS_();

  std::array<double, 16> init_pose_array = franka_robot_model_->getPoseMatrix( franka::Frame::kEndEffector );
  Eigen::Matrix<double, 4, 4> init_pose = Eigen::Map<Eigen::Matrix<double, 4, 4>> (init_pose_array.data());

  state_in_JS_->joint_state_init( initial_q, init_pose );
  const franka::RobotState* robot_state_ptr = franka_robot_model_->getRobotState();
  std::array< double, 16 > O_T_EE_array = robot_state_ptr->O_T_EE;
  Eigen::Matrix4d O_T_EE = Eigen::Map<Eigen::Matrix4d>(O_T_EE_array.data());
  state_in_JS_->init_base_T_EE( O_T_EE );

  task_space_controller_->init( state_in_JS_ );
  joint_space_controller_->init( state_in_JS_ );

  return CallbackReturn::SUCCESS;
}


void Impedance_Controller::update_Joint_States() {
  for (auto i = 0; i < num_joints; ++i) 
  {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_current_[i] = position_interface.get_value();
    joint_velocities_current_[i] = velocity_interface.get_value();
  }

  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());
  state_in_JS_->update_joint_state( joint_positions_current_eigen, joint_velocities_current_eigen );
}


bool Impedance_Controller::update_state_in_JS_() 
{
  std::array<double, 49> mass_matrix_array = franka_robot_model_->getMassMatrix();
  Eigen::Matrix<double, 7, 7> joint_mass_matrix = Eigen::Map<Eigen::Matrix<double, 7, 7>> (mass_matrix_array.data());

  std::array<double, 7> coriolis_force_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d joint_coriolis_force = Eigen::Map<Vector7d> (coriolis_force_array.data());

  std::array<double, 7> gravity_force_array = franka_robot_model_->getGravityForceVector();
  Vector7d joint_gravity_force = Eigen::Map<Vector7d> (gravity_force_array.data());

  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix( franka::Frame::kEndEffector );
  Eigen::Matrix<double, 4, 4> pose = Eigen::Map<Eigen::Matrix<double, 4, 4>> (pose_array.data());

  std::array<double, 42> zero_jacobian_array = franka_robot_model_->getZeroJacobian( franka::Frame::kEndEffector );
  Eigen::Matrix<double, 6, 7> zero_jacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>> (zero_jacobian_array.data());
  Eigen::Matrix<double, 3, 7> jacobian = zero_jacobian.topRows(3);

  state_in_JS_->update( joint_mass_matrix, joint_coriolis_force, joint_gravity_force, pose, zero_jacobian, jacobian );

  Eigen::Matrix3d Lambda = (state_in_JS_->jacobian_ * state_in_JS_->joint_mass_matrix_.inverse() * state_in_JS_->jacobian_.transpose()).inverse();
  update_jacobian_pseudo_inverse( Lambda );

  // 获取robot_state的指针
  const franka::RobotState* robot_state_ptr = franka_robot_model_->getRobotState();

  std::array< double, 6 > wrench_ext_array = robot_state_ptr->O_F_ext_hat_K;
  Eigen::Matrix<double, 6, 1> wrench_ext = Eigen::Map<Eigen::Matrix<double, 6, 1>>(wrench_ext_array.data());
  state_in_JS_->update_wrench( wrench_ext );

  std::array< double, 16 > O_T_EE_array = robot_state_ptr->O_T_EE;
  Eigen::Matrix4d O_T_EE = Eigen::Map<Eigen::Matrix4d>(O_T_EE_array.data());
  state_in_JS_->update_base_T_EE( O_T_EE );

  return true;  
}


/// @brief 
/// @param Lambda 
/// @return 
bool Impedance_Controller::update_jacobian_pseudo_inverse( Eigen::Matrix3d &Lambda )
{
    Eigen::Matrix<double, 7, 3> jacobian_pseudo_inverse = state_in_JS_->joint_mass_matrix_.inverse() * state_in_JS_->jacobian_.transpose() * Lambda;
            // joint_mass_matrix_.inverse() * jacobian_.transpose() * (jacobian_ * joint_mass_matrix_.inverse() * jacobian_.transpose()).inverse()
    state_in_JS_->update_Pseudo_inverse( jacobian_pseudo_inverse );

    return true;
}


bool Impedance_Controller::assign_parameters() 
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  controller_type_ = get_node()->get_parameter("controller").as_string();
  if( controller_type_ != "task_space_controller" && controller_type_ != "joint_space_controller" ) 
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid controller type: %s. Must be 'task_space_controller' or 'joint_space_controller'", controller_type_.c_str() );
    return false;
  }

  Lambda_d_ts_ = get_node()->get_parameter("Lambda_d_ts").as_double();
  K_d_ts_ = get_node()->get_parameter("K_d_ts").as_double();
  D_d_ts_ = get_node()->get_parameter("D_d_ts").as_double();
  K_D_ts_ = get_node()->get_parameter("K_D_ts").as_double();

  auto k_gains_js = get_node()->get_parameter("k_gains_js").as_double_array();
  auto d_gains_js = get_node()->get_parameter("d_gains_js").as_double_array();
  if (k_gains_js.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains_js parameter not set");
    return false;
  }
  if (k_gains_js.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains_js should be of size %d but is of size %ld",
                 num_joints_, k_gains_js.size());
    return false;
  }
  if (d_gains_js.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains_js parameter not set");
    return false;
  }
  if (d_gains_js.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains_js should be of size %d but is of size %ld",
                 num_joints_, d_gains_js.size());
    return false;
  }
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_js_(i) = d_gains_js.at(i);
    k_gains_js_(i) = k_gains_js.at(i);
  }

  return true;
}

}  // namespace force_control

#include "pluginlib/class_list_macros.hpp"

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS( force_control::Impedance_Controller,
                        controller_interface::ControllerInterface )

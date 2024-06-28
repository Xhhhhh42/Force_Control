// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <fstream>
#include <sstream>

#include <Eigen/Eigen>

#include <force_control/trajectory_datarecord_controller.hpp>

namespace force_control {

controller_interface::InterfaceConfiguration
Trajectory_Datarecord_Controller::command_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration
Trajectory_Datarecord_Controller::state_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }

  return config;
}


controller_interface::return_type Trajectory_Datarecord_Controller::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates();
  elapsed_time_ = elapsed_time_ + period.seconds();

  if( !returnOrigin() ) {
    return controller_interface::return_type::OK;
  }

  Vector7d q_goal;
  Vector7d q_dot_goal;
  Vector7d q_ddot_goal;
  static int num = q_ref_.size();
  static int cur = 0;
  if( cur++ < num ) {
    q_goal = q_ref_[cur];
    q_dot_goal = q_dot_ref_[cur];
    q_ddot_goal = q_ddot_ref_[cur];
    std::cerr << "Iteration num: " << cur << std::endl;
  } else {
    q_goal = q_;
    q_dot_goal.setZero();
    q_ddot_goal.setZero();
  }

  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis(coriolis_array.data());
  std::array<double, 49> mass_matrix_array = franka_robot_model_->getMassMatrix();
  Eigen::Matrix<double, 7, 7> joint_mass_matrix = Eigen::Map<Eigen::Matrix<double, 7, 7>> (mass_matrix_array.data());
  std::array<double, 7> gravity_force_array = franka_robot_model_->getGravityForceVector();
  Vector7d joint_gravity_force = Eigen::Map<Vector7d> (gravity_force_array.data());

  std::cerr << "q_error: " << q_goal - q_ << std::endl;
  std::cerr << "dq_error: " << q_dot_goal - dq_ << std::endl;

  Vector7d tau_d_calculated =
      k_gains_js_.cwiseProduct(q_goal - q_) + d_gains_js_.cwiseProduct(q_dot_goal - dq_) + coriolis + joint_mass_matrix * q_ddot_goal;
      // k_gains_js_.cwiseProduct(q_goal - q_) + d_gains_js_.cwiseProduct(q_dot_goal - dq_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  // std::cerr << "tau_d_calculated: " << tau_d_calculated << std::endl;
  // std::cerr << "dq_: " << dq_ << std::endl;
  // std::cerr << "q_: " << q_ << std::endl;

  // 保存data到文件
  if (!dataFile_.is_open()) {
    dataFile_.open( dataFile_filename_, std::ios::out | std::ios::app );
  }
  if (dataFile_.is_open()) {
    // timestamp(1) 
    dataFile_ << elapsed_time_ << " "; 

    // q_real(7)
    writeVector7d(q_,dataFile_);

    // q_ref(7) 
    writeVector7d(q_goal,dataFile_);

    // qDot_real(7) 
    writeVector7d(dq_,dataFile_);
    
    // qDot_ref(7) 
    writeVector7d(q_dot_goal,dataFile_);
    
    // ?u(7)? 
    writeVector7d(tau_d_calculated,dataFile_);
    
    // torques_real(7)
    writeVector7d(torques_real_,dataFile_);
    
    // G_org(7) || vector gravity
    writeVector7d(joint_gravity_force,dataFile_);

    // C_org(7) || vector coriolis
    writeVector7d(coriolis,dataFile_);

    // qDDot_ref(7)
    writeVector7d(q_ddot_goal,dataFile_);

    // M_org(7x7) || inertia matrix
    for (int i = 0; i < num_joints; ++i) {
      for (int j = 0; j < num_joints; ++j) {
      dataFile_ << joint_mass_matrix(i,j) << " ";
      }
    } 

    dataFile_ << "\n";
  }
  dataFile_.close();

  return controller_interface::return_type::OK;
}


void Trajectory_Datarecord_Controller::writeVector7d( Eigen::Matrix<double, 7, 1> &data, std::ofstream &file )
{
  if (!file.is_open()) {
    file.open( dataFile_filename_, std::ios::out | std::ios::app );
  }
  if (file.is_open()) {
    for (int i = 0; i < num_joints; ++i) 
      { file << data(i) << " "; }
  }
}


CallbackReturn Trajectory_Datarecord_Controller::on_init() 
{
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains_js", {});
    auto_declare<std::vector<double>>("d_gains_js", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}


CallbackReturn Trajectory_Datarecord_Controller::on_configure( const rclcpp_lifecycle::State& /*previous_state*/) 
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                   arm_id_ + "/" + k_robot_state_interface_name));

  return CallbackReturn::SUCCESS;
}

CallbackReturn Trajectory_Datarecord_Controller::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  initial_q_ = q_;
  curr_i_ = 0;
  elapsed_time_ = 0.0;

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  std::ifstream file(origin_path_);
  if (!file.is_open()) {
    std::cerr << "Unable to open file: " << origin_path_ << std::endl;
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < 7; ++i) {
    if (!(file >> q_origin_(i))) {
      std::cerr << "Error reading file: " << origin_path_ << std::endl;
      break;
    }
  }
  file.close();
  std::cout << q_origin_ << std::endl;

  return CallbackReturn::SUCCESS;
}

void Trajectory_Datarecord_Controller::updateJointStates() 
{
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(3 * i);
    const auto& velocity_interface = state_interfaces_.at(3 * i + 1);
    const auto& effort_interface = state_interfaces_.at(3 * i + 2);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    assert(effort_interface.get_interface_name() == "effort");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
    torques_real_(i) = effort_interface.get_value();
  }
}


bool Trajectory_Datarecord_Controller::assign_parameters() 
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains_js = get_node()->get_parameter("k_gains_js").as_double_array();
  auto d_gains_js = get_node()->get_parameter("d_gains_js").as_double_array();
  // auto qref_filename = get_node()->get_parameter("qref_filename").as_string();
  // auto qdotref_filename = get_node()->get_parameter("qdotref_filename").as_string();
  // auto qddotref_filename = get_node()->get_parameter("qddotref_filename").as_string();
  dataFile_filename_ = get_node()->get_parameter("dataFile_filename").as_string();
  input_filename_ = get_node()->get_parameter("input_filename").as_string();
  origin_path_ = get_node()->get_parameter("origin_path").as_string();
  traj_num_ = get_node()->get_parameter("traj_num").as_int();
  origin_path_ = origin_path_ + "Trajectory_" + std::to_string(traj_num_) + "/gen_q0.txt";

  if (k_gains_js.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains_js parameter not set");
    return false;
  }
  if (k_gains_js.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains_js should be of size %d but is of size %ld",
                 num_joints, k_gains_js.size());
    return false;
  }
  if (d_gains_js.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains_js parameter not set");
    return false;
  }
  if (d_gains_js.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains_js should be of size %d but is of size %ld",
                 num_joints, d_gains_js.size());
    return false;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_js_(i) = d_gains_js.at(i);
    k_gains_js_(i) = k_gains_js.at(i);
  }
  // if( qref_filename.empty() || qdotref_filename.empty() || qddotref_filename.empty() || dataFile_filename_.empty() ) {
  //   RCLCPP_FATAL(get_node()->get_logger(), "Unmatched CSV File name, please check");
  //   return false;
  // } 

  std::ifstream file(input_filename_);
  if (!file.is_open()) {
    std::cerr << "Unable to load input data file: " << input_filename_ << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double time;
    Vector7d q_com;
    Vector7d dq_slow_new;
    Vector7d ddq_slow_new;

    // 读取时间
    iss >> time;

    // 读取 q_com
    for (int i = 0; i < 7; ++i) {
        iss >> q_com[i];
    }

    // 读取 dq_slow_new
    for (int i = 0; i < 7; ++i) {
        iss >> dq_slow_new[i];
    }

    // 读取 ddq_slow_new
    for (int i = 0; i < 7; ++i) {
        iss >> ddq_slow_new[i];
    }

    q_ref_.push_back(q_com);
    q_dot_ref_.push_back(dq_slow_new);
    q_ddot_ref_.push_back(ddq_slow_new);
  }

  file.close();

  size_ref_ = q_ref_.size();
  if( size_ref_ != q_dot_ref_.size() || size_ref_ != q_ddot_ref_.size() ) {
    RCLCPP_FATAL(get_node()->get_logger(), "Input trajectory datas with different size.");
    return false;
  } 

  dataFile_.open(dataFile_filename_, std::ios::out | std::ios::trunc);
  if (!dataFile_.is_open()) {
    std::cerr << "Failed to open file: " << dataFile_filename_ << std::endl;
  }

  dq_filtered_.setZero();

  return true;
}


bool Trajectory_Datarecord_Controller::readCSV( const std::string& filename, std::vector<Eigen::Matrix<double, 7, 1>> &ref ) 
{
  std::ifstream file(filename);
  // std::vector<Eigen::Matrix<double, 7, 1>> data;
  std::string line;
  while (std::getline(file, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<double> values;
    while (std::getline(lineStream, cell, ',')) {
      values.push_back(std::stod(cell));
    }
    Eigen::VectorXd vec = Eigen::Map<Eigen::Matrix<double, 7, 1>>(values.data(), values.size());
    ref.push_back(vec);
  }
  return true;
}


bool Trajectory_Datarecord_Controller::returnOrigin()
{
  if( !init_ ) {
    Vector7d q_error = q_origin_ - q_;
    for (int i = 0; i < 4; i++)
      { q_error[i] = std::max(-0.04, std::min(0.04, q_error[i])); }
    
    q_error[4] = std::max(-0.1, std::min(0.1, q_error[4]));
    q_error[5] = std::max(-0.1, std::min(0.1, q_error[5]));
    q_error[6] = std::max(-0.3, std::min(0.3, q_error[6]));

    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_gains_js_.cwiseProduct(q_error) + d_gains_js_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(tau_d_calculated(i));
    }
    // std::cerr << "Init: " << std::endl;
    // std::cerr << "q_error: " << q_error << std::endl;
    // std::cerr << "q_: " << q_ << std::endl;
    // return true;
  }
  if( init_ ) return true;
  if( (q_ - q_origin_ ).norm() < 0.1 ) init_ = true;
  return false;
}

}  // namespace force_control

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE

PLUGINLIB_EXPORT_CLASS(force_control::Trajectory_Datarecord_Controller,
                       controller_interface::ControllerInterface)

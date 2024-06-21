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

#pragma once

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace force_control {

/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class Trajectory_Datarecord_Controller : public controller_interface::ControllerInterface 
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  bool readCSV( const std::string& filename, std::vector<Eigen::Matrix<double, 7, 1>> &ref );

private:
  bool assign_parameters();

  void updateJointStates();

  void writeVector7d( Eigen::Matrix<double, 7, 1> &data, std::ofstream &file );

  std::string arm_id_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d torques_real_;
  Vector7d k_gains_js_;
  Vector7d d_gains_js_;
  double elapsed_time_{0.0};

  std::vector<Eigen::Matrix<double, 7, 1>> q_ref_;
  std::vector<Eigen::Matrix<double, 7, 1>> q_dot_ref_;
  std::vector<Eigen::Matrix<double, 7, 1>> q_ddot_ref_;
  size_t size_ref_;
  size_t curr_i_;
  std::ofstream dataFile_;
  std::string dataFile_filename_;

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
};

}  // namespace force_control

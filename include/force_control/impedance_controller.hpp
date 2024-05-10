#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace force_control {

class Impedance_Controller : public controller_interface::ControllerInterface 
{
    
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
        const override;

    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
        const override;

    controller_interface::return_type update( const rclcpp::Time& time,
                                              const rclcpp::Duration& period ) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure( const rclcpp_lifecycle::State& previous_state ) override;

    CallbackReturn on_activate( const rclcpp_lifecycle::State& previous_state ) override;

private:
    void updateJointStates();

    /**
     * @brief Calculates the new pose based on the initial pose.
     *
     * @return  Eigen::Vector3d calculated sinosuidal period for the x,z position of the pose.
     */
    Eigen::Vector3d compute_new_position();

    /**
     * @brief Calculates the new pose based on the initial pose.
     *
     * @return  Eigen::Vector3d calculated sinosuidal period for the x,z position of the pose.
     */
    Vector7d compute_new_position( const rclcpp::Duration& period );

    /**
     * @brief computes the torque commands based on impedance control law with compensated coriolis
     * terms
     *
     * @return Eigen::Vector7d torque for each joint of the robot
     */
    Vector7d compute_torque_command(const Vector7d& joint_positions_desired,
                                    const Vector7d& joint_positions_current,
                                    const Vector7d& joint_velocities_current);

    /**
     * @brief assigns the Kp, Kd and arm_id parameters
     *
     * @return true when parameters are present, false when parameters are not available
     */
    bool assign_parameters();

    std::string arm_id_;
    const int num_joints = 7;
    Vector7d q_;
    Vector7d initial_q_;
    Vector7d dq_;
    Vector7d dq_filtered_;
    Vector7d k_gains_;
    Vector7d d_gains_;

    Eigen::Quaterniond orientation_;
    Eigen::Vector3d position_;

    double elapsed_time_{0.0};
    double trajectory_period_{0.001};

    //test
    int num_joints_{7};

    std::vector<double> joint_positions_desired_;
    std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
    std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};

    // std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};

};

}  // namespace force_control

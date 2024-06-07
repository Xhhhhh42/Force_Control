#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp/rclcpp.hpp>

#include <force_control/state_in_joint_space.hpp>
#include <force_control/task_space_controller.hpp>
#include <force_control/joint_space_controller.hpp>

#include <force_control/franka_robot_model_extended.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
    void update_Joint_States();

    bool update_jacobian_pseudo_inverse( Eigen::Matrix3d &Lambda );

    bool update_state_in_JS_();

    /**
     * @brief assigns the Kp, Kd and arm_id parameters
     *
     * @return true when parameters are present, false when parameters are not available
     */
    bool assign_parameters();

    std::string arm_id_;
    const int num_joints = 7;
    std::string controller_type_;
    double Lambda_d_ts_;
    double K_d_ts_;
    double D_d_ts_;
    double K_D_ts_;
    Eigen::Matrix<double, 7, 1> k_gains_js_;
    Eigen::Matrix<double, 7, 1> d_gains_js_;

    Eigen::Quaterniond orientation_;
    Eigen::Vector3d position_;

    std::shared_ptr<State_in_Joint_Space> state_in_JS_;
    std::shared_ptr<Task_Space_Controller> task_space_controller_;
    std::shared_ptr<Joint_Space_Controller> joint_space_controller_;

    //test
    int num_joints_{7};

    std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
    std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};

    std::unique_ptr<FrankaRobotModelExtended> franka_robot_model_;
    
    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};
};

}  // namespace force_control

#pragma once

#include <Eigen/Dense>
#include <memory>

#include <force_control/state_in_joint_space.hpp>
#include <rclcpp/rclcpp.hpp>


namespace force_control {

class Joint_Space_Controller
{
public:
    Joint_Space_Controller( Eigen::Matrix<double, 7, 1> &k_gains, Eigen::Matrix<double, 7, 1> &d_gains );

    ~Joint_Space_Controller() {};

    bool init( std::shared_ptr<State_in_Joint_Space> &state_in_JS );

    Eigen::Matrix<double, 7, 1> update( const rclcpp::Duration& period );

    // State
    std::shared_ptr<State_in_Joint_Space> state_in_JS_;
    
private:
    Eigen::Matrix<double, 7, 1> still_position();

    /**
     * @brief Calculates the new pose based on the initial pose.
     *
     * @return  Eigen::Vector3d calculated sinosuidal period for the x,z position of the pose.
     */
    Eigen::Matrix<double, 7, 1> compute_new_position( const rclcpp::Duration& period );

    /**
     * @brief computes the torque commands based on impedance control law with compensated coriolis
     * terms
     *
     * @return Eigen::Vector7d torque for each joint of the robot
     */
    Eigen::Matrix<double, 7, 1> compute_torque_command( const Eigen::Matrix<double, 7, 1>& joint_positions_desired,
                                                        const Eigen::Matrix<double, 7, 1>& joint_positions_current,
                                                        const Eigen::Matrix<double, 7, 1>& joint_velocities_current );

    bool inited_;
    Eigen::Matrix<double, 7, 1> dq_filtered_;
    Eigen::Matrix<double, 7, 1> k_gains_;
    Eigen::Matrix<double, 7, 1> d_gains_;
    double elapsed_time_{0.0};
};


}  // namespace force_control
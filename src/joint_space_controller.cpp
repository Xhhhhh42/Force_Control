#include <force_control/joint_space_controller.hpp>

namespace force_control {

using Vector7d = Eigen::Matrix<double, 7, 1>;

Joint_Space_Controller::Joint_Space_Controller( Eigen::Matrix<double, 7, 1> &k_gains, Eigen::Matrix<double, 7, 1> &d_gains )
    : k_gains_( k_gains ), d_gains_( d_gains )
{
    dq_filtered_ = Eigen::Matrix<double, 7, 1>::Zero();
    inited_ = false;
}


bool Joint_Space_Controller::init( std::shared_ptr<State_in_Joint_Space> &state_in_JS )
{
    state_in_JS_ = state_in_JS;
    inited_ = true;

    return true;
}


Eigen::Matrix<double, 7, 1> Joint_Space_Controller::still_position() 
{
    Vector7d q_goal = state_in_JS_->initial_q_;
  return q_goal;
}


Eigen::Matrix<double, 7, 1> Joint_Space_Controller::compute_new_position( const rclcpp::Duration& period )
{
  Vector7d q_goal = state_in_JS_->initial_q_;
  elapsed_time_ = elapsed_time_ + period.seconds();

  double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * elapsed_time_));

  q_goal(3) += delta_angle;
  q_goal(4) += delta_angle;

  return q_goal;
}


Eigen::Matrix<double, 7, 1> Joint_Space_Controller::compute_torque_command( const Eigen::Matrix<double, 7, 1>& joint_positions_desired,
                                                                            const Eigen::Matrix<double, 7, 1>& joint_positions_current,
                                                                            const Eigen::Matrix<double, 7, 1>& joint_velocities_current ) 
{
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
    Vector7d q_error = joint_positions_desired - joint_positions_current;

    Vector7d coriolis = state_in_JS_->joint_coriolis_force_;

    Vector7d tau_d_calculated =
        k_gains_.cwiseProduct( q_error ) + d_gains_.cwiseProduct(-dq_filtered_) + coriolis;
        // k_gains_.cwiseProduct( q_error ) + d_gains_.cwiseProduct(-dq_filtered_);

    return tau_d_calculated;
}


Eigen::Matrix<double, 7, 1> Joint_Space_Controller::update( const rclcpp::Duration& period )
{
    // // Option 1: joints 4 and 5 move in a periodic movement
    // Vector7d q_goal = compute_new_position( period );

    // Option 2: keep still
    Vector7d q_goal = still_position();

    auto tau_d_calculated = compute_torque_command( q_goal, state_in_JS_->q_, state_in_JS_->dq_ );

    return tau_d_calculated;
}

}  // namespace force_control





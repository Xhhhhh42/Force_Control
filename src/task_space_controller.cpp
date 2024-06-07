#include <force_control/task_space_controller.hpp>

// log
#include <rclcpp/rclcpp.hpp>

namespace force_control {

Task_Space_Controller::Task_Space_Controller( double &Lambda_d_ts, double &K_d_ts, double &D_d_ts, double &K_D_ts )
{
    x_dot_d_ = Eigen::Vector3d::Zero();
    x_ddot_d_ = Eigen::Vector3d::Zero();
    last_Fex_ = Eigen::Vector3d::Zero();
    Lambda_d_ = Eigen::Matrix3d::Identity() * Lambda_d_ts;  // 刚度矩阵
    Kd_ = Eigen::Matrix3d::Identity() * K_d_ts;   // 阻尼矩阵
    Dd_ = Eigen::Matrix3d::Identity() * D_d_ts;    // 惯性矩阵
    K_D_ = Eigen::Matrix<double, 7, 7>::Identity() * K_D_ts;
    inited_ = false;

    out.open("/home/forcecontrol/Downloads/output.txt", std::ios::trunc);
    if (!out.is_open()) {
        RCLCPP_FATAL(rclcpp::get_logger("Unable to open log file"), "noopen");
    }
    out.close();

    out.open("/home/forcecontrol/Downloads/output.txt", std::ios::app);
    if (out.is_open()) {
        out << "--------------------------Parameters-------------------------------------" << std::endl;
        out<<std::fixed<<std::setprecision(4) << "Lambda_d_ts:   " <<Lambda_d_ts<< std::endl;
        out<<std::fixed<<std::setprecision(4) << "K_d_ts:   " <<K_d_ts<< std::endl;
        out<<std::fixed<<std::setprecision(4) << "D_d_ts:   " <<D_d_ts<< std::endl;
        out<<std::fixed<<std::setprecision(4) << "K_D_ts:   " <<K_D_ts<< std::endl;
        out << "-------------------------------------------------------------------------" << std::endl;
        out.close(); // Close the file after logging data
    } else {
        std::cerr << "Unable to open log file\n";
    }
}


bool Task_Space_Controller::init( std::shared_ptr<State_in_Joint_Space> &state_in_JS )
{
    state_in_JS_ = state_in_JS;
    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d(state_in_JS_->init_O_T_EE_));
    xd_ = initial_transform.translation();
    orientation_d_ = initial_transform.linear();

    state_in_TS_ = std::make_shared<State_in_Task_Space>();

    inited_ = true;

    return true;
}

void Task_Space_Controller::update_Task_State()
{
    // 计算任务空间位置和速度
    Eigen::Affine3d transform(Eigen::Matrix4d(state_in_JS_->O_T_EE_));
    x_ = transform.translation();
    orientation_ = transform.linear();

    // compute error to desired equilibrium pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << x_ - xd_;

    
    out.open("/home/forcecontrol/Downloads/output.txt", std::ios::app);
    if (out.is_open()) {
        out<<std::fixed<<std::setprecision(4) << "xd_:   " <<xd_[0]<<","<<xd_[1]<<","<<xd_[2] << std::endl;
        out<<std::fixed<<std::setprecision(4) << "x_:    " <<x_[0]<<","<<x_[1]<<","<<x_[2] << std::endl;
        out<<std::fixed<<std::setprecision(4) << "Error: " <<error[0]<<","<<error[1]<<","<<error[2] << std::endl;
        out << "-------------------------------------------------------------------------" << std::endl;
        out.close(); // Close the file after logging data
    } else {
        std::cerr << "Unable to open log file\n";
    }

    // orientation error
    // "difference" quaternion
    if (orientation_d_.coeffs().dot(orientation_.coeffs()) < 0.0) {
        orientation_.coeffs() << -orientation_.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation_.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    x_dot_ = state_in_JS_->jacobian_ * state_in_JS_->dq_;

    Eigen::Matrix3d Lambda = (state_in_JS_->jacobian_ * state_in_JS_->joint_mass_matrix_.inverse() * state_in_JS_->jacobian_.transpose()).inverse();
    // Eigen::Vector3d Fg = state_in_JS_->jacobian_pseudo_inverse_.transpose() * state_in_JS_->joint_gravity_force_;
    // Eigen::Vector3d mu = Lambda * ();

    print_lambda = Lambda;

    // Eigen::Vector3d F_ext = state_in_JS_->wrench_ext_.head<3>();
    // const double kAlpha = 0.5;
    // F_ext = (1 - kAlpha) * F_ext + kAlpha * last_Fex_;
    // last_Fex_ = F_ext;

    Eigen::Vector3d F_ext;

    updateTaskSpaceControl( Lambda, x_, xd_, x_dot_, x_dot_d_, x_ddot_d_, F_ext );
}


/// @brief compute control torque in Task Space
/// @param Lambda 
/// @param x 
/// @param xd 
/// @param x_dot 
/// @param x_dot_d 
/// @param x_ddot_d 
/// @param F_ext 
/// @return 
bool Task_Space_Controller::updateTaskSpaceControl(
        const Eigen::Matrix3d& Lambda, const Eigen::Vector3d& x, const Eigen::Vector3d& xd, 
        const Eigen::Vector3d& x_dot, const Eigen::Vector3d& x_dot_d, 
        const Eigen::Vector3d& x_ddot_d, const Eigen::Vector3d& F_ext)
{
    Eigen::Vector3d x_error = calculateError(x, xd);
    Eigen::Vector3d x_dot_error = x_dot - x_dot_d;

    out.open("/home/forcecontrol/Downloads/output.txt", std::ios::app);
    if (out.is_open()) {
        out<<std::fixed<<std::setprecision(4) << "x_error:   " <<x_error[0]<<","<<x_error[1]<<","<<x_error[2] << std::endl;
        out<<std::fixed<<std::setprecision(4) << "x_dot_error:    " <<x_dot_error[0]<<","<<x_dot_error[1]<<","<<x_dot_error[2] << std::endl;
        out << "-------------------------------------------------------------------------" << std::endl;
        out.close(); // Close the file after logging data
    } else {
        std::cerr << "Unable to open log file\n";
    }
    
    Eigen::Vector3d F_c = Lambda * (Lambda_d_.inverse() * (-Kd_ * x_error - Dd_ * x_dot_error) + x_ddot_d) 
                           + (Lambda * Lambda_d_.inverse() - Eigen::Matrix3d::Identity()) * F_ext;
    print1 = Lambda * (Lambda_d_.inverse() * (-Kd_ * x_error - Dd_ * x_dot_error) + x_ddot_d);
    print1_1 = Lambda_d_.inverse() * (-Kd_ * x_error - Dd_ * x_dot_error) + x_ddot_d;
    print1_2 = x_ddot_d;
    print1_3 = -Kd_ * x_error - Dd_ * x_dot_error;
    print2 = (Lambda * Lambda_d_.inverse() - Eigen::Matrix3d::Identity()) * F_ext;

    out.open("/home/forcecontrol/Downloads/output.txt", std::ios::app);
    if (out.is_open()) {
        out<<std::fixed<<std::setprecision(4) << "F_ext:   " <<F_ext[0]<<","<<F_ext[1]<<","<<F_ext[2] << std::endl;
        out<<std::fixed<<std::setprecision(4) << "(Lambda * Lambda_d_.inverse() - Eigen::Matrix3d::Identity()) * F_ext:    " <<print2[0]<<","<<print2[1]<<","<<print2[2] << std::endl;
        out<<std::fixed<<std::setprecision(4) << "-Kd * x_error - Dd * x_dot_error:    " <<print1_3[0]<<","<<print1_3[1]<<","<<print1_3[2] << std::endl;        
        out<<std::fixed<<std::setprecision(4) << "Lambda * (Lambda_d.inverse() * (-Kd * x_error - Dd * x_dot_error) + x_ddot_d):    " <<print1[0]<<","<<print1[1]<<","<<print1[2] << std::endl;        
        out << "-------------------------------------------------------------------------" << std::endl;
        out.close(); // Close the file after logging data
    } else {
        std::cerr << "Unable to open log file\n";
    }

    state_in_TS_->update_Fc( F_c );
    return true;
}  


/// @brief 转换为关节空间力矩
/// @param jacobian 
/// @param F_c 
/// @return 
Eigen::Matrix<double, 7, 1> Task_Space_Controller::control_torque_in_JointSpace( Eigen::Matrix<double, 3, 7> &jacobian, Eigen::Vector3d &F_c )
{
    return jacobian.transpose() * F_c;
}


/// @brief add a null-space damping term to make the system stable
/// @param tau_c 
/// @param K_D 
/// @return 
Eigen::Matrix<double, 7, 1> Task_Space_Controller::add_null_space_damping( Eigen::Matrix<double, 7, 1> &tau_c, const Eigen::Matrix<double, 7, 7> &K_D, Eigen::Matrix<double, 7, 3> &jacobian_pseudo_inverse )
{
    return tau_c + ( Eigen::Matrix<double, 7, 7>::Identity() - state_in_JS_->jacobian_.transpose() * jacobian_pseudo_inverse.transpose()) * (-K_D * state_in_JS_->dq_);
}


Eigen::Matrix<double, 7, 1> Task_Space_Controller::update()
{
    Vector7d tau_d_calculated;
    if( !inited_ ) return tau_d_calculated;

    update_Task_State();
    
    Vector7d tau_c = control_torque_in_JointSpace( state_in_JS_->jacobian_, state_in_TS_->F_c_ );
    tau_d_calculated = add_null_space_damping( tau_c, K_D_, state_in_JS_->jacobian_pseudo_inverse_ );

    out.open("/home/forcecontrol/Downloads/output.txt", std::ios::app);
    if (out.is_open()) {
        out<<std::fixed<<std::setprecision(4) << "F_c_:   " <<state_in_TS_->F_c_[0]<<","<<state_in_TS_->F_c_[1]<<","<<state_in_TS_->F_c_[2] << std::endl;
        out<<std::fixed<<std::setprecision(4) << "control_torque_in_JointSpace:   " <<tau_c[0]<<","<<tau_c[1]<<","<<tau_c[2] 
                                                                        <<","<<tau_c[3]<<","<<tau_c[4]<<","<<tau_c[5]
                                                                        <<","<<tau_c[6]<< std::endl;
        out<<std::fixed<<std::setprecision(4) << "task_controller:   " <<tau_d_calculated[0]<<","<<tau_d_calculated[1]<<","<<tau_d_calculated[2] 
                                                                        <<","<<tau_d_calculated[3]<<","<<tau_d_calculated[4]<<","<<tau_d_calculated[5]
                                                                        <<","<<tau_d_calculated[6]<< std::endl;
        out << "-------------------------------------------------------------------------" << std::endl;
        out.close(); // Close the file after logging data
    } else {
        std::cerr << "Unable to open log file\n";
    }

    return tau_d_calculated;
}

}  // namespace force_control





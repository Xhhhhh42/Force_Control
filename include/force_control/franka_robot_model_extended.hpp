#pragma once

#include <franka_semantic_components/franka_robot_model.hpp>
#include <franka/robot_state.h>

class FrankaRobotModelExtended : public franka_semantic_components::FrankaRobotModel 
{
public:
    FrankaRobotModelExtended(
        const std::string& robot_model_interface_name, 
        const std::string& robot_state_interface_name)
        : franka_semantic_components::FrankaRobotModel(robot_model_interface_name, robot_state_interface_name)
    {
    }

    const franka::RobotState* getRobotState() const {
        return this->robot_state;
    }
};
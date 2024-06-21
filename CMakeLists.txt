cmake_minimum_required(VERSION 3.8)
project(force_control)       


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library)
find_package(franka_semantic_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/impedance_controller.cpp
        src/state_in_joint_space.cpp
        src/state_in_task_space.cpp
        src/task_space_controller.cpp
        src/joint_space_controller.cpp
        src/trajectory_datarecord_controller.cpp
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        franka_semantic_components
)

generate_parameter_library(force_control_parameters src/model_impedance_controller_parameters.yaml)

target_link_libraries(${PROJECT_NAME} force_control_parameters)

pluginlib_export_plugin_description_file(
        controller_interface force_control.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

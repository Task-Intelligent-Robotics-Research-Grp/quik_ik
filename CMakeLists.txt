cmake_minimum_required(VERSION 3.16)
project(quik_ik)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(fmt REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(range-v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rsl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tl-expected REQUIRED)

generate_parameter_library(
  quik_ik_parameters
  src/quik_ik_parameters.yaml
)
add_library(quik_ik_plugin SHARED
  src/fk_moveit.cpp
  src/forward_kinematics.cpp
  src/quik_ik_plugin.cpp
  src/goal.cpp
  src/ik_memetic.cpp
  src/ik_gradient.cpp
  src/robot.cpp
)
target_compile_features(quik_ik_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(quik_ik_plugin PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/quik_ik>)
target_link_libraries(quik_ik_plugin
  PUBLIC
    moveit_core::moveit_robot_model
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_kdl::tf2_kdl
    tl::expected
    rsl::rsl
  PRIVATE
    fmt::fmt
    quik_ik_parameters
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_state
    pluginlib::pluginlib
    range-v3::range-v3
    rclcpp::rclcpp
)

pluginlib_export_plugin_description_file(
  moveit_core
  quik_ik_kinematics_description.xml
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

install(
  DIRECTORY include/
  DESTINATION include/quik_ik
)
install(
  TARGETS quik_ik_plugin quik_ik_parameters
  EXPORT export_quik_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_quik_ik HAS_LIBRARY_TARGET)
ament_export_dependencies(
  fmt
  moveit_core
  pluginlib
  range-v3
  rclcpp
  rsl
  tf2_geometry_msgs
  tf2_kdl
  tl-expected
)
ament_package()

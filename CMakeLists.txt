cmake_minimum_required(VERSION 3.16)
project(quik_ik)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(quik REQUIRED)

generate_parameter_library(
  ${PROJECT_NAME}_parameters
    src/quik_ik_parameters.yaml
)

add_library(
  ${PROJECT_NAME}_plugin SHARED
    src/quik_ik_plugin.cpp
)

target_compile_features(
  ${PROJECT_NAME}_plugin PUBLIC
    c_std_99 cxx_std_17)

target_include_directories(
  ${PROJECT_NAME}_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(
  ${PROJECT_NAME}_plugin
  PUBLIC
    moveit_core::moveit_robot_model
  PRIVATE
    ${PROJECT_NAME}_parameters
    class_loader::class_loader
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_state
    pluginlib::pluginlib
    rclcpp::rclcpp
    quik::quik
)

install(
  DIRECTORY   include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_plugin ${PROJECT_NAME}_parameters
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(
  moveit_core
    ${PROJECT_NAME}_kinematics_description.xml
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  class_loader
  moveit_core
  pluginlib
  rclcpp
  quik
)

ament_package()

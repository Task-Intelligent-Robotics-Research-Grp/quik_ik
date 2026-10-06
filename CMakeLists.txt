cmake_minimum_required(VERSION 3.20)
project(quik_ik)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
#  dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

##########################
#  libquik_ik_plugin.so  #
##########################
generate_parameter_library(
  quik_ik_parameters
    src/quik_ik_parameters.yaml
)

ament_auto_add_library(
  ${PROJECT_NAME}_plugin SHARED
    src/quik_ik_plugin.cpp
)

target_include_directories(
  ${PROJECT_NAME}_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${quik_INCLUDE_DIRS}
)

# target_link_libraries(
#   ${PROJECT_NAME}_plugin PUBLIC
#     quik_ik_parameters
# )

pluginlib_export_plugin_description_file(
  moveit_core
    quik_ik_kinematics_description.xml
)

##########
#  test  #
##########
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

ament_auto_package()

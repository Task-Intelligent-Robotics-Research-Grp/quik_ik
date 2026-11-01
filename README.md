![GitHub Release](https://img.shields.io/github/v/release/Task-Intelligent-Robotics-Research-Grp/quik_ik)
![GitHub License](https://img.shields.io/github/license/Task-Intelligent-Robotics-Research-Grp/quik_ik)

| ROS 2 Distribution | Jazzy                                                                                                                                                                    | Lyrical                                                                                                                                                                  |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Build Status       | [![jazzy-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/quik_ik/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/quik_ik/actions/workflows/jazzy-build.yaml) | [![lyrical-build](https://github.com/Task-Intelligent-Robotics-Research-Grp/quik_ik/actions/workflows/lyrical-build.yaml/badge.svg)](https://github.com/Task-Intelligent-Robotics-Research-Grp/quik_ik/actions/workflows/lyrical-build.yaml) |

quik_ik
==================================================
## Overview
This package provides a MoveIt2 plugin for computing inverse kinematics(IK) using [QuIK](https://github.com/steffanlloyd/quik/).

## Install
First, download this software by
  ```bash
 cd ~/ros2_ws/src
 git clone https://github.com/Task-Intelligent-Robotics-Research-Grp/quik_ik
 ```

 Then, download `QuIK` and apply it the following [patch](./quik.patch). This patch modifies only `package.xml` and `CMakeLists.txt`.
 No modification on C++ code will be made.
 ```bash
 vcs import . --input=quik_ik/dependencies.repos
 cd quik
 patch < ../quik_ik/quik.patch
 ```
Then, install other dependencies.
 ```bash
 cd ..
 rosdep update
 rosdep install -iy --from-paths ./
 ```
 Finally, build your workspace.
 ```bash
 cd ~/ros2_ws
 colcon build
 ```
 
// BSD 3-Clause License
//
// Copyright (c) 2023, National Institute of Industrial Science
// and Technology(AIST)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
// OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
//
#pragma once

#include <iostream>
#include <quik/Robot.hpp>
#include <quik/IKSolver.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>

namespace quik_ik
{
class ParamListener;

/************************************************************************
*  class QuikIKPlugin                                                   *
************************************************************************/
class QuikIKPlugin : public kinematics::KinematicsBase
{
  private:
    using pose_t        = geometry_msgs::msg::Pose;
    using robot_model_t = moveit::core::RobotModel;
    using robot_state_t = moveit::core::RobotState;
    using link_model_t  = moveit::core::LinkModel;
    using error_code_t  = moveit_msgs::msg::MoveItErrorCodes;
    using options_t     = kinematics::KinematicsQueryOptions;

    using robot_t       = quik::Robot<>;
    using solver_t      = quik::IKSolver<>;
    using link_type_t   = quik::JOINTTYPE_t;
    using link_types_t  = Eigen::VectorX<link_type_t>;
    using dh_params_t   = Eigen::Array<double, Eigen::Dynamic, 4>;
    using transform_t   = Eigen::Isometry3d;
    using vector3_t     = Eigen::Vector3d;
    using vector4_t     = Eigen::Vector4d;
    using vector6_t     = Eigen::Vector<double, 6>;
    using vector_t      = Eigen::VectorXd;
    using map_t         = Eigen::Map<const vector_t>;

  public:
    struct dh_param_t
    {
        double      a      = 0.0;  // distance between adjacent joint axes
        double      alpha  = 0.0;  // angle between adjacent joint axes
        double      lambda = 0.0;
        double      mu     = 0.0;
        transform_t S      = transform_t::Identity();
    };

  public:
    const std::vector<std::string>& getJointNames()             const   ;
    const std::vector<std::string>& getLinkNames()              const   ;

    bool initialize(const rclcpp::Node::SharedPtr& node,
                    const robot_model_t& robot_model,
                    const std::string& group_name,
                    const std::string& base_frame,
                    const std::vector<std::string>& tip_frames,
                    double search_discretization)                       ;

    bool getPositionFK(const std::vector<std::string>& link_names,
                       const std::vector<double>& joint_angles,
                       std::vector<pose_t>& poses)              const   ;

    bool getPositionIK(const pose_t& ik_pose,
                       const std::vector<double>& ik_seed_state,
                       std::vector<double>& solution,
                       error_code_t& error_code,
                       const options_t& options)                const   ;

    bool searchPositionIK(const std::vector<pose_t>& ik_poses,
                          const std::vector<double>& ik_seed_state,
                          double timeout,
                          const std::vector<double>& consistency_limits,
                          std::vector<double>& solution,
                          const IKCallbackFn& solution_callback,
                          const IKCostFn& cost_function,
                          error_code_t& error_code,
                          const options_t& options = options_t(),
                          const robot_state_t* context_state=nullptr) const;

    bool searchPositionIK(const pose_t& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double timeout,
                          std::vector<double>& solution,
                          error_code_t& error_code,
                          const options_t& options=options_t()) const   ;

    bool searchPositionIK(const pose_t& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double timeout,
                          const std::vector<double>& consistency_limits,
                          std::vector<double>& solution,
                          error_code_t& error_code,
                          const options_t& options=options_t()) const   ;

    bool searchPositionIK(const pose_t& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double timeout,
                          std::vector<double>& solution,
                          const IKCallbackFn& solution_callback,
                          error_code_t& error_code,
                          const options_t& options=options_t()) const   ;

    bool searchPositionIK(const pose_t& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double timeout,
                          const std::vector<double>& consistency_limits,
                          std::vector<double>& solution,
                          const IKCallbackFn& solution_callback,
                          error_code_t& error_code,
                          const options_t& options=options_t()) const   ;

    bool searchPositionIK(const std::vector<pose_t>& ik_poses,
                          const std::vector<double>& ik_seed_state,
                          double timeout,
                          const std::vector<double>& consistency_limits,
                          std::vector<double>& solution,
                          const IKCallbackFn& solution_callback,
                          error_code_t& error_code,
                          const options_t& options=options_t(),
                          const robot_state_t* context_state=nullptr) const;

  private:
    dh_param_t  computeDHParam(const vector3_t& np, const vector3_t& nc,
                               const transform_t& T)            const   ;
    bool        withinLimits(const vector_t& q)                 const   ;
    bool        getPositionFKQuik(const std::vector<std::string>& link_names,
                                  const std::vector<double>& joint_angles,
                                  std::vector<pose_t>& poses)   const   ;
    bool        getPositionFKMoveIt(const std::vector<std::string>& link_names,
                                    const std::vector<double>& joint_angles,
                                    std::vector<pose_t>& poses) const   ;

  private:
    rclcpp::Node::SharedPtr             node_;
    std::shared_ptr<ParamListener>      param_listener_;

    std::vector<std::string>            joint_names_;
    std::vector<std::string>            link_names_;
    std::vector<transform_t>            S_;
    std::shared_ptr<robot_t>            robot_;
};
}  // namespace quik_ik

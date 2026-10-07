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
#include <quik_ik/quik_ik_parameters.hpp>
#include <quik_ik/quik_ik_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <string>
#include <vector>

namespace quik_ik
{
namespace
{
auto const LOGGER = rclcpp::get_logger("quik_ik");

template <class T> std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    for (const auto& elm : v)
        out << ' ' << elm;
    return out;
}
}

/************************************************************************
*  class QuikIKPlugin                                                   *
************************************************************************/
std::vector<std::string> const&
QuikIKPlugin::getJointNames() const
{
    return joint_names_;
}

std::vector<std::string> const&
QuikIKPlugin::getLinkNames() const
{
    return link_names_;
}

QuikIKPlugin::QuikIKPlugin()
{
}
    
QuikIKPlugin::~QuikIKPlugin()
{
}
    
bool
QuikIKPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                         const robot_model_t& robot_model,
                         const std::string& group_name,
                         const std::string& base_frame,
                         const std::vector<std::string>& tip_frames,
                         double search_discretization)
{
    using namespace moveit::core;

    RCLCPP_INFO_STREAM(LOGGER,
                       "quik_ik initialized: group_name=" << group_name
                       << ", base_frame=" << base_frame
                       << ", tip_frames=[" << tip_frames << ']');

    node_           = node;
    param_listener_ = std::make_shared<ParamListener>(
                          node, std::string("robot_description_kinematics.")
                          .append(group_name));

    // Initialize internal state of base class KinematicsBase
    // Creates these internal state variables:
    //   robot_model_                    <- shared_ptr to RobotModel
    //   robot_description_              <- empty string
    //   group_name_                     <- group_name string
    //   base_frame_                     <- base_frame without leading /
    //   tip_frames_                     <- tip_frames without leading /
    //   redundant_joint_discretization_ <- vector initialized with
    //                                      search_discretization
    storeValues(robot_model, group_name, base_frame, tip_frames,
                search_discretization);

    // Initialize internal state
    jmg_ = robot_model_->getJointModelGroup(group_name);
    if (!jmg_)
    {
        RCLCPP_ERROR_STREAM(LOGGER,
                            "failed to get joint model group " << group_name);
        return false;
    }

    // Joint names come from jmg
    for (const auto* joint : jmg_->getJointModels())
        if (joint->getType() == JointModel::REVOLUTE ||
            joint->getType() == JointModel::PRISMATIC)
            joint_names_.push_back(joint->getName());

    dh_params_t  DH(joint_names_.size(), 4);
    link_types_t link_types(joint_names_.size());
    qsign_t      qsigns(joint_names_.size());
    auto         T = transform_t::Identity();
    transform_t  Tbase;
    vector3_t    np;
    double       lambda = 0.0;
    ssize_t      joint_num = 0;
    for (const auto joint : jmg_->getJointModels())
    {
        RCLCPP_WARN_STREAM(LOGGER, "### joint_name=" << joint->getName());
                           
        const auto child_link = joint->getChildLinkModel();

        T = T * child_link->getJointOriginTransform();

        if (joint_num == 0)
            Tbase = T;
        
        switch (joint->getType())
        {
          case JointModel::REVOLUTE:
          {
            const auto nc = static_cast<const RevoluteJointModel*>(joint)
                            ->getAxis().normalized();
            const auto dh = computeDHParam(np, nc, T);
            DH(joint_num, 0) = dh.a;
            DH(joint_num, 1) = dh.alpha;
            DH(joint_num, 3) = 0.0;
            if (joint_num > 0)
                DH(joint_num-1, 2) = lambda - dh.mu;
            lambda            = dh.lambda;
            qsigns[joint_num] = 1;
            link_types[joint_num++] = quik::JOINT_REVOLUTE;
            T.setIdentity();
            np = nc;
            break;
          }
          case JointModel::PRISMATIC:
          {
            const auto nc = static_cast<const PrismaticJointModel*>(joint)
                          ->getAxis().normalized();
            const auto dh = computeDHParam(np, nc, T);
            DH(joint_num, 0) = dh.a;
            DH(joint_num, 1) = dh.alpha;
            DH(joint_num, 3) = 0.0;
            if (joint_num > 0)
                DH(joint_num-1, 2) = lambda - dh.mu;
            lambda            = dh.lambda;
            qsigns[joint_num] = 1;
            link_types[joint_num++] = quik::JOINT_PRISMATIC;
            T.setIdentity();
            np = nc;
            break;
          }
          default:
            break;
        }
    }

    DH(joint_num-1, 2) = lambda;

    RCLCPP_WARN_STREAM(LOGGER, "### DH ###\n" << DH);
    // RCLCPP_WARN_STREAM(LOGGER, "### link_types=" << link_types);
    RCLCPP_WARN_STREAM(LOGGER, "### qsigns=" << qsigns);

    robot_ = make_shared<robot_t>(DH, link_types, qsigns,
                                  Tbase.matrix(), T.matrix());

  // link_names are the same as tip frames
  // TODO: why do we need to set this
    link_names_ = tip_frames_;

    RCLCPP_WARN_STREAM(LOGGER, "### joint_names=" << joint_names_);
    RCLCPP_WARN_STREAM(LOGGER, "### link_names=" << link_names_);

  // tip_link_indices_ = get_link_indices(robot_model_, tip_frames_)
  //                    .or_else([](auto const& error)
  //                             { throw std::invalid_argument(error); })
  //                    .value();
    
    return true;
}

bool
QuikIKPlugin::getPositionFK(const std::vector<std::string>&,
                            const std::vector<double>&,
                            std::vector<pose_t>&) const
{
    return false;
}

bool
QuikIKPlugin::getPositionIK(const pose_t&,
                            const std::vector<double>&,
                            std::vector<double>&,
                            error_code_t&,
                            const options_t&) const
{
    return false;
}

bool
QuikIKPlugin::searchPositionIK(const std::vector<pose_t>& ik_poses,
                               const std::vector<double>& ik_seed_state,
                               double timeout,
                               const std::vector<double>&,
                               std::vector<double>& solution,
                               const IKCallbackFn& solution_callback,
                               const IKCostFn& cost_function,
                               error_code_t& error_code,
                               const options_t& options,
                               const robot_state_t* context_state) const
{
    (void)context_state;  // not used

    const auto  params = param_listener_->get_params();
    const auto  algo   = (params.algorithm == "QUIK" ? quik::ALGORITHM_QUIK :
                          params.algorithm == "NR"   ? quik::ALGORITHM_NR   :
                                                       quik::ALGORITHM_BFGS);
    const solver_t solver(robot_,
                          params.max_iterations,
                          algo,
                          params.exit_tolerance,
                          params.minimum_step_size,
                          params.relative_improvement_tolerance,
                          params.max_consecutive_grad_fails,
                          params.max_gradient_fails,
                          params.lambda_squared,
                          params.max_linear_step_size,
                          params.max_angular_step_size,
                          params.armijo_sigma,
                          params.armijo_beta);
}

bool
QuikIKPlugin::searchPositionIK(const pose_t& ik_pose,
                               const std::vector<double>& ik_seed_state,
                               double timeout,
                               std::vector<double>& solution,
                               error_code_t& error_code,
                               const options_t& options) const
{
    return searchPositionIK(std::vector<pose_t>{ik_pose},
                            ik_seed_state,
                            timeout,
                            std::vector<double>(),      // consistency_limits
                            solution,
                            IKCallbackFn(),             // solition_callback
                            error_code,
                            options);
}

bool
QuikIKPlugin::searchPositionIK(const pose_t& ik_pose,
                               const std::vector<double>& ik_seed_state,
                               double timeout,
                               const std::vector<double>& consistency_limits,
                               std::vector<double>& solution,
                               error_code_t& error_code,
                               const options_t& options) const
{
    return searchPositionIK(std::vector<pose_t>{ik_pose},
                            ik_seed_state,
                            timeout,
                            consistency_limits,
                            solution,
                            IKCallbackFn(),             // solition_callback
                            error_code,
                            options);
}

bool
QuikIKPlugin::searchPositionIK(const pose_t& ik_pose,
                               const std::vector<double>& ik_seed_state,
                               double timeout,
                               std::vector<double>& solution,
                               const IKCallbackFn& solution_callback,
                               error_code_t& error_code,
                               const options_t& options) const
{
    return searchPositionIK(std::vector<pose_t>{ik_pose},
                            ik_seed_state,
                            timeout,
                            std::vector<double>(),      // consistency_limits
                            solution,
                            solution_callback,
                            error_code,
                            options);
}

bool
QuikIKPlugin::searchPositionIK(const pose_t& ik_pose,
                               const std::vector<double>& ik_seed_state,
                               double timeout,
                               const std::vector<double>& consistency_limits,
                               std::vector<double>& solution,
                               const IKCallbackFn& solution_callback,
                               error_code_t& error_code,
                               const options_t& options) const
{
    return searchPositionIK(std::vector<pose_t>{ik_pose},
                            ik_seed_state,
                            timeout,
                            consistency_limits,
                            solution,
                            solution_callback,
                            error_code,
                            options);
}

bool
QuikIKPlugin::searchPositionIK(const std::vector<pose_t>& ik_poses,
                               const std::vector<double>& ik_seed_state,
                               double timeout,
                               const std::vector<double>& consistency_limits,
                               std::vector<double>& solution,
                               const IKCallbackFn& solution_callback,
                               error_code_t& error_code,
                               const options_t& options,
                               const robot_state_t* context_state) const
{
    return searchPositionIK(ik_poses,
                            ik_seed_state,
                            timeout,
                            consistency_limits,
                            solution,
                            solution_callback,
                            IKCostFn(),                 // cost_function
                            error_code,
                            options,
                            context_state);
}

QuikIKPlugin::dh_param_t
QuikIKPlugin::computeDHParam(const vector3_t& np,
			     const vector3_t& nc, const transform_t& T) const
{
    const auto	t = T.translation();
    const auto	n = T.linear() * nc;
    const auto	x = np.cross(n);
    const auto	l = x.norm();

    dh_param_t	dh;

    if (l < 0.01)
    {
	dh.a      = n.cross(t.cross(n)).norm();
	dh.alpha  = 0.0;
	dh.lambda = 0.0;
	dh.mu     = 0.0;
    }
    else
    {
	dh.a      = x.dot(t) / l;
	dh.alpha  = std::asin(l);
	dh.lambda = np.dot(n.cross(t.cross(n)))  / (l*l);
	dh.mu     = n.dot(np.cross(t.cross(np))) / (l*l);
    }
    
    return dh;
}
}  // namespace quik_ik

PLUGINLIB_EXPORT_CLASS(quik_ik::QuikIKPlugin, kinematics::KinematicsBase);

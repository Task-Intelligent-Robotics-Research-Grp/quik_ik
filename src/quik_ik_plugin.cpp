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
#include <tf2_eigen/tf2_eigen.hpp>

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

template <class K, class V> std::ostream&
operator <<(std::ostream& out, const std::map<K, V>& m)
{
    for (const auto& elm : m)
        out << ' ' << elm.first << ':' << elm.second;
    return out;
}

std::ostream&
operator <<(std::ostream& out, const geometry_msgs::msg::Pose& pose)
{
    return out << '['   << pose.position.x
               << ' '   << pose.position.y
               << ' '   << pose.position.z
               << ";  " << pose.orientation.x
               << ' '   << pose.orientation.y
               << ' '   << pose.orientation.z
               << ' '   << pose.orientation.w
               << ']';
}

std::ostream&
operator <<(std::ostream& out, const Eigen::Isometry3d& transform)
{
    const Eigen::Vector3d     t(transform.translation());
    const Eigen::Quaterniond  q(transform.linear());
    return out << '[' << t.x() << ' ' << t.y() << ' ' << t.z() << ";  "
               << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
               << ']';
}

std::ostream&
operator <<(std::ostream& out, const QuikIKPlugin::dh_param_t& dh)
{
    return out << "a=" << dh.a << ", alpha=" << dh.alpha
               << ", lambda=" << dh.lambda << ", mu=" << dh.mu
               << "\n      S=" << dh.S
               << "\n      -- S --\n"
               << dh.S.matrix();
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

bool
QuikIKPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                         const robot_model_t& robot_model,
                         const std::string& group_name,
                         const std::string& base_frame,
                         const std::vector<std::string>& tip_frames,
                         double search_discretization)
{
    using namespace moveit::core;

    std::cerr << std::fixed;

    RCLCPP_INFO_STREAM(LOGGER,
                       "quik_ik initialized: group_name=" << group_name
                       << ", base_frame=" << base_frame
                       << ", tip_frames=[" << tip_frames << ']');

    node_           = node;
    param_listener_ = std::make_shared<ParamListener>(
                          node, std::string("robot_description_kinematics.")
                          .append(group_name));

    // Initialize internal state of base class KinematicsBase.
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
    const auto  jmg = robot_model_->getJointModelGroup(group_name);
    if (!jmg)
    {
        RCLCPP_ERROR_STREAM(LOGGER,
                            "failed to get joint model group " << group_name);
        return false;
    }

    // Joint names come from jmg
    for (const auto joint : jmg->getJointModels())
        if (joint->getType() == JointModel::REVOLUTE ||
            joint->getType() == JointModel::PRISMATIC)
            joint_names_.push_back(joint->getName());

    dh_params_t         DH(joint_names_.size(), 4);
    link_types_t        link_types(joint_names_.size());
    vector_t            qsigns(joint_names_.size());
    transform_t         T = transform_t::Identity();
    transform_t         Tbase = transform_t::Identity();
    vector3_t           nz = vector3_t::Zero();
    double              mu = 0.0;
    ssize_t             joint_num = 0;
    const link_model_t* child_link;
    DH.setZero();
    for (const auto joint : jmg->getJointModels())
    {
        child_link = joint->getChildLinkModel();

        std::cerr << "### [" << joint_num
                  << "] joint_name=" << joint->getName()
                  << ", child_link=" << child_link->getName() << std::endl;

        std::cerr << "  Tlocal=" << child_link->getJointOriginTransform()
                  << std::endl;

        T = T * child_link->getJointOriginTransform();

        switch (joint->getType())
        {
          case JointModel::REVOLUTE:
          case JointModel::PRISMATIC:
          {
            std::cerr << "  T     =" << T << std::endl;

            link_names_.push_back(child_link->getName());
            link_types[joint_num] = (joint->getType() == JointModel::REVOLUTE ?
                                     quik::JOINT_REVOLUTE :
                                     quik::JOINT_PRISMATIC);
            qsigns[joint_num] = 1;

            const auto nc = (joint->getType() == JointModel::REVOLUTE ?
                             static_cast<const RevoluteJointModel*>(joint)
                             ->getAxis().normalized() :
                             static_cast<const PrismaticJointModel*>(joint)
                             ->getAxis().normalized());

          // Setup DH parameters
          //   column_0 = a:     link length.
          //   column_1 = alpha: angle between previous and current joint axis.
          //   column_2 = d:     link offset along the joint axis.
          //   column_3 = theta: joint position, theta.
            if (joint_num == 0)
            {
                Tbase = T;
                mu    = 0.0;
            }
            else
            {
                const auto dh = computeDHParam(nz, nc, T);
                std::cerr << "  dh: " << dh << std::endl;
                DH(joint_num-1, 0) = dh.a;
                DH(joint_num-1, 1) = dh.alpha;
                DH(joint_num-1, 2) = dh.lambda - mu;
                DH(joint_num-1, 3) = 0.0;
                mu                 = dh.mu;
                S_.push_back(dh.S);
            }

            T.setIdentity();
            nz = nc;
            ++joint_num;
            break;
          }
          default:
            break;
        }
    }

    // DH(joint_num-1, 2) = -mu;
    link_names_.push_back(child_link->getName());

  //Tbase = transform_t::Identity();

    std::cerr << "### DH: [a, alpha, d, theta] ###\n" << DH << std::endl;
    std::cerr << "Tbase=" << Tbase << std::endl;
    std::cerr << "Ttool=" << T << std::endl;

    robot_ = make_shared<robot_t>(DH, link_types, qsigns,
                                  Tbase.matrix(), T.matrix());

    std::cerr << "joint_names=" << getJointNames() << std::endl;
    std::cerr << "link_names=" << getLinkNames() << std::endl;

    return true;
}

QuikIKPlugin::dh_param_t
QuikIKPlugin::computeDHParam(const vector3_t& nz,
                             const vector3_t& n, const transform_t& T) const
{
    const auto  t  = T.translation();
    const auto  nc = T.linear() * n;
    auto        nx = nz.cross(nc);
    auto        l  = nx.norm();
    auto        ny = nz.cross(nx);

    std::cerr << "  t=" << t.transpose() << ", nc=" << nc.transpose()
              << ", nx=" << nx.transpose()
              << std::endl;

    dh_param_t  dh;

    if (l < 0.01)      // Is 'nc' parallel to 'nz'?
    {
        nx = nz.cross(t.cross(nz));  // 'nx' is a vertical component of 't'.
        l  = nx.norm();

        if (l < 0.0001)  // Is 't' parallel to 'nz'?
            throw std::runtime_error("Invalid link structure: two adjacent axes are colinear.");

        std::cerr << "  *** 'nc' is parralel to 'nz'. ***" << std::endl;

        nx /= l;
        ny  = nz.cross(nx);
        dh.a      = nx.dot(t);
        dh.alpha  = 0.0;
        dh.lambda = 0.0;
        dh.mu     = nz.dot(t);
    }
    else
    {
        nx /= l;
        ny /= l;
        dh.a      = nx.dot(t);
        dh.alpha  = std::asin(l);
        dh.lambda = t.dot(nc.cross(nx)) / l;
        dh.mu     = t.dot(nz.cross(nx)) / l;
    }

    const auto  c = std::cos(dh.alpha), s = std::sin(dh.alpha);
    dh.S.linear().col(0) = nx;
    dh.S.linear().col(1) =  c * ny + s * nz;
    dh.S.linear().col(2) = -s * ny + c * nz;
    dh.S.translation()   =  dh.a * nx + dh.lambda * nz;

    transform_t S = T;
    S.translation() += T.translation() + dh.mu * nc;
    std::cerr << "      -- S' --\n" << S.matrix() << std::endl;

    return dh;
}

bool
QuikIKPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                  const std::vector<double>& joint_angles,
                                  std::vector<pose_t>& poses) const
{
    RCLCPP_WARN_STREAM(LOGGER, "### BEGIN: getPositionFK()");

    return false;
}

bool
QuikIKPlugin::getPositionFKQuik(const std::vector<std::string>& link_names,
                                const std::vector<double>& joint_angles,
                                std::vector<pose_t>& poses) const
{
    RCLCPP_WARN_STREAM(LOGGER, "### BEGIN: getPositionFKQuik() [base_frame="
                       << getBaseFrame() << ']');

    if (!robot_)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "kinematics not active");
        return false;
    }

    Eigen::Matrix<double, Eigen::Dynamic, 4>    T(4*(robot_->dof + 1), 4);
    robot_->FK(map_t(joint_angles.data(), ssize_t(joint_angles.size())), T);

    poses.clear();
    for (const auto& link_name : link_names)
    {
        const auto idx = std::find(link_names_.cbegin(), link_names_.cend(),
                                   link_name) - link_names_.cbegin();
        if (size_t(idx) >= link_names.size())
        {
            RCLCPP_ERROR_STREAM(LOGGER, "getPositionFK(): unknown link name["
                                << link_name);
            return false;
        }

        std::cerr << "### link_name=" << link_name << ", idx=" << idx
                  << std::endl;

        const auto  Ti = T.template middleRows<4>(4*idx);
        const auto& S  = S_[idx];
        std::cerr << " -- Ti --\n" << Ti
                  << "\n S=" << S << std::endl;
        vector4_t   orientation;
        vector3_t   position;
        quik::geometry::hgt2quatpos((Ti * S.inverse()).matrix(),
                                    orientation, position);

        pose_t      pose;
        pose.position.x    = position.x();
        pose.position.y    = position.y();
        pose.position.z    = position.z();
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        pose.orientation.w = orientation.w();
        poses.push_back(pose);

        std::cerr << " " << link_name << ": " << pose << std::endl;
    }

    return true;
}

bool
QuikIKPlugin::getPositionFKMoveIt(const std::vector<std::string>& link_names,
                                  const std::vector<double>& joint_angles,
                                  std::vector<pose_t>& poses) const
{
    RCLCPP_WARN_STREAM(LOGGER, "### BEGIN: getPositionFKMoveIt()");

  // Set joint angles to the robot state
    robot_state_t robot_state(robot_model_);
    robot_state.setToDefaultValues();
    robot_state.setJointGroupPositions(group_name_, joint_angles);

  // Get transform from root to base_link of the group.
    const auto  Tb0 = robot_state.getGlobalLinkTransform(getBaseFrame())
                                 .inverse();

  // Get pose for each link
    poses.clear();
    for (const auto& link_name : link_names)
    {
        const auto pose = tf2::toMsg(Tb0 * robot_state.getGlobalLinkTransform(
                                               link_name));
        poses.push_back(pose);
        std::cerr << "  " << link_name << ": " << pose << std::endl;
    }

    return true;
}

bool
QuikIKPlugin::getPositionIK(const pose_t& ik_pose,
                            const std::vector<double>& ik_seed_state,
                            std::vector<double>& solution,
                            error_code_t& error_code,
                            const options_t& options) const
{
    RCLCPP_WARN_STREAM(LOGGER, "### BEGIN: getPositionIK()");

    const IKCallbackFn  solution_callback = nullptr;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            default_timeout_,
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
                               const IKCostFn& cost_function,
                               error_code_t& error_code,
                               const options_t& options,
                               const robot_state_t* context_state) const
{
    RCLCPP_WARN_STREAM(LOGGER, "### BEGIN: searchPositionIK()");

    if (!robot_)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "kinematics not active");
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    (void)context_state;  // not used

    std::vector<pose_t> poses;
    getPositionFKQuik(link_names_, ik_seed_state, poses);
    getPositionFKMoveIt(link_names_, ik_seed_state, poses);

    try
    {
        using break_reason_t = quik::BREAKREASON_t;

        if (int(ik_seed_state.size()) != robot_->dof)
            throw std::runtime_error("Provided seed_state size["
                                     + std::to_string(ik_seed_state.size())
                                     + "] is not equal to the robot's DOF["
                                     + std::to_string(robot_->dof)
                                     + "]!");

        const auto     params = param_listener_->get_params();
        const solver_t solver(robot_,
                              int(params.max_iterations),
                              quik::str2algorithm(params.algorithm),
                              params.exit_tolerance,
                              params.minimum_step_size,
                              params.relative_improvement_tolerance,
                              int(params.max_consecutive_grad_fails),
                              int(params.max_gradient_fails),
                              params.lambda_squared,
                              params.max_linear_step_size,
                              params.max_angular_step_size,
                              params.armijo_sigma,
                              params.armijo_beta);
        const map_t    q0(ik_seed_state.data(), ssize_t(ik_seed_state.size()));
        bool           found = false;

        RCLCPP_WARN_STREAM(LOGGER, "  q0=" << q0.transpose());

        for (const auto& ik_pose : ik_poses)
        {
            const vector4_t orientation(ik_pose.orientation.x,
                                        ik_pose.orientation.y,
                                        ik_pose.orientation.z,
                                        ik_pose.orientation.w);
            const vector3_t position(ik_pose.position.x,
                                     ik_pose.position.y,
                                     ik_pose.position.z);
            vector_t        q(q0.size());
            vector6_t       e;
            int             iter;
            break_reason_t  break_reason;

            RCLCPP_WARN_STREAM(LOGGER, "  desired position   =" << position.transpose());
            RCLCPP_WARN_STREAM(LOGGER, "  desired orientation=" << orientation.transpose());

            solver.IK(orientation, position, q0, q, e, iter, break_reason);

            if (break_reason == break_reason_t::BREAKREASON_TOLERANCE)
            {
                solution.clear();
                for (ssize_t i = 0; i < q.size(); ++i)
                    solution.push_back(q(i));
                found = true;
            }
            else
            {
                RCLCPP_WARN_STREAM(LOGGER,
                                   "searchPositionIK(): IK broke with reason["
                                   << quik::breakreason2str(break_reason)
                                   << ']');
            }

            if (!found)
                throw std::runtime_error("solution not found!");
        }
    }
    catch (const std::exception& err)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "searchPositionIK(): " << err.what());
        return false;
    }

    RCLCPP_WARN_STREAM(LOGGER, "### END:   searchPositionIK()");

    return true;
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

bool
QuikIKPlugin::withinLimits(const vector_t& q) const
{
    robot_state_t       robot_state(robot_model_);
    robot_state.setToDefaultValues();
    robot_state.setJointGroupPositions(group_name_, q.data());

    return robot_state.satisfiesBounds();
}
}  // namespace quik_ik

PLUGINLIB_EXPORT_CLASS(quik_ik::QuikIKPlugin, kinematics::KinematicsBase);

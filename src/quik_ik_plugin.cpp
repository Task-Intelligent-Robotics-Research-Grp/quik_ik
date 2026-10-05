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
#include <quik_ik/fk_moveit.hpp>
#include <quik_ik/goal.hpp>
#include <quik_ik/ik_gradient.hpp>
#include <quik_ik/ik_memetic.hpp>
#include <quik_ik/quik_ik_parameters.hpp>
#include <quik_ik/quik_ik_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
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

    node_ = node;
    parameter_listener_ = std::make_shared<ParamListener>(
        node,
        std::string("robot_description_kinematics.").append(group_name));

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

    dh_params_array_t DH(joint_names_.size());
    link_types_t      link_types(joint_names_.size());
    qsign_t           qsigns(joint_names_.size());
    auto              S = transform_t::Identity();    // DH coordinates
    auto              T = transform_t::Identity();
    transform_t       T01;
    vector3_t         np;
    double            lambda = 0.0;
    size_t            joint_num = 0;
    for (const auto joint : jmg_->getJointModels())
    {
        const auto child_link = joint->getChildLinkModel();

        T = T * child_link->getJointOriginTransform();

        if (joint_num == 0)
        {
            T01 = T;
            DH(0, 0) = 0.0;     // a_1
            DH(0, 1) = 0.0;     // alpha_1
            lambda   = 0.0;
        }
        else
        {
            switch (joint->getType())
            {
              case JointModel::REVOLUTE:
              {
                const auto nc = static_cast<const RevoluteJointModel*>(joint)
                              ->getAxis().normalized();
                const auto dh = computeDHParam(np, nc, T);
                DH(joint_num,   0) = dh.a;
                DH(joint_num,   1) = dh.alpha;
                DH(joint_num-1, 2) = lambda - dh.mu;
                lambda = dh.lambda;
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
                DH(joint_num,   0) = dh.a;
                DH(joint_num,   1) = dh.alpha;
                DH(joint_num-1, 2) = lambda - dh.mu;
                lambda = dh.lambda;
                link_types[joint_num++] = quik::JOINT_PRISMATIC;
                T.setIdentity();
                np = nc;
                break;
              }
              default:
                break;
            }
        }
    }

    robot_ = make_unique<robot_t>(DH, link_types, qsigns, T01, T);

    // link_names are the same as tip frames
    // TODO: why do we need to set this
    link_names_ = tip_frames_;

    tip_link_indices_ = get_link_indices(robot_model_, tip_frames_)
                       .or_else([](auto const& error)
                                { throw std::invalid_argument(error); })
                       .value();
    robot_ = Robot::from(robot_model_, jmg_, tip_link_indices_);

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

    // Read current ROS parameters
    auto params = parameter_listener_->get_params();

    auto const goal_frames = [&]()
        {
        auto robot_state = robot_state_t(robot_model_);
        robot_state.setToDefaultValues();
        robot_state.setJointGroupPositions(jmg_, ik_seed_state);
        robot_state.update();
        return transform_poses_to_frames(robot_state,
                                         ik_poses, getBaseFrame());
    }();

    // Test functions to determine if we are at our goal frame
    auto const test_position = (params.position_scale > 0);
    std::optional<double> position_threshold = std::nullopt;
    if (test_position)
    {
        position_threshold = params.position_threshold;
    }
    auto const test_rotation = (params.rotation_scale > 0);
    std::optional<double> orientation_threshold = std::nullopt;
    if (test_rotation)
    {
        orientation_threshold = params.orientation_threshold;
    }
    auto const frame_tests =
        make_frame_tests(goal_frames, position_threshold,
                         orientation_threshold);

    // Cost functions used for optimizing towards goal frames
    auto const pose_cost_functions =
        make_pose_cost_functions(goal_frames, params.position_scale,
                                 params.rotation_scale);

    // forward kinematics function
    auto const fk_fn = make_fk_fn(robot_model_, jmg_, fk_mutex_,
                                  tip_link_indices_);

    // Create goals (weighted cost functions)
    auto goals = std::vector<Goal>{};
    if (params.center_joints_weight > 0.0)
    {
        goals.push_back(Goal{make_center_joints_cost_fn(robot_),
                             params.center_joints_weight});
    }
    if (params.avoid_joint_limits_weight > 0.0)
    {
        goals.push_back(
            Goal{make_avoid_joint_limits_cost_fn(robot_),
                 params.avoid_joint_limits_weight});
    }
    if (params.minimal_displacement_weight > 0.0)
    {
        goals.push_back(Goal{make_minimal_displacement_cost_fn(robot_, ik_seed_state),
                             params.minimal_displacement_weight});
    }
    if (cost_function)
    {
        for (auto const& pose : ik_poses)
        {
            goals.push_back(
                Goal{make_ik_cost_fn(pose, cost_function, robot_model_, jmg_, ik_seed_state), 1.0});
        }
    }

    // test if this is a valid solution
    auto const solution_fn =
        make_is_solution_test_fn(frame_tests, goals, params.cost_threshold,
                                 fk_fn);

    // single function used by gradient descent to calculate cost of solution
    auto const cost_fn = make_cost_fn(pose_cost_functions, goals, fk_fn);

    // Set up initial optimization variables
    bool done_optimizing = false;
    bool found_valid_solution = false;
    double remaining_timeout = timeout;
    std::chrono::duration<double> total_optim_time{0.0};
    std::chrono::duration<double> const total_timeout{timeout};
    auto last_optim_time = std::chrono::system_clock::now();

    // If the initial state is not valid, restart from a random valid state.
    auto init_state = ik_seed_state;
    if (!robot_.is_valid_configuration(init_state))
    {
        RCLCPP_WARN(
            LOGGER,
            "Initial guess exceeds joint limits. Regenerating a random valid configuration.");
        robot_.set_random_valid_configuration(init_state);
    }

    // Optimize until a valid solution is found or we have timed out.
    while (!done_optimizing)
    {
        // Search for a solution using either the local or global solver.
        std::optional<std::vector<double>> maybe_solution;
        if (params.mode == "global")
        {
            MemeticIkParams ik_params;
            ik_params.population_size = static_cast<size_t>(params.memetic_population_size);
            ik_params.elite_size = static_cast<size_t>(params.memetic_elite_size);
            ik_params.wipeout_fitness_tol = params.memetic_wipeout_fitness_tol;
            ik_params.stop_optimization_on_valid_solution =
                params.stop_optimization_on_valid_solution;
            ik_params.num_threads = static_cast<size_t>(params.memetic_num_threads);
            ik_params.stop_on_first_soln = params.memetic_stop_on_first_solution;
            ik_params.max_generations = static_cast<int>(params.memetic_max_generations);
            ik_params.max_time = remaining_timeout;

            ik_params.gd_params.step_size = params.gd_step_size;
            ik_params.gd_params.min_cost_delta = params.gd_min_cost_delta;
            ik_params.gd_params.max_iterations = static_cast<int>(params.memetic_gd_max_iters);
            ik_params.gd_params.max_time = params.memetic_gd_max_time;

            maybe_solution = ik_memetic(ik_seed_state,
                                        robot_,
                                        cost_fn,
                                        solution_fn,
                                        ik_params,
                                        options.return_approximate_solution,
                                        false /* No debug print */);
        }
        else if (params.mode == "local")
        {
            GradientIkParams gd_params;
            gd_params.step_size = params.gd_step_size;
            gd_params.min_cost_delta = params.gd_min_cost_delta;
            gd_params.max_time = remaining_timeout;
            gd_params.max_iterations = static_cast<int>(params.gd_max_iters);
            gd_params.stop_optimization_on_valid_solution =
                params.stop_optimization_on_valid_solution;

            maybe_solution = ik_gradient(ik_seed_state,
                                         robot_,
                                         cost_fn,
                                         solution_fn,
                                         gd_params,
                                         options.return_approximate_solution);
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Invalid solver mode: %s", params.mode.c_str());
            return false;
        }

        if (maybe_solution.has_value())
        {
            // Set the output parameter solution.
            // Assumes that the angles were already wrapped by the solver.
            error_code.val = error_code.SUCCESS;
            solution = maybe_solution.value();
        }
        else
        {
            error_code.val = error_code.NO_IK_SOLUTION;
            solution = ik_seed_state;
        }

        // If using an approximate solution, check against the maximum allowable pose and joint
        // thresholds. If the approximate solution is too far from the goal frame,
        // fall back to the initial state.
        if (options.return_approximate_solution)
        {
            // Check pose thresholds
            std::optional<double> approximate_solution_position_threshold = std::nullopt;
            if (test_position)
            {
                approximate_solution_position_threshold =
                    params.approximate_solution_position_threshold;
            }
            std::optional<double> approximate_solution_orientation_threshold = std::nullopt;
            if (test_rotation)
            {
                approximate_solution_orientation_threshold =
                    params.approximate_solution_orientation_threshold;
            }
            auto const approx_frame_tests =
                make_frame_tests(goal_frames,
                                 approximate_solution_position_threshold,
                                 approximate_solution_orientation_threshold);

            // If we have no cost threshold, we don't need to check the goals
            if (params.approximate_solution_cost_threshold <= 0.0) {
                goals.clear();
            }

            auto const approx_solution_fn =
                make_is_solution_test_fn(frame_tests,
                                         goals,
                                         params.approximate_solution_cost_threshold,
                                         fk_fn);

            bool approx_solution_valid = approx_solution_fn(solution);

            // Check joint thresholds
            if (approx_solution_valid && params.approximate_solution_joint_threshold > 0.0) {
                for (size_t i = 0; i < solution.size(); ++i)
                {
                    if (std::abs(solution[i] - ik_seed_state[i]) >
                        params.approximate_solution_joint_threshold)
                    {
                        approx_solution_valid = false;
                        break;
                    }
                }
            }

            if (!approx_solution_valid)
            {
                error_code.val = error_code.NO_IK_SOLUTION;
                solution = ik_seed_state;
            }
        }

        // Execute solution callback only on successful solution.
        auto const found_solution = error_code.val == error_code.SUCCESS;
        if (solution_callback && found_solution) {
            solution_callback(ik_poses.front(), solution, error_code);
        }
        found_valid_solution = error_code.val == error_code.SUCCESS;

        // Check for timeout.
        auto const current_time = std::chrono::system_clock::now();
        total_optim_time += (current_time - last_optim_time);
        last_optim_time = current_time;
        bool const timeout_elapsed = (total_optim_time >= total_timeout);

        // If we found a valid solution or hit the timeout, we are done optimizing.
        // Otherwise, quik a random new initial seed and keep optimizing with the remaining
        // time.
        if (found_valid_solution || timeout_elapsed)
        {
            done_optimizing = true;
        }
        else
        {
            robot_.set_random_valid_configuration(init_state);
            remaining_timeout = timeout - total_optim_time.count();
        }
    }

    return found_valid_solution;
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

QuikIKPlugin::transform_t
QuikIKPlugin::computeDHParams(const transform_t& S, const transform_t& T,
                              const joint_model_t& joint,
                              dh_params_t& dh_params,
                              quik::JOINTTYPE_t& link_type, double& qsign)
{
    using namespace moveit::core;

    Eigen::Vector3d     axis;

    if (joint.getType() == JointModel::REVOLUTE)
    {
        axis = static_cast<const RevoluteJointModel&>(joint).getAxis()
                                                            .normalized();
        link_type = quik::JOINT_REVOLUTE;
    }
    else
    {
        axis = static_cast<const PrismaticJointModel&>(joint).getAxis()
                                                             .normalized();
        link_type = quik::JOINT_PRISMATIC;
    }
}

bool
QuikIKPlugin::getDHParams(std::vector<DHParam>& dhparameters,
                          const joint_model_t* joint,
                          const transform_t& parentWorldTransform,
                          const Eigen::Vector3d& parentX,
                          const Eigen::Vector3d& parentZ,
                          const Eigen::Vector3d& parentPos,
                          bool asRootJoint,
                          EigenTransform& parentWorldTransformDH) const
{
    const auto* child_link = joint->getChildLinkModel();
    if (!child_link)
    {
        RCLCPP_ERROR(LOGGER, "consistency, no child link");
        return false;
    }

//    ROS_INFO_STREAM("Transform is for link "<<child_link->name);
/*    LinkConstPtr parentLink = trav->readLink(joint->parent_link_name);
    ROS_INFO_STREAM("Parent is "<<parentLink->name); */

    Eigen::Vector3d z, pos, x;

    EigenTransform jointWorldTransform = EigenTransform::Identity();
    EigenTransform jointWorldTransformDH = EigenTransform::Identity();

    if (asRootJoint)
    {
        // if this is the root joint, we won't consider the joint's
        // previous transforms, as we'll set this one to be the origin
        jointWorldTransform.setIdentity();
        parentWorldTransformDH.setIdentity();

        z = urdf_traverser::getRotationAxis(joint);
        pos = Eigen::Vector3d(0, 0, 0);
        x = parentX;
    }
    else
    {
        EigenTransform jointTransform = urdf_traverser::getTransform(joint);
        jointWorldTransform = parentWorldTransform * jointTransform;
        // ROS_INFO_STREAM("Parent world transform: "<<parentWorldTransform);
        // ROS_INFO_STREAM("Joint transform: "<<jointTransform);
        // ROS_INFO_STREAM("Joint world transform: "<<jointWorldTransform);

        // the global rotation axis of the joint is going to be the
        // new global z-axis of the DH joint transform.
        // the position is the global position of the joint.
        getGlobalCoordinates(joint, parentWorldTransform, z, pos);
        // ROS_INFO_STREAM("Global rotation axis of joint "<<joint->name<<": "<<z<<", pos "<<pos);

        DHParam param;
        if (!DHParam::toDenavitHartenberg(param, parentZ, parentX, parentPos, z, pos, x))
        {
            ROS_ERROR("could not obtain dh params");
            return false;
        }

        // The global pose of the DH joint may be different to the global pose
        // of the URDF joint. Calculate the pose of the DH joint in world coordinates
        // and apply it to \e pos, so that the recursion can take this into account.
        // ROS_INFO_STREAM("Translating  "<<parentPos<<" ( target: "<<pos<<" ) using transform = "<<parentWorldTransformDH);

        EigenTransform jointTransformDH = EigenTransform::Identity();
        Eigen::Vector3d z(0,0,1);
        jointTransformDH.translate(z*param.d);
        jointTransformDH.rotate(Eigen::AngleAxisd(param.theta,z));
        Eigen::Vector3d x(1,0,0);
        jointTransformDH.translate(x*param.r);
        jointTransformDH.rotate(Eigen::AngleAxisd(param.alpha,x));

        jointWorldTransformDH = parentWorldTransformDH * jointTransformDH;

        /*ROS_INFO_STREAM("parentWorldTransformDH: "<<jointTransformDH);
        ROS_INFO_STREAM("joint transform DH: "<<jointTransformDH);
        ROS_INFO_STREAM("joint world transform DH: "<<jointWorldTransformDH);*/

        Eigen::Vector3d pi_new = jointWorldTransformDH.translation();
        // ROS_INFO_STREAM("Transformed DH pose: "<<pi_new<<" ( vs URDF space "<<pos<<" )");
        pos = pi_new;

        param.dof_index = dhparameters.size();
        param.joint = trav->readParentJoint(joint);

        if (!param.joint)
        {
            ROS_ERROR_STREAM("Consistency: Joint "<<joint->name
                <<" has no parent, should have been added as root joint instead!");
            return false;
        }
        LinkConstPtr paramChildLink = trav->readLink(param.joint->child_link_name);
        if (!paramChildLink)
        {
            ROS_ERROR("consistency, no child link");
            return false;
        }
        param.child_link = paramChildLink;
        dhparameters.push_back(param);
        // ROS_INFO_STREAM("DH for Joint "<<param.joint->name<<" parent axes: z="<<z<<", x="<<x<<", pos="<<pos<<": "<<param);
    }


    if (child_link->child_joints.empty())
    {
        // we've come to the end of the chain, so we have to add the parameter for the last frame.
        // while r should technically be the link length, we'll just leave this frame at the previous joint's
        // frame, as the DH parameters for the last joint don't really matter, they only specify the location
        // of a joint that would follow this one.

        DHParam param;
        param.dof_index = dhparameters.size();
        param.joint = joint;
        param.child_link = child_link;

        param.r = 0;
        param.d = 0;
        param.theta = 0;
        param.alpha = 0;

        dhparameters.push_back(param);
        // ROS_INFO_STREAM("DH for Joint "<<param.joint->name<<", parent axes: z="<<z<<", x="<<x<<", pos="<<pos<<": "<<param);
        return true;
    }


    // recurse to child joints
    for (std::vector<JointPtr>::const_iterator pj = child_link->child_joints.begin();
            pj != child_link->child_joints.end(); pj++)
    {
        if (!getDHParams(dhparameters, *pj, jointWorldTransform, x, z,
                         pos, false, jointWorldTransformDH))
            return false;
    }

    return true;
}

QuikIKPlugin::dh_param_t
QuikIKPlugin::computeDHParam(const vector3_t& np, const vector3_t& nc,
                             const transform_t& T) const
{
}
}  // namespace quik_ik

PLUGINLIB_EXPORT_CLASS(quik_ik::QuikIKPlugin, kinematics::KinematicsBase);

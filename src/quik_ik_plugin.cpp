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
//#include <quik_ik/quik_ik_parameters.hpp>
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

    dh_params_t  DH(joint_names_.size());
    link_types_t link_types(joint_names_.size());
    qsign_t      qsigns(joint_names_.size());
    auto         T = transform_t::Identity();
    transform_t  Tbase;
    vector3_t    np;
    double       lambda = 0.0;
    size_t       joint_num = 0;
    for (const auto joint : jmg_->getJointModels())
    {
        const auto child_link = joint->getChildLinkModel();

        T = T * child_link->getJointOriginTransform();

        if (joint_num == 0)
        {
            Tbase    = T;
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
                lambda             = dh.lambda;
		qsigns[joint_num]  = 
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

    DH(joint_num-1, 2) = lambda;

    robot_ = make_unique<robot_t>(DH, link_types, qsigns, Tbase, T);
    solver_ = make_unique<solver_t>(
                  robot_, 200, // max number of iterations
                  quik::ALGORITHM_QUIK,
                  1e-12, // Exit tolerance
                  1e-14, // Minimum step tolerance
                  0.05, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
                  10, // max consequitive gradient fails
                  80, // Max gradient fails
                  1e-10, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
                  0.34, // Max linear error step
                  1 // Max angular error step
        );
    
    // link_names are the same as tip frames
    // TODO: why do we need to set this
    link_names_ = tip_frames_;

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

    // Read current ROS parameters
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

bool
QuikIKPlugin::getDHParams(std::vector<quik::DHParam>& dhparameters,
                          const joint_model_t* joint,
                          const transform_t& parentWorldTransform,
                          const vector3_t& parentX,
                          const vector3_t& parentZ,
                          const vector3_t& parentPos,
                          bool asRootJoint,
                          transform_t& parentWorldTransformDH) const
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

    vector3_t z, pos, x;

    transform_t jointWorldTransform = transform_t::Identity();
    transform_t jointWorldTransformDH = transform_t::Identity();

    if (asRootJoint)
    {
        // if this is the root joint, we won't consider the joint's
        // previous transforms, as we'll set this one to be the origin
        jointWorldTransform.setIdentity();
        parentWorldTransformDH.setIdentity();

        z = urdf_traverser::getRotationAxis(joint);
        pos = vector3_t(0, 0, 0);
        x = parentX;
    }
    else
    {
        transform_t jointTransform = urdf_traverser::getTransform(joint);
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
            RCLCPP_ERROR("could not obtain dh params");
            return false;
        }

        // The global pose of the DH joint may be different to the global pose
        // of the URDF joint. Calculate the pose of the DH joint in world coordinates
        // and apply it to \e pos, so that the recursion can take this into account.
        // ROS_INFO_STREAM("Translating  "<<parentPos<<" ( target: "<<pos<<" ) using transform = "<<parentWorldTransformDH);

        transform_t jointTransformDH = transform_t::Identity();
        vector3_t z(0,0,1);
        jointTransformDH.translate(z*param.d);
        jointTransformDH.rotate(Eigen::AngleAxisd(param.theta,z));
        vector3_t x(1,0,0);
        jointTransformDH.translate(x*param.r);
        jointTransformDH.rotate(Eigen::AngleAxisd(param.alpha,x));

        jointWorldTransformDH = parentWorldTransformDH * jointTransformDH;

        /*ROS_INFO_STREAM("parentWorldTransformDH: "<<jointTransformDH);
        ROS_INFO_STREAM("joint transform DH: "<<jointTransformDH);
        ROS_INFO_STREAM("joint world transform DH: "<<jointWorldTransformDH);*/

        vector3_t pi_new = jointWorldTransformDH.translation();
        // ROS_INFO_STREAM("Transformed DH pose: "<<pi_new<<" ( vs URDF space "<<pos<<" )");
        pos = pi_new;

        param.dof_index = dhparameters.size();
        param.joint = trav->readParentJoint(joint);

        if (!param.joint)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Consistency: Joint "<<joint->name
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
}  // namespace quik_ik

PLUGINLIB_EXPORT_CLASS(quik_ik::QuikIKPlugin, kinematics::KinematicsBase);

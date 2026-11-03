/**
 * @file quik_ik_plugin.cpp
 * @author Toshio Ueshiba (t.ueshiba@aist.go.jp)
 * @date 2026-11-2
 * @brief Implementation of QuikIKPlugin class
 * @copyright Copyright (c) 2026
 */
#define DEBUG

#include <quik_ik/quik_ik_parameters.hpp>
#include <quik_ik/quik_ik_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace quik_ik
{
namespace
{
/************************************************************************
*  static variables and functions                                       *
************************************************************************/
const auto LOGGER = rclcpp::get_logger("quik_ik");

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
    return out << "theta0=" << dh.theta0 << ", a=" << dh.a
               << ", d=" << dh.d << ", alpha=" << dh.alpha
               << ", mu=" << dh.mu
               << "\n-- U --\n" << dh.U.matrix()
               << "\n-- D --\n" << dh.D().matrix()
               << "\n-- S --\n" << dh.S().matrix();
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
                         const moveit::core::RobotModel& robot_model,
                         const std::string& group_name,
                         const std::string& base_frame,
                         const std::vector<std::string>& tip_frames,
                         double search_discretization)
{
    using namespace Eigen;
    using namespace moveit::core;

    std::cerr << std::fixed;

    RCLCPP_INFO_STREAM(LOGGER,
                       "initialize quik_ik: group_name=" << group_name
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
    if (joint_names_.size() == 0)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "no active joints in joint model group["
                            << group_name << ']');
        return false;
    }

    Array<double, Dynamic, 4>   DH(joint_names_.size(), 4);
    VectorX<quik::JOINTTYPE_t>  link_types(joint_names_.size());
    VectorXd                    qsigns(joint_names_.size());
    auto                        T = Isometry3d::Identity();
    auto                        Tbase = Isometry3d::Identity();
    Vector3d                    ip = Vector3d::Zero(), kp = Vector3d::Zero();
    double                      mu = 0.0;
    ssize_t                     joint_num = 0;
    const LinkModel*            child_link;
    
    for (const auto joint : jmg->getJointModels())
    {
        child_link = joint->getChildLinkModel();
        T = T * child_link->getJointOriginTransform();
#if defined(DEBUG)
        std::cerr << "\n### [" << joint_num
                  << "] joint_name=" << joint->getName()
                  << "\n  T =" << T << std::endl;
#endif
        switch (joint->getType())
        {
          case JointModel::REVOLUTE:
          case JointModel::PRISMATIC:
          {
            link_names_.push_back(child_link->getName());
            link_types[joint_num] = (joint->getType() == JointModel::REVOLUTE ?
                                     quik::JOINT_REVOLUTE :
                                     quik::JOINT_PRISMATIC);
            qsigns[joint_num] = 1;

            const auto k = (joint->getType() == JointModel::REVOLUTE ?
                            static_cast<const RevoluteJointModel*>(joint)
                            ->getAxis().normalized() :
                            static_cast<const PrismaticJointModel*>(joint)
                            ->getAxis().normalized());

            if (joint_num == 0)
            {
                Tbase = T;
            }
            else
            {
              // Setup DH parameters
              //   column_0 = a:      link length
              //   column_1 = alpha:  angle between parent and child joint axis
              //   column_2 = d:      link offset along the parent joint axis
              //   column_3 = theta0: parent joint reference position
                try
                {
                    const auto dh = computeDHParam(k, kp, ip, mu, T);
#if defined(DEBUG)
                    std::cerr << "  dh: " << dh << std::endl;
#endif
                    DH(joint_num-1, 0) = dh.a;
                    DH(joint_num-1, 1) = dh.alpha;
                    DH(joint_num-1, 2) = dh.d;
                    DH(joint_num-1, 3) = dh.theta0;
                    ip                 = dh.ip;
                    mu                 = dh.mu;
                    S_.push_back(dh.S());

                    if (joint_num == 1)
                        Tbase = Tbase * dh.U;
                }
                catch (const std::exception& err)
                {
                    RCLCPP_ERROR_STREAM(LOGGER, err.what());
                    return false;
                }
            }

            T.setIdentity();
            kp = k;
            ++joint_num;
            break;
          }
          default:
            break;
        }
    }

  // Set DH parameters for the last active joint.
    DH(joint_num-1, 0) = 0.0;
    DH(joint_num-1, 1) = 0.0;
    DH(joint_num-1, 2) = -mu;
    DH(joint_num-1, 3) = 0.0;

  // Set transformation from DH frame to MoveIt frame for the child link
  // of the last active joint.
    Isometry3d S;
    S.linear().col(0) = ip;
    S.linear().col(1) = kp.cross(ip);
    S.linear().col(2) = kp;
    S.translation()   = Vector3d::Zero();
    S_.push_back(S);

  // Compute transformation from the tip frame to the DH frame of the child
  // link of the last active joint.
    T = S.inverse() * T;

  // Set name and transformation for the tip link.
    link_names_.push_back(child_link->getName());
    S_.push_back(Isometry3d::Identity());

#if defined(DEBUG)
    std::cerr << "\n### DH: [a, alpha, d, theta] ###\n" << DH
              << "\nTbase=" << Tbase << "\nTtool=" << T << std::endl;
#endif
  // Create a link structure in Denevit-Hartenburg(DH) form.
    robot_ = make_shared<quik::Robot<> >(DH, link_types, qsigns,
                                         Tbase.matrix(), T.matrix());

    RCLCPP_INFO_STREAM(LOGGER,
                       "quik_ik successfully initialized: joint_names="
                       << getJointNames()
                       << ", link_names=" << getLinkNames());

    return true;
}

QuikIKPlugin::dh_param_t
QuikIKPlugin::computeDHParam(const Eigen::Vector3d& k,
                             const Eigen::Vector3d& kp,
                             const Eigen::Vector3d& ip, double mu,
                             const Eigen::Isometry3d& T) const
{
    using namespace Eigen;
    
    const auto t   = T.translation();  // Translation from parent to child link
    const auto kc  = T.linear() * k;   // Transform child axis to parent frame
    auto       ic  = kp.cross(kc);     // Child link direction in parent frame
    const auto c   = kp.dot(kc);
    const auto det = (1.0 + c) * (1.0 - c);
    
    dh_param_t dh;
    dh.alpha = std::asin(std::sqrt(det));
    
    if (l < 0.01)  // Is 'kc' parallel to 'kp'?
    {
        ic = t - t.dot(kp) * kp;       // 'ic' is a vertical component of 't'.
        l  = ic.norm();

        if (l < 0.0001)                // Is 't' parallel to 'k'?
            throw std::runtime_error("Invalid link structure: two adjacent axes are colinear.");
    }

    const auto w = std::exp(-param_listener_->get_params().dh_weight * det);
    Matrix2d   A;
    A << 1.0 + w, -c, -c, 1.0;
    Vector2d   b(kp.dot(t), -kc.dot(t));
    const auto x = A.colPivHouseholderQr().solve(b);
    dh.d  = x(0) - mu;
    dh.mu = x(1);

    ic /= l;
    dh.a = ic.dot(t);

    if (ip.dot(ip) < 0.01)              // Is 'kp' the first active joint?
    {
        dh.theta0 = 0.0;
        dh.U.linear().col(0) = ic;
    }
    else
    {
        dh.theta0 = std::acos(ip.dot(ic));
        dh.U.linear().col(0) = ip;
    }

    dh.U.linear().col(1) = kp.cross(dh.U.linear().col(0));
    dh.U.linear().col(2) = kp;
    dh.U.translation()   = mu * kp;

  // Transfrom child link direction into child joint frame.
    dh.ip = T.linear().transpose() * ic;
#if defined(DEBUG)
    std::cerr << "  kp=" << kp.transpose() << ", ip=" << ip.transpose()
              << ",\n  kc=" << kc.transpose() << ", ic="  <<  ic.transpose()
              << ",\n  t=" << t.transpose()
              << std::endl;
#endif
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
    using namespace Eigen;
    
    RCLCPP_WARN_STREAM(LOGGER, "### BEGIN: getPositionFKQuik() [base_frame="
                       << getBaseFrame() << ']');

    if (!robot_)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "kinematics not active");
        return false;
    }

    Matrix<double, Dynamic, 4> T(4*(robot_->dof + 1), 4);
    robot_->FK(Map<const VectorXd>(joint_angles.data(),
                                   ssize_t(joint_angles.size())), T);

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

        const auto  Tdh = T.template middleRows<4>(4*idx);
        const auto& S   = S_[size_t(idx)];
#if defined(DEBUG)
        // std::cerr << "### [" << idx << "] link_name=" << link_name
        //           << "\n -- Tdh --\n" << Tdh
        //           << "\n -- S --\n" << S.matrix()
        //           << std::endl;
#endif
        Vector4d orientation;
        Vector3d position;
        quik::geometry::hgt2quatpos((Tdh * S.inverse()).matrix(),
                                    orientation, position);

        pose_t pose;
        pose.position.x    = position.x();
        pose.position.y    = position.y();
        pose.position.z    = position.z();
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        pose.orientation.w = orientation.w();
        poses.push_back(pose);
#if defined(DEBUG)
        std::cerr << " " << link_name << ": " << pose << std::endl;
#endif
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
#if defined(DEBUG)
        std::cerr << "  " << link_name << ": " << pose << std::endl;
#endif
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
    RCLCPP_DEBUG_STREAM(LOGGER, "### BEGIN: getPositionIK()");

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
    RCLCPP_DEBUG_STREAM(LOGGER, "### BEGIN: searchPositionIK()");

    if (!robot_)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "kinematics not active");
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    (void)context_state;  // not used

#if defined(DEBUG)
    std::vector<pose_t> poses;
    getPositionFKQuik(link_names_, ik_seed_state, poses);
    getPositionFKMoveIt(link_names_, ik_seed_state, poses);
#endif

    try
    {
        using namespace Eigen;
        using break_reason_t = quik::BREAKREASON_t;

        if (int(ik_seed_state.size()) != robot_->dof)
            throw std::runtime_error("Provided seed_state size["
                                     + std::to_string(ik_seed_state.size())
                                     + "] is not equal to the robot's DOF["
                                     + std::to_string(robot_->dof)
                                     + "]!");

        const auto                params = param_listener_->get_params();
        const quik::IKSolver<>    solver(robot_,
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
        const Map<const VectorXd> q0(ik_seed_state.data(),
                                     ssize_t(ik_seed_state.size()));
        bool                      found = false;

        RCLCPP_DEBUG_STREAM(LOGGER, "q0=" << q0.transpose());

        for (const auto& ik_pose : ik_poses)
        {
            const Vector4d    orientation(ik_pose.orientation.x,
                                          ik_pose.orientation.y,
                                          ik_pose.orientation.z,
                                          ik_pose.orientation.w);
            const Vector3d    position(ik_pose.position.x,
                                       ik_pose.position.y,
                                       ik_pose.position.z);
            VectorXd          q(q0.size());
            Vector<double, 6> e;
            int               iter;
            break_reason_t    break_reason;

            RCLCPP_DEBUG_STREAM(LOGGER, "desired position   ="
                                << position.transpose());
            RCLCPP_DEBUG_STREAM(LOGGER, "desired orientation="
                                << orientation.transpose());

          // Solve for joint positions, q, from Cartesian position
          // and orientation.
            solver.IK(orientation, position, q0, q, e, iter, break_reason);

            if (break_reason == break_reason_t::BREAKREASON_TOLERANCE)
            {
                solution.resize(q.size());
                std::copy_n(q.data(), q.size(), solution.begin());
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
QuikIKPlugin::withinLimits(const Eigen::VectorXd& q) const
{
    robot_state_t       robot_state(robot_model_);
    robot_state.setToDefaultValues();
    robot_state.setJointGroupPositions(group_name_, q.data());

    return robot_state.satisfiesBounds();
}
}  // namespace quik_ik

PLUGINLIB_EXPORT_CLASS(quik_ik::QuikIKPlugin, kinematics::KinematicsBase);

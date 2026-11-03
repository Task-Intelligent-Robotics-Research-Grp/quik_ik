/**
 * @file quik_ik_plugin.hpp
 * @author Toshio Ueshiba (t.ueshiba@aist.go.jp)
 * @date 2026-11-2
 * @brief Header file of QuikIKPlugin class
 * @copyright Copyright (c) 2026
 */
#pragma once

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
    using error_code_t  = moveit_msgs::msg::MoveItErrorCodes;
    using options_t     = kinematics::KinematicsQueryOptions;
    using robot_state_t = moveit::core::RobotState;

  public:
    struct dh_param_t
    {
        double          theta0 = 0.0;
        double          d      = 0.0;  // link offset along parent joint axis
        double          a      = 0.0;  // distance between adjacent joint axes
        double          alpha  = 0.0;  // angle between adjacent joint axes
        double          mu     = 0.0;  // origin offset along child joint axis
        Eigen::Vector3d ip     = Eigen::Vector3d::Zero();
        Eigen::Isometry3d U    = Eigen::Isometry3d::Identity();

        Eigen::Isometry3d       D() const
                                {
                                    const auto  ct = std::cos(theta0),
                                                st = std::sin(theta0);
                                    const auto  ca = std::cos(alpha),
                                                sa = std::sin(alpha);
                                    Eigen::Isometry3d   T;
                                    T.matrix() << ct, -st*ca,  st*sa, ct*a,
                                                  st,  ct*ca, -ct*sa, st*a,
                                                   0,     sa,     ca,    d,
                                                   0,      0,      0,    1;
                                    return T;
                                }

      // Transform from DH frame origin on child axis to URDF frame origin
        Eigen::Isometry3d       S()             const   { return U * D(); }
    };

  public:
    const std::vector<std::string>& getJointNames()             const   ;
    const std::vector<std::string>& getLinkNames()              const   ;

    bool initialize(const rclcpp::Node::SharedPtr& node,
                    const moveit::core::RobotModel& robot_model,
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
    dh_param_t  computeDHParam(const Eigen::Vector3d& k,
                               const Eigen::Vector3d& ip,
                               const Eigen::Vector3d& kp,
                               double mu,
                               const Eigen::Isometry3d& T)      const   ;
    bool        getPositionFKQuik(const std::vector<std::string>& link_names,
                                  const std::vector<double>& joint_angles,
                                  std::vector<pose_t>& poses)   const   ;
    bool        getPositionFKMoveIt(const std::vector<std::string>& link_names,
                                    const std::vector<double>& joint_angles,
                                    std::vector<pose_t>& poses) const   ;
    bool        withinLimits(const Eigen::VectorXd& q)          const   ;

  private:
    rclcpp::Node::SharedPtr             node_;
    std::shared_ptr<ParamListener>      param_listener_;

    std::vector<std::string>            joint_names_;
    std::vector<std::string>            link_names_;
    std::vector<Eigen::Isometry3d>      S_;
    std::shared_ptr<quik::Robot<> >     robot_;
};
}  // namespace quik_ik

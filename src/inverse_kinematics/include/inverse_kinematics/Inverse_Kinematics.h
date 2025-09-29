#pragma once

#include "Analytical_Solution.h"
#include <iostream>
#include <moveit/kinematics_base/kinematics_base.h>
#include <eigen3/Eigen/Eigen>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include "rclcpp/rclcpp.hpp"

#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

namespace Inverse_Kinematics_Plugin
{

    class IKPlugin : public kinematics::KinematicsBase
    {
    public:
        // 初始化函数
        bool initialize(
            const rclcpp::Node::SharedPtr &node,
            const moveit::core::RobotModel &robot_model,
            const std::string &group_name,
            const std::string &base_frame,
            const std::vector<std::string> &tip_frames,
            double search_discretization) override;

        // getPositionIK
        bool getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                           const std::vector<double> &ik_seed_state,
                           std::vector<double> &solution,
                           moveit_msgs::msg::MoveItErrorCodes &error_code,
                           const kinematics::KinematicsQueryOptions &options =
                               kinematics::KinematicsQueryOptions()) const override;

        // getPositionFK
        bool getPositionFK(const std::vector<std::string> &link_names,
                           const std::vector<double> &joint_angles,
                           std::vector<geometry_msgs::msg::Pose> &poses) const override;

        // supportsGroup
        bool supportsGroup(const moveit::core::JointModelGroup *group,
                           std::string *msg = nullptr) const override;

        // 关节/链路名称接口
        const std::vector<std::string> &getJointNames() const override { return joint_names_; }
        const std::vector<std::string> &getLinkNames() const override { return link_names_; }

        // 1
        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            std::vector<double> &solution,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

        // 2
        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            const std::vector<double> &consistency_limits,
            std::vector<double> &solution,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

        // 3
        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            std::vector<double> &solution,
            const kinematics::KinematicsBase::IKCallbackFn &solution_callback,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;
        // 4
        bool searchPositionIK(
            const geometry_msgs::msg::Pose &ik_pose,
            const std::vector<double> &ik_seed_state,
            double timeout,
            const std::vector<double> &consistency_limits,
            std::vector<double> &solution,
            const kinematics::KinematicsBase::IKCallbackFn &solution_callback,
            moveit_msgs::msg::MoveItErrorCodes &error_code,
            const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

    private:
        std::vector<std::string> joint_names_, link_names_; // 保存关节组里各关节和末端的名字
        std::string group_names_;
        mutable std::vector<double> current_joint_positions_; // 添加当前状态存储
        mutable std::mutex state_mutex_;                      // 线程安全
        std::unique_ptr<IKSolution> ik_solver_;               // 逆运动学求解器
        rclcpp::Node::SharedPtr node_;
    };
}

#endif
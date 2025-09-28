#include "inverse_kinematics/Inverse_Kinematics.h"
#include "inverse_kinematics/Analytical_Solution.h"
#include <pluginlib/class_list_macros.hpp>
#include <moveit/robot_state/robot_state.h>
#include "rclcpp/rclcpp.hpp"

namespace Inverse_Kinematics_Plugin
{
    bool IKPlugin::initialize(const rclcpp::Node::SharedPtr &node,
                              const moveit::core::RobotModel &robot_model,
                              const std::string &group_name,
                              const std::string &base_frame,
                              const std::vector<std::string> &tip_frames,
                              double search_discretization)
    {
        (void)base_frame;
        (void)search_discretization;
        node_ = node;
        group_names_ = group_name;
        robot_model_ = std::const_pointer_cast<const moveit::core::RobotModel>(
            std::shared_ptr<moveit::core::RobotModel>(const_cast<moveit::core::RobotModel *>(&robot_model)));

        if (!node_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("IKPlugin"), "传入的 node 指针为空！");
            return false;
        }

        // 输出组名称信息
        RCLCPP_INFO(node_->get_logger(), "初始化规划组: %s", group_name.c_str());

        // 获取关节模型组并验证
        const auto *jmg = robot_model.getJointModelGroup(group_name);
        if (!jmg || jmg->getActiveVariableCount() == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "规划组 %s 不存在或无活动关节", group_name.c_str());
            return false;
        }

        // 获取并输出关节信息
        joint_names_ = jmg->getActiveJointModelNames();
        RCLCPP_INFO(node_->get_logger(), "规划组 %s 的活动关节数: %lu",
                    group_name.c_str(), joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "  关节 %lu: %s", i + 1, joint_names_[i].c_str());
        }

        // 获取并输出末端帧信息
        link_names_ = tip_frames;
        RCLCPP_INFO(node_->get_logger(), "末端帧(tip_frames)数量: %lu", link_names_.size());
        for (size_t i = 0; i < link_names_.size(); ++i)
        {
            RCLCPP_INFO(node_->get_logger(), "  末端帧 %lu: %s", i + 1, link_names_[i].c_str());
        }

        // 初始化IK求解器
        ik_solver_ = std::make_unique<IKSolution>(robot_model);
        if (!ik_solver_->initIK(joint_names_, robot_model))
        {
            RCLCPP_ERROR(node_->get_logger(), "IK 求解器初始化失败！");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "IKPlugin 初始化成功！关节数：%lu", joint_names_.size());
        return true;
    }

    bool IKPlugin::getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                 const std::vector<double> &ik_seed_state,
                                 std::vector<double> &solution,
                                 moveit_msgs::msg::MoveItErrorCodes &error_code,
                                 const kinematics::KinematicsQueryOptions &) const
    {
        ik_solver_->readCurrentStatus(ik_seed_state);
        solution = ik_solver_->AS(ik_pose);
        if (solution.empty())
        {
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        error_code.val = error_code.SUCCESS;
        return true;
    }

    bool IKPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                 const std::vector<double> &joint_angles,
                                 std::vector<geometry_msgs::msg::Pose> &poses) const
    {
        if (link_names.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "FK: No link names provided");
            return false;
        }

        if (joint_angles.size() != joint_names_.size())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "FK: Joint angles size mismatch. Expected: %zu, Got: %zu",
                         joint_names_.size(), joint_angles.size());
            return false;
        }

        try
        {
            poses.resize(link_names.size());

            // 修复：直接使用传入的robot_model引用创建RobotState
            moveit::core::RobotState robot_state(robot_model_);

            robot_state.setJointGroupPositions(group_names_, joint_angles);
            robot_state.update();

            for (size_t i = 0; i < link_names.size(); ++i)
            {
                const std::string &link_name = link_names[i];
                const moveit::core::LinkModel *link_model = robot_state.getRobotModel()->getLinkModel(link_name);
                if (!link_model)
                {
                    RCLCPP_ERROR(node_->get_logger(), "FK: Link '%s' not found", link_name.c_str());
                    return false;
                }

                const Eigen::Isometry3d &transform = robot_state.getGlobalLinkTransform(link_model);
                poses[i].position.x = transform.translation().x();
                poses[i].position.y = transform.translation().y();
                poses[i].position.z = transform.translation().z();

                Eigen::Quaterniond quaternion(transform.rotation());
                quaternion.normalize();
                poses[i].orientation.x = quaternion.x();
                poses[i].orientation.y = quaternion.y();
                poses[i].orientation.z = quaternion.z();
                poses[i].orientation.w = quaternion.w();
            }

            RCLCPP_DEBUG(node_->get_logger(), "FK computed successfully for %zu links", link_names.size());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "FK computation failed: %s", e.what());
            return false;
        }
    }

    bool IKPlugin::supportsGroup(const moveit::core::JointModelGroup *group, std::string *msg) const
    {
        if (group->getActiveVariableCount() != 6)
        {
            if (msg)
                *msg = "This plugin only support 6-DOF group";
            return false;
        }
        return true;
    }

    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, {}, solution, error_code, options);
    }

    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        (void)timeout;
        (void)consistency_limits;
        (void)options;
        // 修复：使用种子状态
        ik_solver_->readCurrentStatus(ik_seed_state);
        solution = ik_solver_->AS(ik_pose);

        if (solution.empty())
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            RCLCPP_WARN(rclcpp::get_logger("IKPlugin"), "无逆解");
            return false;
        }
        else
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
            return true;
        }
    }

    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const kinematics::KinematicsBase::IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, {}, solution, solution_callback, error_code, options);
    }

    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const kinematics::KinematicsBase::IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        bool success = searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, error_code, options);

        if (success && solution_callback)
        {
            solution_callback(ik_pose, solution, error_code);
        }

        return success;
    }

}

PLUGINLIB_EXPORT_CLASS(Inverse_Kinematics_Plugin::IKPlugin, kinematics::KinematicsBase)
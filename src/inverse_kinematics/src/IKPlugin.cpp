#include "inverse_kinematics/Inverse_Kinematics.h"
#include "inverse_kinematics/Analytical_Solution.h"
#include <pluginlib/class_list_macros.hpp>
#include <moveit/robot_state/robot_state.h>
#include "rclcpp/rclcpp.hpp"

namespace Inverse_Kinematics_Plugin
{
    bool IKPlugin::initialize(const rclcpp::Node::SharedPtr &node, // Node 指针
                              const moveit::core::RobotModel &robot_model,
                              const std::string &group_name,              // SRDF 里 <group> 的名字
                              const std::string &base_frame,              // base 坐标系（用不到）
                              const std::vector<std::string> &tip_frames, // 末端链路名字列表
                              double search_discretization)               // 搜索离散精度（用不到）
    {
        node_ = node;
        group_names_ = group_name;
        if (!node_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("IKPlugin"), "传入的 node 指针为空！");
            return false;
        }
        // 根据组名拿到关节模型组指针
        const auto *jmg = robot_model.getJointModelGroup(group_name);
        if (!jmg || jmg->getActiveVariableCount() == 0)
            return false;
        // 保存关节顺序和末端名字，后续 IK/FK 都要按这个顺序算
        joint_names_ = jmg->getActiveJointModelNames();
        link_names_ = tip_frames; // 通常只有 1 个末端

        ik_solver_ = std::make_unique<IKSolution>(robot_model);
        if (!ik_solver_->initIK(joint_names_, robot_model))
        {
            RCLCPP_ERROR(node_->get_logger(), "IK 求解器初始化失败！");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "IKPlugin 初始化成功！关节数：%lu", joint_names_.size());
        return true;
    }

    // 逆运动学核心
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
    // 正运动学核心
    bool IKPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                 const std::vector<double> &joint_angles,
                                 std::vector<geometry_msgs::msg::Pose> &poses) const
    {
        // 输入验证
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
            // 调整输出容器大小
            poses.resize(link_names.size());

            // 使用MoveIt的RobotState计算正运动学
            moveit::core::RobotState robot_state(robot_model_);

            // 设置关节位置
            robot_state.setJointGroupPositions(group_names_, joint_angles);

            // 更新机器人状态（计算变换矩阵）
            robot_state.update();

            // 计算每个请求链路的位姿
            for (size_t i = 0; i < link_names.size(); ++i)
            {
                const std::string &link_name = link_names[i];

                // 获取链路模型
                const moveit::core::LinkModel *link_model = robot_model_->getLinkModel(link_name);
                if (!link_model)
                {
                    RCLCPP_ERROR(node_->get_logger(), "FK: Link '%s' not found", link_name.c_str());
                    return false;
                }

                // 获取从基坐标系到该链路的变换矩阵
                const Eigen::Isometry3d &transform = robot_state.getGlobalLinkTransform(link_model);

                // 转换Eigen变换为geometry_msgs::Pose
                poses[i].position.x = transform.translation().x();
                poses[i].position.y = transform.translation().y();
                poses[i].position.z = transform.translation().z();

                // 提取旋转矩阵并转换为四元数
                Eigen::Quaterniond quaternion(transform.rotation());
                quaternion.normalize(); // 确保四元数归一化

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

    // 告诉 MoveIt 本插件支持哪些关节组（可在这里过滤自由度、关节类型等）
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

    //----------------------------------------------------------------------------------
    // -------------------------- 1 --------------------------
    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        // 传入空的 consistency_limits
        return searchPositionIK(ik_pose, ik_seed_state, timeout, {}, solution, error_code, options);
    }

    // -------------------------- 2--------------------------
    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options) const
    {
        // 1. 调用逆解器计算解（传入限制条件）
        solution = ik_solver_->AS(ik_pose);

        // 2. 设置错误码
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

    // -------------------------- 3 --------------------------
    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const kinematics::KinematicsBase::IKCallbackFn &solution_callback, 
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options ) const
    {
        // 传入空的 consistency_limits
        return searchPositionIK(ik_pose, ik_seed_state, timeout, {}, solution, solution_callback, error_code, options);
    }

    // -------------------------- 4--------------------------
    bool IKPlugin::searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const kinematics::KinematicsBase::IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options ) const
    {
        // 
        bool success = searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, error_code, options);

        // 
        if (success && solution_callback)
        {
            solution_callback(ik_pose, solution, error_code);
        }

        return success;
    }
    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

} // namespace Inverse_Kinematics_Plugin

PLUGINLIB_EXPORT_CLASS(Inverse_Kinematics_Plugin::IKPlugin, kinematics::KinematicsBase)
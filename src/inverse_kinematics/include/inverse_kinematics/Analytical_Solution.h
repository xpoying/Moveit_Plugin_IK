#pragma once
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/robot_model/robot_model.h>
#include <memory>

#ifndef ANALYTICAL_SOLUTION_H
#define ANALYTICAL_SOLUTION_H

namespace Inverse_Kinematics_Plugin
{
    // DH
    struct DHParam
    {
        double a, alpha, d, theta;
    };
    // 逆运动学解算器
    class IKSolution
    {
    public:
        // 构造
        IKSolution(const moveit::core::RobotModelConstPtr &robot_model);
        IKSolution() = delete;
        // 初始化  得到机械臂的DH参数
        bool initIK(std::vector<std::string> &group_names_);
        // 解析解
        std::vector<double> AS(const geometry_msgs::msg::Pose &link_pose);
        // R₀¹·R₁²·R₂³
        inline Eigen::Matrix3d forwardRot123(double theta1, double theta2, double theta3,
                                             const std::vector<DHParam> &dh);
        // 筛选sol
        std::vector<double> selectOptimalSolution(const std::vector<std::vector<double>> &solutions);

        // 有效解
        std::vector<std::vector<double>> filterValidSolutions(
            const std::vector<std::vector<double>> &solutions,
            const std::vector<std::pair<double, double>> &joint_limits);
        // 角度归一化
        double normalizeAngle(double angle);
        // 最优解
        std::vector<double> selectBestFromValidSolutions(
            const std::vector<std::vector<double>> &valid_solutions);
        // 计算关节距离
        double calculateJointDistance(const std::vector<double> &sol1,
                                      const std::vector<double> &sol2);
        // 读取状态
        void readCurrentStatus(const std::vector<double> &ik_seed_state);

    private:
        std::vector<DHParam> dh_;              // 标准DH
        std::vector<std::string> joint_names_; // 关节
        moveit::core::RobotModelConstPtr robot_model_;
        std::vector<double> current_joint_positions_;               // 状态存储
        DHParam extractDHFromTransform(const Eigen::Isometry3d &T); // 从变换矩阵提取DH参数的辅助函数
        bool debug = false;
    };

}

#endif
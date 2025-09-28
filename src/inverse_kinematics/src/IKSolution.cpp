#include "inverse_kinematics/Inverse_Kinematics.h"
#include "inverse_kinematics/Analytical_Solution.h"
#include <cmath>

namespace Inverse_Kinematics_Plugin
{

    IKSolution::IKSolution(const moveit::core::RobotModel &robot_model) : robot_model_(&robot_model) {} // 初始化指针
    bool IKSolution::initIK(std::vector<std::string> &joint_names, const moveit::core::RobotModel &robot_model)
    {
        dh_.clear();
        dh_.reserve(joint_names.size());
        joint_names_ = joint_names;
        for (const auto &jname : joint_names)
        {
            DHParam dh;
            const auto *joint = robot_model.getJointModel(jname);
            // 父连杆到子连杆的齐次坐标变换矩阵
            Eigen::Isometry3d T = joint->getParentLinkModel()->getJointOriginTransform();
            // 平移
            Eigen::Vector3d o = T.translation();
            // 旋转
            Eigen::Matrix3d R = T.rotation();

            dh.a = std::hypot(o.x(), o.y());          // 连杆长度
            dh.alpha = std::atan2(-R(2, 1), R(2, 2)); // 连杆扭角
            dh.d = o.z();                             // 连杆偏距
            dh.theta = std::atan2(R(1, 0), R(0, 0));  // 关节角（绕 z）

            dh_.push_back(dh);
        }
        return !dh_.empty();
    }

    inline Eigen::Matrix3d IKSolution::forwardRot123(double theta1, double theta2, double theta3,
                                                     const std::vector<DHParam> &dh)
    {
        double c1 = std::cos(theta1), s1 = std::sin(theta1);
        double c2 = std::cos(theta2), s2 = std::sin(theta2);
        double c3 = std::cos(theta3), s3 = std::sin(theta3);

        
        double  alpha1 = dh[0].alpha;

        // 标准 DH 连乘：R₀¹·R₁²·R₂³
        Eigen::Matrix3d R01, R12, R23;

        R01 << c1, -s1, 0,
            s1, c1, 0,
            0, 0, 1;

        R12 << c2, -s2, 0,
            s2 * std::cos(alpha1), c2 * std::cos(alpha1), -std::sin(alpha1),
            s2 * std::sin(alpha1), c2 * std::sin(alpha1), std::cos(alpha1);

        R23 << c3, -s3, 0,
            s3, c3, 0,
            0, 0, 1;

        return R01 * R12 * R23;
    }
    std::vector<double> IKSolution::AS(const geometry_msgs::msg::Pose &link_pose)
    {
        std::vector<std::vector<double>> sol; // 解
        const double EPS = 1e-6;

        // wxyz 转 P
        double x = link_pose.position.x;
        double y = link_pose.position.y;
        double z = link_pose.position.z;
        double qx = link_pose.orientation.x;
        double qy = link_pose.orientation.y;
        double qz = link_pose.orientation.z;
        double qw = link_pose.orientation.w;

        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d p(x, y, z);
        Eigen::Isometry3d P_6 = Eigen::Isometry3d::Identity();
        P_6.linear() = R;
        P_6.translation() = p;
        //

        // w求解
        // 工具坐标系计算
        Eigen::Isometry3d T_tool = Eigen::Isometry3d::Identity();
        for (size_t i = 3; i < joint_names_.size(); ++i)
        {
            const auto *joint = robot_model_->getJointModel(joint_names_[i]);
            T_tool = T_tool * joint->getParentLinkModel()->getJointOriginTransform();
        }
        // double D_tool = 0.0;
        // for (size_t i = 4; i < joint_names_.size(); ++i)
        // {
        //     double z_offset = robot_model_.getJointModel(joint_names_[i])->getParentLinkModel()->getJointOriginTransform().translation().z(); // 每个joint1的z方向向量
        //     D_tool += z_offset;
        // }
        // Eigen::Vector3d zb = P_6.linear().col(2); // 由w指向末端的Z方向向量
        Eigen::Vector3d P_W = (P_6 * T_tool.inverse()).translation();

        // r  W在xy平面上的投影
        double r = std::hypot(P_W.x(), P_W.y()) - dh_[0].a;

        // s
        double s = P_W.z() - dh_[0].d;
        // theta_1
        // 1. 肩奇异
        if (std::abs(r) < EPS)
        {
            r = (r >= 0) ? EPS : -EPS;
        }

        double theta_11 = std::atan2(P_W.y(), P_W.x()); // 主解
        double theta_12 = theta_11 + M_PI;              // 翻转解

        // theta_3    cosθ₃ = (r² + (z – d₁)² – a₂² – a₃²) / (2 a₂ a₃)
        std::vector<double> theta_3_sol;
        double c3 = (r * r + s * s - std::pow(dh_[1].a, 2) - std::pow(dh_[2].a, 2)) / (2.0 * dh_[1].a * dh_[2].a);
        double s3 = std::sqrt(1.0 - c3 * c3);

        // double theta_31 = std::atan2(s3, c3);
        // double theta_32 = std::atan2(-s3, c3);
        // 2. 肘极限
        if (std::abs(c3) > 1.0 - EPS)
        {
            theta_3_sol.resize(1);
            theta_3_sol[0] = (c3 > 0) ? 0.0 : M_PI;
        }
        else
        {
            theta_3_sol.push_back(std::atan2(s3, c3));
            theta_3_sol.push_back(std::atan2(-s3, c3));
        }

        // R₃₆ = R₃⁰ ⋅ R₀⁶ = (R₀³)ᵀ ⋅ R₀⁶
        for (auto theta_1 : {theta_11, theta_12})
        {

            for (auto theta_3 : theta_3_sol)
            {
                // theta_2    θ₂ = atan2(z – d₁, r) – atan2(a₃ sinθ₃, a₂ + a₃ cosθ₃)
                double c3_i = std::cos(theta_3);
                double s3_i = std::sin(theta_3);
                double theta_2 = std::atan2(s, r) - std::atan2(dh_[2].a * s3_i, dh_[1].a + dh_[2].a * c3_i);
                Eigen::Matrix3d R36 = forwardRot123(theta_1, theta_2, theta_3, dh_);
                // θ₅ = atan2(√(R₃₆(0,2)² + R₃₆(1,2)²), R₃₆(2,2))
                double c5_abs = std::sqrt(R36(0, 2) * R36(0, 2) + R36(1, 2) * R36(1, 2));
                double theta_51 = std::atan2(c5_abs, R36(2, 2));
                double theta_52 = std::atan2(-c5_abs, R36(2, 2));
                for (auto theta_5 : {theta_51, theta_52})
                {
                    // θ₄ = atan2(R₃₆(1,2)/sinθ₅, R₃₆(0,2)/sinθ₅)
                    // θ₆ = atan2(R₃₆(2,1)/sinθ₅, -R₃₆(2,0)/sinθ₅)
                    double theta_4, theta_6;
                    double s5 = std::sin(theta_5);
                    // 3. 手腕翻转 (gimbal)
                    if (std::abs(s5) < EPS)
                    {                                                // θ₅ ≈ 0° or 180°
                        theta_4 = std::atan2(-R36(1, 0), R36(1, 1)); // 用 c5 行
                        theta_6 = std::atan2(R36(0, 1), R36(0, 0));
                    }
                    else
                    {
                        theta_4 = std::atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                        theta_6 = std::atan2(R36(2, 1) / s5, -R36(2, 0) / s5);
                    }

                    sol.push_back({theta_1, theta_2, theta_3, theta_4, theta_5, theta_6});
                }
            }
        }
        if (!sol.empty())
        {
            return selectOptimalSolution(sol);
        }
        else
        {
            return {};
        }
    }

    // 筛选sol
    std::vector<double> IKSolution::selectOptimalSolution(const std::vector<std::vector<double>> &solutions)
    {
        if (solutions.empty())
            return {};
        if (solutions.size() == 1)
        {
            return solutions[0];
        }

        // 在IKSolution::selectOptimalSolution中替换硬编码
        std::vector<std::pair<double, double>> joint_limits;
        for (const auto &jname : joint_names_)
        {
            const auto *joint = robot_model_->getJointModel(jname);
            const auto &bounds = joint->getVariableBounds();
            joint_limits.emplace_back(bounds[0].min_position_, bounds[0].max_position_);
        }

        // 第一步：筛选有效解
        std::vector<std::vector<double>> valid_solutions = filterValidSolutions(solutions, joint_limits);

        // 第二步：从有效解中选择最优
        if (!valid_solutions.empty())
        {
            return selectBestFromValidSolutions(valid_solutions);
        }
        else
        {

            return {};
        }
    }

    std::vector<std::vector<double>> IKSolution::filterValidSolutions(
        const std::vector<std::vector<double>> &solutions,
        const std::vector<std::pair<double, double>> &joint_limits)
    {
        std::vector<std::vector<double>> valid_solutions;

        for (const auto &sol : solutions)
        {
            if (sol.size() != joint_limits.size())
            {
                continue; // 解的长度不匹配
            }

            bool valid = true;
            for (size_t i = 0; i < sol.size(); ++i)
            {
                double normalized_angle = normalizeAngle(sol[i]);
                if (normalized_angle < joint_limits[i].first || normalized_angle > joint_limits[i].second)
                {
                    valid = false;
                    break;
                }
            }

            if (valid)
            {
                valid_solutions.push_back(sol);
            }
        }

        return valid_solutions;
    }
    double IKSolution::normalizeAngle(double angle)
    {
        // 将角度归一化到 [-π, π] 范围
        angle = std::fmod(angle, 2 * M_PI);
        if (angle > M_PI)
        {
            angle -= 2 * M_PI;
        }
        else if (angle < -M_PI)
        {
            angle += 2 * M_PI;
        }
        return angle;
    }

    std::vector<double> IKSolution::selectBestFromValidSolutions(
        const std::vector<std::vector<double>> &valid_solutions)
    {
        double min_distance = std::numeric_limits<double>::max();
        std::vector<double> best_solution;

        for (const auto &sol : valid_solutions)
        {
            double distance = calculateJointDistance(sol, current_joint_positions_);

            if (distance < min_distance)
            {
                min_distance = distance;
                best_solution = sol;
            }
        }

        return best_solution;
    }

    double IKSolution::calculateJointDistance(const std::vector<double> &sol1,
                                              const std::vector<double> &sol2)
    {
        if (sol1.size() != sol2.size())
        {
            return std::numeric_limits<double>::max();
        }
        double distance = 0.0;
        for (size_t i = 0; i < sol1.size(); ++i)
        {
            double diff = std::abs(normalizeAngle(sol1[i]) - normalizeAngle(sol2[i]));
            // 考虑角度周期性，取最短角度差
            diff = std::min(diff, 2 * M_PI - diff);
            distance += diff * diff; // 欧氏距离平方
        }
        return std::sqrt(distance);
    }

    void IKSolution::readCurrentStatus(const std::vector<double> &ik_seed_state)
    {
        current_joint_positions_ = ik_seed_state;
    }
}
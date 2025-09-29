#include "inverse_kinematics/Inverse_Kinematics.h"
#include "inverse_kinematics/Analytical_Solution.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace Inverse_Kinematics_Plugin
{

    IKSolution::IKSolution(const moveit::core::RobotModelConstPtr &robot_model) : robot_model_(robot_model) // 初始化指针
    {
        current_joint_positions_.resize(6, 0.0);
    }
    bool IKSolution::initIK(std::vector<std::string> &joint_names)
    {
        dh_.clear();
        dh_.reserve(joint_names.size());
        joint_names_ = joint_names;
        if (!robot_model_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("IKSolution"), "RobotModel 未初始化!");
            return false;
        }
        if (debug)
        {
            for (const auto &jname : joint_names)
            {
                const auto *joint = robot_model_->getJointModel(jname);
                if (!joint)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("IKSolution"), "关节 %s 不在机器人模型中", jname.c_str());
                    dh_.clear();
                    return false;
                }

                // 获取关节变换
                Eigen::Isometry3d T = joint->getChildLinkModel()->getJointOriginTransform();
                DHParam dh = extractDHFromTransform(T);
                dh_.push_back(dh);

                RCLCPP_INFO(rclcpp::get_logger("IKSolution"),
                            "关节 %s: a=%.4f, alpha=%.4f, d=%.4f, theta=%.4f",
                            jname.c_str(), dh.a, dh.alpha, dh.d, dh.theta);
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("IKSolution"), "使用UR机器人标准DH参数");

            // UR系列机器人标准DH参数（单位：米，弧度）
            // 注意：这些参数需要根据您的具体UR型号进行调整
            // 以下是UR5的典型DH参数，其他型号请相应调整

            // 关节1: shoulder_pan_joint
            dh_.push_back({0.0, M_PI / 2, 0.089159, 0.0});
            // 关节2: shoulder_lift_joint
            dh_.push_back({-0.425, 0.0, 0.0, 0.0});
            // 关节3: elbow_joint
            dh_.push_back({-0.39225, 0.0, 0.0, 0.0});
            // 关节4: wrist_1_joint
            dh_.push_back({0.0, M_PI / 2, 0.10915, 0.0});
            // 关节5: wrist_2_joint
            dh_.push_back({0.0, -M_PI / 2, 0.09465, 0.0});
            // 关节6: wrist_3_joint
            dh_.push_back({0.0, 0.0, 0.0823, 0.0});
        }
        return !dh_.empty();
    }

    inline Eigen::Matrix3d IKSolution::forwardRot123(double theta1, double theta2, double theta3,
                                                     const std::vector<DHParam> &dh)
    {

        double c1 = std::cos(theta1), s1 = std::sin(theta1);
        double c2 = std::cos(theta2), s2 = std::sin(theta2);
        double c3 = std::cos(theta3), s3 = std::sin(theta3);

        double alpha1 = dh[0].alpha;

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

        RCLCPP_INFO(rclcpp::get_logger("IKSolution"),
                    "========== 逆解计算输入参数 ==========");
        RCLCPP_INFO(rclcpp::get_logger("IKSolution"), "位置: x=%f, y=%f, z=%f", x, y, z);
        RCLCPP_INFO(rclcpp::get_logger("IKSolution"), "四元数: qx=%f, qy=%f, qz=%f, qw=%f", qx, qy, qz, qw);

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
        for (size_t i = 6; i < joint_names_.size(); ++i)
        {
            const auto *joint = robot_model_->getJointModel(joint_names_[i]);
            if (!joint)
            {
                RCLCPP_ERROR(rclcpp::get_logger("IKSolution"), "工具关节 %s 不存在", joint_names_[i].c_str());
                return {};
            }
            T_tool = T_tool * joint->getChildLinkModel()->getJointOriginTransform();
        }
        Eigen::Vector3d P_W = (P_6 * T_tool.inverse()).translation();
        Eigen::Isometry3d TOW = P_6 * T_tool.inverse();

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
                Eigen::Matrix3d R03 = forwardRot123(theta_1, theta_2, theta_3, dh_);
                Eigen::Matrix3d R06 = TOW.linear();
                Eigen::Matrix3d R36 = R03.transpose() * R06;

                // θ₅ = atan2(√(R₃₆(0,2)² + R₃₆(1,2)²), R₃₆(2,2))
                double c5_abs = std::sqrt(R36(0, 2) * R36(0, 2) + R36(1, 2) * R36(1, 2));
                double theta_51 = std::atan2(c5_abs, R36(2, 2));
                double theta_52 = std::atan2(-c5_abs, R36(2, 2));
                // 在theta_5计算中添加奇异性检查
                for (auto theta_5 : {theta_51, theta_52})
                {
                    double theta_4, theta_6;
                    double s5 = std::sin(theta_5);

                    // 改进的奇异性处理
                    if (std::abs(s5) < EPS)
                    {
                        // θ₅ ≈ 0° or 180° 时的万向锁情况
                        // 此时θ₄和θ₆不是独立的，我们固定θ₄，计算θ₆
                        theta_4 = 0.0; // 或者使用当前θ₄值
                        theta_6 = std::atan2(R36(1, 0), R36(0, 0)) - theta_4;
                    }
                    else
                    {
                        theta_4 = std::atan2(R36(1, 2) / s5, R36(0, 2) / s5);
                        theta_6 = std::atan2(R36(2, 1) / s5, -R36(2, 0) / s5);
                    }

                    // 角度归一化到[-π, π]
                    auto normalize_angle = [](double angle)
                    {
                        while (angle > M_PI)
                            angle -= 2 * M_PI;
                        while (angle < -M_PI)
                            angle += 2 * M_PI;
                        return angle;
                    };

                    std::vector<double> solution = {
                        normalize_angle(theta_1),
                        normalize_angle(theta_2),
                        normalize_angle(theta_3),
                        normalize_angle(theta_4),
                        normalize_angle(theta_5),
                        normalize_angle(theta_6)};

                    sol.push_back(solution);
                }
            }
        }
        if (!sol.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("IKSolution"), "========== 所有候选解 ==========");
            RCLCPP_INFO(rclcpp::get_logger("IKSolution"), "共找到 %zu 个候选解:", sol.size());
            return selectOptimalSolution(sol);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("IKSolution"), "========== 无解 ==========");
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
            RCLCPP_INFO(rclcpp::get_logger("IKSolution"), "已选出最优解");
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

    DHParam IKSolution::extractDHFromTransform(const Eigen::Isometry3d &T)
    {
        DHParam dh;

        Eigen::Vector3d translation = T.translation();
        Eigen::Matrix3d rotation = T.linear();

        // 更准确的DH参数提取
        dh.a = translation.x();
        dh.d = translation.z();

        // 提取绕x轴的旋转角度 (alpha)
        // 从旋转矩阵的(2,1)和(2,2)元素提取
        if (std::abs(rotation(2, 2)) > 1e-6)
        {
            dh.alpha = std::atan2(rotation(2, 1), rotation(2, 2));
        }
        else
        {
            // 处理奇异情况
            dh.alpha = (rotation(2, 1) > 0) ? M_PI / 2 : -M_PI / 2;
        }

        // 提取绕z轴的旋转角度 (theta)
        // 从旋转矩阵的(1,0)和(0,0)元素提取
        if (std::abs(rotation(0, 0)) > 1e-6 || std::abs(rotation(1, 0)) > 1e-6)
        {
            dh.theta = std::atan2(rotation(1, 0), rotation(0, 0));
        }
        else
        {
            // 处理奇异情况
            dh.theta = std::atan2(-rotation(0, 1), rotation(1, 1));
        }

        RCLCPP_DEBUG(rclcpp::get_logger("IKSolution"),
                     "提取DH: a=%.4f, alpha=%.4f, d=%.4f, theta=%.4f",
                     dh.a, dh.alpha, dh.d, dh.theta);

        return dh;
    }
}
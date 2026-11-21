#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "extended_kalman_filter.hpp"

namespace auto_aim
{

struct EkfParams {
  double q_xyz_acc_var = 100.0;  // Q 矩阵：未建模加速度（z；xy 由控制量承担）
  double q_yaw_acc_var = 75.0;   // Q 矩阵：角加速度方差
  double q_u_acc_var = 1.0;      // 控制量 u=(ax,ay) 的加速度噪声，经 B 进 Q
  double r_yaw = 4.0e-2;         // R 矩阵：yaw 观测噪声
  double r_pitch = 4.0e-2;       // R 矩阵：pitch 观测噪声
  double r_dist_base = 1.5e-2;   // R 矩阵：距离观测噪声基础值
};

class Target
{
public:
  ArmorName name = ArmorName::one;
  ArmorType armor_type = ArmorType::small;
  ArmorPriority priority = ArmorPriority::first;
  bool jumped = false;
  int last_id = 0;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, EkfParams ekf_params);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void control(const Eigen::Vector3d & acc);

  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;
  bool convergened();

  bool isinit = false;
  bool checkinit();

private:
  int armor_num_ = 4;
  int switch_count_ = 0;
  int update_count_ = 0;

  bool is_switch_ = false;
  bool is_converged_ = false;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;
  EkfParams ekf_params_;
  double last_dt_ = 0.0;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP

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
  double q_xyz_acc_var = 100.0;  // Q 矩阵：位置/速度加速度方差
  double q_yaw_acc_var = 75.0;   // Q 矩阵：角加速度方差
  double r_yaw = 4.0e-2;         // R 矩阵：yaw 观测噪声
  double r_pitch = 4.0e-2;       // R 矩阵：pitch 观测噪声
  double r_dist_base = 1.5e-2;   // R 矩阵：距离观测噪声基础值

  // 三角（正弦）补偿项：把"观测里混入的自车偏置公转"建模成
  //   comp = [a_x·cos(ωt) + b_x·sin(ωt), a_y·cos(ωt) + b_y·sin(ωt)]
  // 直接加在观测模型上，由 EKF 在线估计 4 个系数并从目标状态中扣除。
  // ω 取底盘自转角速度（真实系统来自陀螺）；0 表示关闭补偿。
  double comp_w = 0.0;          // 补偿项角频率 (rad/s)
  double comp_q_var = 1.0;      // 补偿系数随机游走方差 (m^2/s)，给系数留自适应余量
  double comp_p0_var = 1.0e-2;  // 补偿系数初始方差 (m^2) → 初值幅度 0.1 m
};

class Target
{
public:
  // 状态维数：11 维原状态 + 4 维正弦补偿系数 (a_x, b_x, a_y, b_y)
  static constexpr int kBaseStateSize = 11;
  static constexpr int kCompStateSize = 4;
  static constexpr int kStateSize = kBaseStateSize + kCompStateSize;

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

  // 正弦补偿项的相位 ω·t（在 predict(dt) 中按仿真步长累加）
  double comp_phase_ = 0.0;
  // 当前补偿量 [dx, dy]（m），已包含在观测预测里
  Eigen::Vector2d comp_offset(const Eigen::VectorXd & x) const;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP

#include "target.hpp"

#include <numeric>
#include <algorithm>
#include "math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, EkfParams ekf_params)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  armor_num_(armor_num),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  t_(t),
  ekf_params_(ekf_params)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h  |  ax bx ay by
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  // ax/bx: 补偿项 cos/sin 系数（x 轴），ay/by: y 轴
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0,
                     0, 0, 0, 0}};  // 初始化预测量

  // P0：前 11 维用配置值，补偿系数维给初值方差
  Eigen::VectorXd p0_dig(Eigen::VectorXd::Zero(kStateSize));
  p0_dig.head(kBaseStateSize) = P0_dig.head(kBaseStateSize);
  p0_dig.tail(kCompStateSize).setConstant(
    ekf_params_.comp_w > 0.0 ? ekf_params_.comp_p0_var : 0.0);
  Eigen::MatrixXd P0 = p0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  // 初始化滤波器
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(kStateSize, kStateSize);

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵：前 11 维与原版一致（CV + 匀角速度 + 常值 r/l/h），
  // 后 4 维为三角补偿系数，常值模型
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateSize, kStateSize);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;

  // Piecewise White Noise Model
  double v1 = ekf_params_.q_xyz_acc_var;
  double v2 = ekf_params_.q_yaw_acc_var;
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  for (int k = 0; k < 3; k++) {  // x / y / z 三个轴的位置-速度块
    Q(2 * k, 2 * k) = a * v1;
    Q(2 * k, 2 * k + 1) = b * v1;
    Q(2 * k + 1, 2 * k) = b * v1;
    Q(2 * k + 1, 2 * k + 1) = c * v1;
  }
  Q(6, 6) = a * v2;
  Q(6, 7) = b * v2;
  Q(7, 6) = b * v2;
  Q(7, 7) = c * v2;

  // 三角补偿项：相位随 ω·dt 推进；系数按随机游走，允许缓慢自适应
  if (ekf_params_.comp_w > 0.0) {
    comp_phase_ = std::fmod(comp_phase_ + ekf_params_.comp_w * dt, 2.0 * M_PI);
    const double q_comp = ekf_params_.comp_q_var * dt;
    for (int i = kBaseStateSize; i < kStateSize; i++) Q(i, i) = q_comp;
  }

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  // 装甲板匹配
  int id = 0;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个 distance 最小的装甲板匹配
  int match_candidates = std::min(3, armor_num_);
  for (int i = 0; i < match_candidates; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  update_ypda(armor, id);
}

void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);

  Eigen::VectorXd R_dig{
    {ekf_params_.r_yaw,
     ekf_params_.r_pitch,
     std::log(std::abs(delta_angle) + 1.0) + 1.0,
     std::log(std::abs(armor.ypd_in_world[2]) + 1.0) / 200.0 + ekf_params_.r_dist_base}};

  Eigen::MatrixXd R = R_dig.asDiagonal();

  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  // 三角补偿项：叠加在观测预测上（估计出的自车偏置公转）
  const Eigen::Vector2d comp = comp_offset(x);
  armor_x += comp.x();
  armor_y += comp.y();

  return {armor_x, armor_y, armor_z};
}

/// 三角（正弦）补偿项：a·cos(ωt) + b·sin(ωt)，x/y 两轴各一组系数
Eigen::Vector2d Target::comp_offset(const Eigen::VectorXd & x) const
{
  if (ekf_params_.comp_w <= 0.0 || x.size() < kStateSize) return Eigen::Vector2d::Zero();

  const double c = std::cos(comp_phase_);
  const double s = std::sin(comp_phase_);
  return {x[11] * c + x[12] * s, x[13] * c + x[14] * s};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;
  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // 位置对状态的雅可比 (4 x 15)：前 11 维与原版一致，后 4 维为三角补偿系数
  Eigen::MatrixXd H_armor_xyza = Eigen::MatrixXd::Zero(4, kStateSize);
  H_armor_xyza(0, 0) = 1.0;
  H_armor_xyza(1, 2) = 1.0;
  H_armor_xyza(2, 4) = 1.0;
  H_armor_xyza(0, 6) = dx_da;
  H_armor_xyza(1, 6) = dy_da;
  H_armor_xyza(0, 8) = dx_dr;
  H_armor_xyza(1, 8) = dy_dr;
  H_armor_xyza(0, 9) = dx_dl;
  H_armor_xyza(1, 9) = dy_dl;
  H_armor_xyza(2, 10) = dz_dh;
  H_armor_xyza(3, 6) = 1.0;
  // d(x,y)/d(ax,bx,ay,by) = cos(ωt) / sin(ωt)
  if (ekf_params_.comp_w > 0.0) {
    const double cc = std::cos(comp_phase_);
    const double ss = std::sin(comp_phase_);
    H_armor_xyza(0, 11) = cc;
    H_armor_xyza(0, 12) = ss;
    H_armor_xyza(1, 13) = cc;
    H_armor_xyza(1, 14) = ss;
  }

  Eigen::Vector3d armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);

  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim

#include "target.hpp"

#include <cmath>

#include "math_tools.hpp"

namespace tracker
{

Target::Target(double radius, int armor_num, Eigen::VectorXd P0_dig, EkfParams ekf_params)
: armor_num_(armor_num > 0 ? armor_num : 4),
  radius_(radius),
  p0_dig_(std::move(P0_dig)),
  ekf_params_(ekf_params)
{
  if (p0_dig_.size() != 11) {
    p0_dig_.resize(11);
    p0_dig_ << 1, 64, 1, 64, 1, 64, 0.4, 400, 1.0e-5, 1, 1;
  }
}

int Target::armor_id(const common::ArmorObservation & obs) const
{
  int id = obs.armor_id;
  if (id < 0 || id >= armor_num_) return 0;
  return id;
}

void Target::init(const common::ArmorObservation & obs)
{
  const Eigen::Vector3d & xyz = obs.xyz_in_gimbal;
  const double yaw_raw = obs.yaw_raw;
  const double r = radius_;

  const double center_x = xyz[0] + r * std::cos(yaw_raw);
  const double center_y = xyz[1] + r * std::sin(yaw_raw);
  const double center_z = xyz[2];

  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, yaw_raw, 0, r, 0, 0}};
  Eigen::MatrixXd P0 = p0_dig_.asDiagonal();

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  initialized_ = true;
}

void Target::predict(double dt)
{
  if (!initialized_) return;
  last_dt_ = dt;

  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  ekf_.predict(F, f);
}

void Target::control(const Eigen::Vector3d & acc)
{
  if (!initialized_) return;
  const double dt = last_dt_;
  const double dt2 = 0.5 * dt * dt;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(11, 2);
  B(0, 0) = dt2;
  B(1, 0) = dt;
  B(2, 1) = dt2;
  B(3, 1) = dt;
  const Eigen::Vector2d u(acc.x(), acc.y());
  Eigen::Matrix2d M = Eigen::Matrix2d::Zero();
  M(0, 0) = ekf_params_.m_ax;
  M(1, 1) = ekf_params_.m_ay;
  ekf_.control(B, u, M);
}

void Target::update(const common::ArmorObservation & obs)
{
  if (!initialized_) {
    init(obs);
    return;
  }
  update_ypda(obs, armor_id(obs));
}

void Target::update_ypda(const common::ArmorObservation & obs, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

  Eigen::VectorXd R_dig{
    {ekf_params_.r_yaw, ekf_params_.r_pitch, ekf_params_.r_dist_base, ekf_params_.r_yaw}};
  Eigen::MatrixXd R = R_dig.asDiagonal();

  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * M_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::Vector3d & ypd = obs.ypd_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], obs.yaw_raw}};

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
  const double dt = last_dt_;
  if (dt > 0.0) {
    const double v1 = ekf_params_.q_xyz_acc_var;
    const double v2 = ekf_params_.q_yaw_acc_var;
    const double a = dt * dt * dt * dt / 4.0;
    const double b = dt * dt * dt / 2.0;
    const double c = dt * dt;
    Q(4, 4) = a * v1;
    Q(4, 5) = b * v1;
    Q(5, 4) = b * v1;
    Q(5, 5) = c * v1;
    Q(6, 6) = a * v2;
    Q(6, 7) = b * v2;
    Q(7, 6) = b * v2;
    Q(7, 7) = c * v2;
  }

  ekf_.update(z, H, R, Q, h, z_subtract);
}

common::TargetEstimate Target::estimate() const
{
  common::TargetEstimate est;
  if (!initialized_) return est;

  const Eigen::VectorXd & x = ekf_.x;
  est.x = x;
  if (x.size() >= 11) {
    est.center = Eigen::Vector3d(x[0], x[2], x[4]);
    est.velocity = Eigen::Vector3d(x[1], x[3], x[5]);
    est.yaw = x[6];
    est.w = x[7];
    est.radius = x[8];
    est.l = x[9];
    est.h = x[10];
    est.jumped = true;
    est.armor_num = armor_num_;
    est.last_nis = ekf_.last_nis;

    const bool r_ok = x[8] > 0.05 && x[8] < 0.5;
    const bool l_ok = x[8] + x[9] > 0.05 && x[8] + x[9] < 0.5;
    est.diverged = !(r_ok && l_ok);

    est.armor_xyza_list.reserve(static_cast<size_t>(armor_num_));
    for (int i = 0; i < armor_num_; i++) {
      const double angle = tools::limit_rad(x[6] + i * 2 * M_PI / armor_num_);
      const Eigen::Vector3d xyz = h_armor_xyz(x, i);
      est.armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
  }
  return est;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * M_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * M_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;
  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

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

}  // namespace tracker

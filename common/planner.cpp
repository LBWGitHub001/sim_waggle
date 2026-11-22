#include "planner.hpp"

#include "frames.hpp"
#include "trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace common
{
namespace
{
void free_solver(TinySolver * solver)
{
  if (!solver) return;
  delete solver->solution;
  delete solver->cache;
  delete solver->settings;
  delete solver->work;
  delete solver;
}
}  // namespace

Planner::Planner(AimerParams aimer, PlannerParams planner)
: aimer_(aimer),
  params_(planner)
{
  setup_solvers();
}

Planner::~Planner()
{
  free_solver(yaw_solver_);
  free_solver(pitch_solver_);
}

void Planner::setup_solvers()
{
  Eigen::MatrixXd A(2, 2);
  A << 1.0, kPlannerDt, 0.0, 1.0;
  Eigen::MatrixXd B(2, 1);
  B << 0.0, kPlannerDt;
  Eigen::VectorXd f = Eigen::VectorXd::Zero(2);

  Eigen::Matrix<double, 2, 1> Qy = params_.Q_yaw;
  Eigen::Matrix<double, 1, 1> Ry;
  Ry << params_.R_yaw;
  tiny_setup(
    &yaw_solver_, A, B, f, Qy.asDiagonal(), Ry.asDiagonal(), 1.0, 2, 1, kPlannerHorizon, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, kPlannerHorizon, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, kPlannerHorizon, 1e17);
  Eigen::MatrixXd u_min_y =
    Eigen::MatrixXd::Constant(1, kPlannerHorizon - 1, -params_.max_yaw_acc);
  Eigen::MatrixXd u_max_y =
    Eigen::MatrixXd::Constant(1, kPlannerHorizon - 1, params_.max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min_y, u_max_y);
  yaw_solver_->settings->max_iter = 10;

  Eigen::Matrix<double, 2, 1> Qp = params_.Q_pitch;
  Eigen::Matrix<double, 1, 1> Rp;
  Rp << params_.R_pitch;
  tiny_setup(
    &pitch_solver_, A, B, f, Qp.asDiagonal(), Rp.asDiagonal(), 1.0, 2, 1, kPlannerHorizon, 0);

  Eigen::MatrixXd u_min_p =
    Eigen::MatrixXd::Constant(1, kPlannerHorizon - 1, -params_.max_pitch_acc);
  Eigen::MatrixXd u_max_p =
    Eigen::MatrixXd::Constant(1, kPlannerHorizon - 1, params_.max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min_p, u_max_p);
  pitch_solver_->settings->max_iter = 10;
}

Planner::Horizon Planner::get_trajectory(TargetEstimate est, double yaw0, double bullet_speed)
{
  Horizon traj = Horizon::Zero();

  est = propagate_estimate(est, -kPlannerDt * (kPlannerHalfHorizon + 1));
  Eigen::Vector2d yp_last;
  if (!aimer_.yaw_pitch(est, bullet_speed, yp_last).valid) {
    throw std::runtime_error("unsolvable shooting yaw/pitch");
  }

  est = propagate_estimate(est, kPlannerDt);
  Eigen::Vector2d yp;
  if (!aimer_.yaw_pitch(est, bullet_speed, yp).valid) {
    throw std::runtime_error("unsolvable shooting yaw/pitch");
  }

  for (int i = 0; i < kPlannerHorizon; i++) {
    est = propagate_estimate(est, kPlannerDt);
    Eigen::Vector2d yp_next;
    if (!aimer_.yaw_pitch(est, bullet_speed, yp_next).valid) {
      throw std::runtime_error("unsolvable shooting yaw/pitch");
    }
    const double yaw_vel = wrap_pi(yp_next[0] - yp_last[0]) / (2.0 * kPlannerDt);
    const double pitch_vel = (yp_next[1] - yp_last[1]) / (2.0 * kPlannerDt);
    traj.col(i) << wrap_pi(yp[0] - yaw0), yaw_vel, yp[1], pitch_vel;
    yp_last = yp;
    yp = yp_next;
  }
  return traj;
}

GimbalCommand Planner::plan(const TargetEstimate & est_in, double bullet_speed)
{
  GimbalCommand cmd;
  if (est_in.x.size() < 11) return cmd;

  if (bullet_speed < 10.0 || bullet_speed > 25.0) bullet_speed = 22.0;

  TargetEstimate est = est_in;
  Eigen::Vector2d yp0;
  AimPoint pt0 = aimer_.yaw_pitch(est, bullet_speed, yp0);
  if (!pt0.valid) return cmd;

  const Eigen::Vector3d xyz0 = pt0.xyza.head<3>();
  const BulletTrajectory fly(
    bullet_speed, std::hypot(xyz0.x(), xyz0.y()), xyz0.z());
  if (!fly.unsolvable) est = propagate_estimate(est, fly.fly_time);
  cmd.fly_time = fly.unsolvable ? 0.0 : fly.fly_time;

  Eigen::Vector2d yp_now;
  AimPoint pt_now = aimer_.yaw_pitch(est, bullet_speed, yp_now);
  if (!pt_now.valid) return cmd;
  const double yaw0 = yp_now[0];

  Horizon traj;
  try {
    traj = get_trajectory(est, yaw0, bullet_speed);
  } catch (const std::exception &) {
    return cmd;
  }

  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, kPlannerHorizon);
  tiny_solve(yaw_solver_);

  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, kPlannerHorizon);
  tiny_solve(pitch_solver_);

  cmd.valid = true;
  cmd.aim_xyz = pt_now.xyza.head<3>();
  cmd.target_yaw = wrap_pi(traj(0, kPlannerHalfHorizon) + yaw0);
  cmd.target_pitch = traj(2, kPlannerHalfHorizon);
  cmd.yaw = wrap_pi(yaw_solver_->work->x(0, kPlannerHalfHorizon) + yaw0);
  cmd.yaw_vel = yaw_solver_->work->x(1, kPlannerHalfHorizon);
  cmd.yaw_acc = yaw_solver_->work->u(0, kPlannerHalfHorizon);
  cmd.pitch = pitch_solver_->work->x(0, kPlannerHalfHorizon);
  cmd.pitch = -cmd.pitch;
  cmd.pitch_vel = pitch_solver_->work->x(1, kPlannerHalfHorizon);
  cmd.pitch_acc = pitch_solver_->work->u(0, kPlannerHalfHorizon);
  return cmd;
}

}  // namespace common

#ifndef COMMON__PLANNER_HPP
#define COMMON__PLANNER_HPP

#include "aim.hpp"
#include "types.hpp"

#include "tinympc/tiny_api.hpp"

#include <Eigen/Dense>

namespace common
{

constexpr double kPlannerDt = 0.01;
constexpr int kPlannerHalfHorizon = 50;
constexpr int kPlannerHorizon = kPlannerHalfHorizon * 2;

/// TinyMPC 开环轨迹平滑：输入是射击角时域，不含云台实测。无开火判定。
class Planner
{
public:
  Planner(AimerParams aimer, PlannerParams planner);
  ~Planner();

  Planner(const Planner &) = delete;
  Planner & operator=(const Planner &) = delete;

  GimbalCommand plan(const TargetEstimate & est, double bullet_speed);

private:
  using Horizon = Eigen::Matrix<double, 4, kPlannerHorizon>;

  void setup_solvers();
  Horizon get_trajectory(TargetEstimate est, double yaw0, double bullet_speed);

  Aimer aimer_;
  PlannerParams params_;
  TinySolver * yaw_solver_ = nullptr;
  TinySolver * pitch_solver_ = nullptr;
};

}  // namespace common

#endif

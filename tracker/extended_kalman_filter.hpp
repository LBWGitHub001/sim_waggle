#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>
#include <string>

namespace tools
{
class ExtendedKalmanFilter
{
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  /// x = Fx, P = F P Fᵀ. Optional f wraps yaw etc.
  Eigen::VectorXd predict(const Eigen::MatrixXd & F);
  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  /// x += B u, P += B M Bᵀ. M is control-noise covariance.
  Eigen::VectorXd control(const Eigen::MatrixXd & B, const Eigen::VectorXd & u,
                          const Eigen::MatrixXd & M);

  /// P += Q, then Joseph measurement update.
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & Q, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  std::map<std::string, double> data;  // 卡方检验数据
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 10000;
  double last_nis = 0.0;

private:
  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  int nees_count_ = 0;
  int nis_count_ = 0;
  int total_count_ = 0;
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP

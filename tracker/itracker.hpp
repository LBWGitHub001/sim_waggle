#ifndef TRACKER__ITRACKER_HPP
#define TRACKER__ITRACKER_HPP

#include <Eigen/Dense>
#include <string>
#include "types.hpp"

namespace tracker
{

/// 自瞄后端目标追踪器抽象接口 (ITracker)
/// 规范化滤波追踪器的输入、预测、更新与状态导出，
/// 为后续即插即用接入偏置补偿卡尔曼算法提供干净的多态基类。
class ITracker
{
public:
  virtual ~ITracker() = default;

  // 使用初始装甲板观测初始化跟踪器
  virtual void init(const common::ArmorObservation & init_obs, double t) = 0;

  // 状态预测推进 dt 秒
  virtual void predict(double dt) = 0;

  // 使用当帧装甲板观测更新滤波器
  virtual void update(const common::ArmorObservation & obs) = 0;

  // 获取当前滤波估计状态
  virtual common::TargetEstimate estimate() const = 0;

  // 预测弹丸飞行时间 flight_time 后的目标瞄准落点 (m)
  virtual Eigen::Vector3d predict_aim_point(double flight_time) const = 0;

  // 检查滤波器是否发散
  virtual bool diverged() const = 0;

  // 获取最近一次更新的归一化新息平方 (NIS)
  virtual double last_nis() const = 0;

  // 获取追踪器名称
  virtual std::string name() const = 0;
};

}  // namespace tracker

#endif  // TRACKER__ITRACKER_HPP

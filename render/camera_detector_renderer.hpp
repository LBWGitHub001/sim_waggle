#ifndef RENDER__CAMERA_DETECTOR_RENDERER_HPP
#define RENDER__CAMERA_DETECTOR_RENDERER_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

#include "common/types.hpp"
#include "common/metrics.hpp"

namespace render
{

/// 车载云台相机目标检测与自瞄跟踪图像渲染器
/// 真实呈现机器人在偏置公转扰动下的感知视界：
/// - 针孔透视模型与动态视差地面网格
/// - 敌方物理装甲板法向判定与发光 LED 灯条 Bloom 效果
/// - 视觉检测器 (YOLO/传统视觉) 4 关键点、绿色 Bounding Box、分类置信度标签
/// - 卡尔曼追踪器 (EKF) 状态重投影框 [EKF] 与跟踪残差线
/// - 弹道前馈超前瞄准点 (AIM LEAD) 动态偏摆圆环 (暴露伪速度造成的脱靶量)
/// - 战术火控分划十字准星与 OSD 状态看板
class CameraDetectorRenderer
{
public:
  CameraDetectorRenderer(int width = 800, int height = 600, double fov_deg = 60.0);

  cv::Mat render(
    const common::GroundTruth & gt,
    const common::TargetEstimate & est,
    const common::ArmorObservation & obs,
    const common::MetricsResult & metrics);

private:
  int w_;
  int h_;
  double fov_deg_;
  double fx_;
  double fy_;
  cv::Point2f cx_cy_;
  double cam_height_; // 相机基准高度 (m, 默认 0.35m)

  bool project_world_to_cam(
    const Eigen::Vector3d & p_w,
    const Eigen::Vector3d & cam_pos,
    double gimbal_yaw,
    double gimbal_pitch,
    cv::Point2f & out_pixel,
    double & out_zc) const;

  void draw_ground_grid(
    cv::Mat & img, const Eigen::Vector3d & cam_pos,
    double gimbal_yaw, double gimbal_pitch) const;

  void draw_reticle(cv::Mat & img) const;

  void draw_hud(
    cv::Mat & img,
    const common::GroundTruth & gt,
    const common::TargetEstimate & est,
    const common::MetricsResult & metrics) const;
};

}  // namespace render

#endif  // RENDER__CAMERA_DETECTOR_RENDERER_HPP

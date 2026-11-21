#include "camera_detector_renderer.hpp"

#include <fmt/format.h>
#include <cmath>
#include <algorithm>

#include "common/frames.hpp"

namespace render
{

CameraDetectorRenderer::CameraDetectorRenderer(int width, int height, double fov_deg)
: w_(width),
  h_(height),
  fov_deg_(fov_deg),
  cx_cy_(width * 0.5f, height * 0.5f),
  cam_height_(0.35)
{
  double fov_rad = fov_deg_ * CV_PI / 180.0;
  fx_ = (w_ * 0.5) / std::tan(fov_rad * 0.5);
  fy_ = fx_;
}

bool CameraDetectorRenderer::project_world_to_cam(
  const Eigen::Vector3d & p_w,
  const Eigen::Vector3d & cam_pos,
  double gimbal_yaw,
  double gimbal_pitch,
  cv::Point2f & out_pixel,
  double & out_zc) const
{
  Eigen::Vector3d delta = p_w - cam_pos;

  double cy = std::cos(gimbal_yaw);
  double sy = std::sin(gimbal_yaw);
  double cp = std::cos(gimbal_pitch);
  double sp = std::sin(gimbal_pitch);

  // 相机坐标系 (+Zc 前向光轴, +Xc 右向, +Yc 下向)
  Eigen::Vector3d f_w(cy * cp, sy * cp, sp);
  Eigen::Vector3d r_w(sy, -cy, 0.0);
  Eigen::Vector3d d_w(sp * cy, sp * sy, -cp);

  double xc = delta.dot(r_w);
  double yc = delta.dot(d_w);
  double zc = delta.dot(f_w);

  out_zc = zc;
  if (zc <= 0.1) {
    return false;
  }

  out_pixel.x = static_cast<float>(fx_ * (xc / zc) + cx_cy_.x);
  out_pixel.y = static_cast<float>(fy_ * (yc / zc) + cx_cy_.y);
  return true;
}

void CameraDetectorRenderer::draw_ground_grid(
  cv::Mat & img, const Eigen::Vector3d & cam_pos,
  double gimbal_yaw, double gimbal_pitch) const
{
  cv::Scalar grid_col(34, 38, 48);

  // 纵向透视网格线
  for (double y = -4.0; y <= 4.0; y += 1.0) {
    cv::Point2f prev_pt;
    bool has_prev = false;
    for (double x = 0.5; x <= 8.0; x += 0.25) {
      cv::Point2f pt;
      double zc;
      if (project_world_to_cam(Eigen::Vector3d(x, y, 0.0), cam_pos, gimbal_yaw, gimbal_pitch, pt, zc)) {
        if (has_prev && pt.x >= -100 && pt.x <= w_ + 100 && pt.y >= -100 && pt.y <= h_ + 100) {
          cv::line(img, prev_pt, pt, grid_col, 1, cv::LINE_AA);
        }
        prev_pt = pt;
        has_prev = true;
      } else {
        has_prev = false;
      }
    }
  }

  // 横向透视网格线
  for (double x = 1.0; x <= 8.0; x += 1.0) {
    cv::Point2f prev_pt;
    bool has_prev = false;
    for (double y = -4.0; y <= 4.0; y += 0.25) {
      cv::Point2f pt;
      double zc;
      if (project_world_to_cam(Eigen::Vector3d(x, y, 0.0), cam_pos, gimbal_yaw, gimbal_pitch, pt, zc)) {
        if (has_prev && pt.x >= -100 && pt.x <= w_ + 100 && pt.y >= -100 && pt.y <= h_ + 100) {
          cv::line(img, prev_pt, pt, grid_col, 1, cv::LINE_AA);
        }
        prev_pt = pt;
        has_prev = true;
      } else {
        has_prev = false;
      }
    }
  }
}

void CameraDetectorRenderer::draw_reticle(cv::Mat & img) const
{
  cv::Scalar reticle_col(160, 175, 120);
  cv::Scalar dim_col(90, 105, 75);

  cv::circle(img, cx_cy_, 18, reticle_col, 1, cv::LINE_AA);
  cv::circle(img, cx_cy_, 2, reticle_col, -1, cv::LINE_AA);

  // 水平分划
  cv::line(img, cv::Point(w_ / 2 - 180, h_ / 2), cv::Point(w_ / 2 - 25, h_ / 2), reticle_col, 1, cv::LINE_AA);
  cv::line(img, cv::Point(w_ / 2 + 25, h_ / 2), cv::Point(w_ / 2 + 180, h_ / 2), reticle_col, 1, cv::LINE_AA);

  for (int dx = 40; dx <= 160; dx += 20) {
    int tick_h = (dx % 40 == 0) ? 6 : 3;
    cv::line(img, cv::Point(w_ / 2 - dx, h_ / 2 - tick_h), cv::Point(w_ / 2 - dx, h_ / 2 + tick_h), dim_col, 1);
    cv::line(img, cv::Point(w_ / 2 + dx, h_ / 2 - tick_h), cv::Point(w_ / 2 + dx, h_ / 2 + tick_h), dim_col, 1);
  }

  // 垂直分划
  cv::line(img, cv::Point(w_ / 2, h_ / 2 - 120), cv::Point(w_ / 2, h_ / 2 - 25), reticle_col, 1, cv::LINE_AA);
  cv::line(img, cv::Point(w_ / 2, h_ / 2 + 25), cv::Point(w_ / 2, h_ / 2 + 120), reticle_col, 1, cv::LINE_AA);

  for (int dy = 40; dy <= 100; dy += 20) {
    int tick_w = (dy % 40 == 0) ? 6 : 3;
    cv::line(img, cv::Point(w_ / 2 - tick_w, h_ / 2 - dy), cv::Point(w_ / 2 + tick_w, h_ / 2 - dy), dim_col, 1);
    cv::line(img, cv::Point(w_ / 2 - tick_w, h_ / 2 + dy), cv::Point(w_ / 2 + tick_w, h_ / 2 + dy), dim_col, 1);
  }
}

void CameraDetectorRenderer::draw_hud(
  cv::Mat & img,
  const common::GroundTruth & gt,
  const common::TargetEstimate & est,
  const common::MetricsResult & metrics) const
{
  // 1. 顶部半透明背景条
  cv::Rect top_bar(0, 0, w_, 64);
  cv::Mat top_roi = img(top_bar);
  cv::Mat top_bg(top_bar.size(), CV_8UC3, cv::Scalar(12, 14, 20));
  cv::addWeighted(top_bg, 0.78, top_roi, 0.22, 0.0, top_roi);
  cv::line(img, cv::Point(0, 64), cv::Point(w_, 64), cv::Scalar(40, 48, 60), 1);

  // 顶部左侧
  cv::circle(img, cv::Point(18, 22), 5, cv::Scalar(0, 255, 128), -1, cv::LINE_AA);
  cv::putText(img, "AUTO-AIM CAM DETECTOR", cv::Point(30, 26), cv::FONT_HERSHEY_DUPLEX, 0.52, cv::Scalar(240, 245, 250), 1, cv::LINE_AA);
  cv::putText(img, "TARGET: B1 (ENEMY) | STATUS: TRACKING (50Hz)", cv::Point(18, 48), cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(170, 185, 200), 1, cv::LINE_AA);

  // 顶部右侧
  const double ghost_vel = common::vel_odom_to_gimbal(gt.target.vel, gt.gimbal).head<2>().norm();
  std::string spin_str = fmt::format("CHASSIS W: {:4.1f} rad/s", gt.chassis.w);
  std::string offset_str = fmt::format("OFFSET: {:4.1f} cm", (gt.gimbal.pos - gt.chassis.pos).head<2>().norm() * 100.0);
  std::string ghost_str = fmt::format("GHOST VEL: {:4.2f} m/s", ghost_vel);
  std::string bias_str = fmt::format("LEAD BIAS: {:4.1f} cm", metrics.aim_lead_bias * 100.0);

  cv::Scalar ghost_col = (ghost_vel > 0.3) ? cv::Scalar(0, 140, 255) : cv::Scalar(0, 255, 128);
  cv::Scalar bias_col = (metrics.aim_lead_bias > 0.08) ? cv::Scalar(40, 40, 255) : cv::Scalar(0, 255, 128);

  cv::putText(img, spin_str, cv::Point(w_ - 320, 24), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(200, 215, 230), 1, cv::LINE_AA);
  cv::putText(img, offset_str, cv::Point(w_ - 150, 24), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(200, 215, 230), 1, cv::LINE_AA);
  cv::putText(img, ghost_str, cv::Point(w_ - 320, 48), cv::FONT_HERSHEY_DUPLEX, 0.44, ghost_col, 1, cv::LINE_AA);
  cv::putText(img, bias_str, cv::Point(w_ - 150, 48), cv::FONT_HERSHEY_DUPLEX, 0.44, bias_col, 1, cv::LINE_AA);

  // 2. 底部半透明状态条与图例
  cv::Rect bot_bar(0, h_ - 30, w_, 30);
  cv::Mat bot_roi = img(bot_bar);
  cv::Mat bot_bg(bot_bar.size(), CV_8UC3, cv::Scalar(12, 14, 20));
  cv::addWeighted(bot_bg, 0.78, bot_roi, 0.22, 0.0, bot_roi);
  cv::line(img, cv::Point(0, h_ - 30), cv::Point(w_, h_ - 30), cv::Scalar(40, 48, 60), 1);

  cv::putText(img, "CAM: 800x600 FOV:60deg | SIM: OFFSET DISTURBANCE", cv::Point(14, h_ - 10), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(150, 165, 180), 1, cv::LINE_AA);

  int lx = w_ - 410;
  int ly = h_ - 10;
  cv::rectangle(img, cv::Rect(lx, ly - 8, 10, 10), cv::Scalar(0, 255, 128), -1);
  cv::putText(img, "Detected", cv::Point(lx + 14, ly), cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(200, 210, 220), 1, cv::LINE_AA);

  lx += 85;
  cv::rectangle(img, cv::Rect(lx, ly - 8, 10, 10), cv::Scalar(255, 0, 255), -1);
  cv::putText(img, "EKF State", cv::Point(lx + 14, ly), cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(200, 210, 220), 1, cv::LINE_AA);

  lx += 90;
  cv::circle(img, cv::Point(lx + 5, ly - 3), 5, cv::Scalar(0, 215, 255), 2);
  cv::putText(img, "Ballistic Lead", cv::Point(lx + 14, ly), cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(200, 210, 220), 1, cv::LINE_AA);
}

cv::Mat CameraDetectorRenderer::render(
  const common::GroundTruth & gt,
  const common::TargetEstimate & est,
  const common::ArmorObservation & obs,
  const common::MetricsResult & metrics)
{
  cv::Mat canvas(h_, w_, CV_8UC3, cv::Scalar(18, 20, 26));

  // 工作系 = gimbal：相机在原点、朝向 +X；敌车用 odom→gimbal 后的真值。
  const common::TargetState enemy = common::express_target_in_gimbal(gt);
  Eigen::Vector3d cam_pos(0.0, 0.0, cam_height_);
  double gimbal_yaw = 0.0;
  double gimbal_pitch = 0.0;

  // 1. 地面透视网格
  draw_ground_grid(canvas, cam_pos, gimbal_yaw, gimbal_pitch);

  // 2. 敌车底盘多边形轮廓
  Eigen::Vector3d target_center_3d(enemy.center.x(), enemy.center.y(), cam_height_);
  std::vector<cv::Point> chassis_pts;
  for (int deg = 0; deg < 360; deg += 30) {
    double rad = deg * CV_PI / 180.0;
    Eigen::Vector3d p_base = target_center_3d + Eigen::Vector3d(
      0.23 * std::cos(rad), 0.23 * std::sin(rad), -0.15
    );
    cv::Point2f px;
    double zc;
    if (project_world_to_cam(p_base, cam_pos, gimbal_yaw, gimbal_pitch, px, zc)) {
      chassis_pts.push_back(cv::Point(std::round(px.x), std::round(px.y)));
    }
  }
  if (chassis_pts.size() >= 6) {
    std::vector<std::vector<cv::Point>> polys{chassis_pts};
    cv::fillPoly(canvas, polys, cv::Scalar(24, 25, 30), cv::LINE_AA);
    cv::polylines(canvas, polys, true, cv::Scalar(45, 48, 55), 1, cv::LINE_AA);
  }

  // 3. 渲染 4 块真实装甲板与发光 LED
  const double width = obs.width;
  const double height = obs.height;
  const double hw = width * 0.5;
  const double hh = height * 0.5;
  const double pitch_armor = 15.0 * CV_PI / 180.0;
  const double cp = std::cos(pitch_armor);
  const double sp = std::sin(pitch_armor);

  cv::Point2f detected_center_px(0.0f, 0.0f);
  bool has_detected = false;

  for (size_t i = 0; i < enemy.armors_pos.size(); i++) {
    double yaw_i = enemy.armors_yaw[i];
    Eigen::Vector3d p_armor(enemy.armors_pos[i].x(), enemy.armors_pos[i].y(), cam_height_);

    Eigen::Vector3d n_a(-std::cos(yaw_i) * cp, -std::sin(yaw_i) * cp, sp);
    Eigen::Vector3d u_r(-std::sin(yaw_i), std::cos(yaw_i), 0.0);
    Eigen::Vector3d v_u(std::cos(yaw_i) * sp, std::sin(yaw_i) * sp, cp);

    Eigen::Vector3d v_ac = cam_pos - p_armor;
    if (n_a.dot(v_ac) <= 0.01) {
      continue; // 背对相机
    }

    Eigen::Vector3d p_tl = p_armor - hw * u_r + hh * v_u;
    Eigen::Vector3d p_tr = p_armor + hw * u_r + hh * v_u;
    Eigen::Vector3d p_br = p_armor + hw * u_r - hh * v_u;
    Eigen::Vector3d p_bl = p_armor - hw * u_r - hh * v_u;

    cv::Point2f px_tl, px_tr, px_br, px_bl;
    double zc_tl, zc_tr, zc_br, zc_bl;

    bool ok = project_world_to_cam(p_tl, cam_pos, gimbal_yaw, gimbal_pitch, px_tl, zc_tl) &&
              project_world_to_cam(p_tr, cam_pos, gimbal_yaw, gimbal_pitch, px_tr, zc_tr) &&
              project_world_to_cam(p_br, cam_pos, gimbal_yaw, gimbal_pitch, px_br, zc_br) &&
              project_world_to_cam(p_bl, cam_pos, gimbal_yaw, gimbal_pitch, px_bl, zc_bl);

    if (!ok) continue;

    // 装甲板基板
    std::vector<cv::Point> armor_poly{
      cv::Point(std::round(px_tl.x), std::round(px_tl.y)),
      cv::Point(std::round(px_tr.x), std::round(px_tr.y)),
      cv::Point(std::round(px_br.x), std::round(px_br.y)),
      cv::Point(std::round(px_bl.x), std::round(px_bl.y))
    };
    std::vector<std::vector<cv::Point>> armor_polys{armor_poly};
    cv::fillPoly(canvas, armor_polys, cv::Scalar(16, 16, 20), cv::LINE_AA);
    cv::polylines(canvas, armor_polys, true, cv::Scalar(40, 42, 50), 1, cv::LINE_AA);

    // 发光 LED 灯条 (高亮蓝)
    cv::line(canvas, px_bl, px_tl, cv::Scalar(240, 80, 20), 5, cv::LINE_AA);
    cv::line(canvas, px_bl, px_tl, cv::Scalar(255, 170, 80), 2, cv::LINE_AA);
    cv::line(canvas, px_bl, px_tl, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    cv::line(canvas, px_br, px_tr, cv::Scalar(240, 80, 20), 5, cv::LINE_AA);
    cv::line(canvas, px_br, px_tr, cv::Scalar(255, 170, 80), 2, cv::LINE_AA);
    cv::line(canvas, px_br, px_tr, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // 数字 1
    cv::Point2f center_px = (px_tl + px_tr + px_br + px_bl) * 0.25f;
    cv::putText(
      canvas, "1", cv::Point(std::round(center_px.x - 4), std::round(center_px.y + 5)),
      cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(220, 235, 255), 1, cv::LINE_AA);

    // 4. 视觉检测管道输出标注
    bool is_detected = (static_cast<int>(i) == gt.visible_armor_id);
    if (is_detected) {
      has_detected = true;
      detected_center_px = center_px;

      // 4 个角点
      cv::circle(canvas, px_tl, 3, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
      cv::circle(canvas, px_tr, 3, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
      cv::circle(canvas, px_br, 3, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
      cv::circle(canvas, px_bl, 3, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);

      // 装甲四边形
      cv::polylines(canvas, armor_polys, true, cv::Scalar(0, 255, 128), 2, cv::LINE_AA);

      // Bounding Box
      cv::Rect bbox = cv::boundingRect(armor_poly);
      cv::Rect det_box(bbox.x - 4, bbox.y - 4, bbox.width + 8, bbox.height + 8);
      cv::rectangle(canvas, det_box, cv::Scalar(0, 255, 128), 1, cv::LINE_AA);

      // 角标
      int clen = std::min(8, std::min(det_box.width, det_box.height) / 3);
      cv::line(canvas, cv::Point(det_box.x, det_box.y), cv::Point(det_box.x + clen, det_box.y), cv::Scalar(0, 255, 128), 2);
      cv::line(canvas, cv::Point(det_box.x, det_box.y), cv::Point(det_box.x, det_box.y + clen), cv::Scalar(0, 255, 128), 2);
      cv::line(canvas, cv::Point(det_box.x + det_box.width, det_box.y), cv::Point(det_box.x + det_box.width - clen, det_box.y), cv::Scalar(0, 255, 128), 2);
      cv::line(canvas, cv::Point(det_box.x + det_box.width, det_box.y), cv::Point(det_box.x + det_box.width, det_box.y + clen), cv::Scalar(0, 255, 128), 2);

      // 标签 Badge
      std::string tag_str = fmt::format("B1: {:.2f}", obs.confidence);
      cv::Rect badge_rect(det_box.x, det_box.y - 18, 56, 17);
      cv::rectangle(canvas, badge_rect, cv::Scalar(0, 160, 75), -1);
      cv::putText(canvas, tag_str, cv::Point(badge_rect.x + 3, badge_rect.y + 13), cv::FONT_HERSHEY_DUPLEX, 0.36, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

      cv::drawMarker(canvas, cv::Point(std::round(center_px.x), std::round(center_px.y)), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 8, 1, cv::LINE_AA);
    }
  }

  // 5. 自瞄跟踪器 EKF 重投影
  if (est.x.size() >= 11) {
    Eigen::Vector3d ekf_center_3d(est.center.x(), est.center.y(), cam_height_);
    Eigen::Vector3d ekf_armor_3d = ekf_center_3d + Eigen::Vector3d(
      -est.radius * std::cos(est.yaw),
      -est.radius * std::sin(est.yaw),
      0.0
    );

    cv::Point2f px_ekf;
    double zc_ekf;
    if (project_world_to_cam(ekf_armor_3d, cam_pos, gimbal_yaw, gimbal_pitch, px_ekf, zc_ekf)) {
      int ekf_box_w = 34;
      int ekf_box_h = 24;
      cv::Rect ekf_rect(
        static_cast<int>(px_ekf.x - ekf_box_w / 2),
        static_cast<int>(px_ekf.y - ekf_box_h / 2),
        ekf_box_w, ekf_box_h);

      cv::Scalar ekf_col(255, 0, 255);
      cv::drawMarker(canvas, cv::Point(std::round(px_ekf.x), std::round(px_ekf.y)), ekf_col, cv::MARKER_CROSS, 10, 1, cv::LINE_AA);

      int bw = 6;
      cv::line(canvas, cv::Point(ekf_rect.x, ekf_rect.y), cv::Point(ekf_rect.x + bw, ekf_rect.y), ekf_col, 1);
      cv::line(canvas, cv::Point(ekf_rect.x, ekf_rect.y), cv::Point(ekf_rect.x, ekf_rect.y + bw), ekf_col, 1);
      cv::line(canvas, cv::Point(ekf_rect.x + ekf_rect.width, ekf_rect.y), cv::Point(ekf_rect.x + ekf_rect.width - bw, ekf_rect.y), ekf_col, 1);
      cv::line(canvas, cv::Point(ekf_rect.x + ekf_rect.width, ekf_rect.y), cv::Point(ekf_rect.x + ekf_rect.width, ekf_rect.y + bw), ekf_col, 1);
      cv::line(canvas, cv::Point(ekf_rect.x, ekf_rect.y + ekf_rect.height), cv::Point(ekf_rect.x + bw, ekf_rect.y + ekf_rect.height), ekf_col, 1);
      cv::line(canvas, cv::Point(ekf_rect.x, ekf_rect.y + ekf_rect.height), cv::Point(ekf_rect.x, ekf_rect.y + ekf_rect.height - bw), ekf_col, 1);
      cv::line(canvas, cv::Point(ekf_rect.x + ekf_rect.width, ekf_rect.y + ekf_rect.height), cv::Point(ekf_rect.x + ekf_rect.width - bw, ekf_rect.y + ekf_rect.height), ekf_col, 1);
      cv::line(canvas, cv::Point(ekf_rect.x + ekf_rect.width, ekf_rect.y + ekf_rect.height), cv::Point(ekf_rect.x + ekf_rect.width, ekf_rect.y + ekf_rect.height - bw), ekf_col, 1);

      cv::putText(canvas, "[EKF]", cv::Point(ekf_rect.x, ekf_rect.y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.35, ekf_col, 1, cv::LINE_AA);

      if (has_detected) {
        cv::line(canvas, detected_center_px, px_ekf, cv::Scalar(220, 50, 220), 1, cv::LINE_4);
      }
    }

    // 6. 弹道前馈超前瞄准点 (AIM LEAD)
    Eigen::Vector3d lead_aim_pt = ekf_armor_3d + est.velocity * metrics.flight_time;
    cv::Point2f px_lead;
    double zc_lead;
    if (project_world_to_cam(lead_aim_pt, cam_pos, gimbal_yaw, gimbal_pitch, px_lead, zc_lead)) {
      cv::Scalar amber_col(0, 215, 255);

      cv::circle(canvas, px_lead, 13, amber_col, 2, cv::LINE_AA);
      cv::circle(canvas, px_lead, 2, amber_col, -1, cv::LINE_AA);

      cv::line(canvas, cv::Point(px_lead.x - 18, px_lead.y), cv::Point(px_lead.x - 13, px_lead.y), amber_col, 2);
      cv::line(canvas, cv::Point(px_lead.x + 13, px_lead.y), cv::Point(px_lead.x + 18, px_lead.y), amber_col, 2);
      cv::line(canvas, cv::Point(px_lead.x, px_lead.y - 18), cv::Point(px_lead.x, px_lead.y - 13), amber_col, 2);
      cv::line(canvas, cv::Point(px_lead.x, px_lead.y + 13), cv::Point(px_lead.x, px_lead.y + 18), amber_col, 2);

      std::string lead_text = fmt::format("AIM LEAD ({:.2f}s)", metrics.flight_time);
      cv::putText(canvas, lead_text, cv::Point(px_lead.x + 18, px_lead.y + 4), cv::FONT_HERSHEY_SIMPLEX, 0.38, amber_col, 1, cv::LINE_AA);

      if (has_detected) {
        cv::line(canvas, detected_center_px, px_lead, cv::Scalar(0, 180, 220), 1, cv::LINE_AA);
      }
    }
  }

  // 7. 中心十字准星
  draw_reticle(canvas);

  // 8. OSD 状态看板
  draw_hud(canvas, gt, est, metrics);

  return canvas;
}

}  // namespace render

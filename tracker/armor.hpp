#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

struct Armor
{
  Color color = blue;
  cv::Point2f center;       // 图像中心
  cv::Point2f center_norm;  // 归一化坐标
  std::vector<cv::Point2f> points;

  ArmorType type = ArmorType::small;
  ArmorName name = ArmorName::one;
  ArmorPriority priority = ArmorPriority::first;
  double confidence = 1.0;

  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();  // 单位：m
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();   // 单位：m
  Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();  // 单位：rad
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();   // 单位：rad
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();   // 球坐标系 (yaw, pitch, distance)

  double yaw_raw = 0.0;  // rad

  Armor() = default;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP

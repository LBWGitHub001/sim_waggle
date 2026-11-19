# -----------------------------------------------------------------------------
# tracker 包：自瞄后端卡尔曼滤波算法 + TrackerNode 线程
# 独立子工程：include 路径只有 tracker/ 与 common/，编译期看不到 sim 头文件
# -----------------------------------------------------------------------------
add_library(tracker_node STATIC
  extended_kalman_filter.cpp
  math_tools.cpp
  target.cpp
  baseline_tracker.cpp
  tracker_node.cpp
)

target_include_directories(tracker_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# armor.hpp / math_tools.cpp 依赖 OpenCV（原版 auto_aim 遗留）
# OmniScope::Cpp 传递提供 omniscope 头文件路径 / OpenCV / fmt / nlohmann_json
target_link_libraries(tracker_node PUBLIC
  omni_common
  OmniScope::Cpp
  ${OpenCV_LIBS}
  Threads::Threads
)

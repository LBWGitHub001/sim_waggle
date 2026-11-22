# -----------------------------------------------------------------------------
# tracker 包：EKF + Target + TrackerNode
# include 只有 tracker/ 与 common/，编译期看不到 sim / aimer
# -----------------------------------------------------------------------------
add_library(tracker_node STATIC
  extended_kalman_filter.cpp
  math_tools.cpp
  target.cpp
  metrics.cpp
  tracker_node.cpp
)

target_include_directories(tracker_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(tracker_node PUBLIC
  omni_common
  OmniScope::Cpp
  Threads::Threads
)

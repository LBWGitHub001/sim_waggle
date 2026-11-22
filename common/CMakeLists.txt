# -----------------------------------------------------------------------------
# common 包：领域模型、话题消息、线程同步原语与评估体系
# （等价 ROS 的 msgs/common 包，是 sim 与 tracker 两包的唯一耦合点）
# -----------------------------------------------------------------------------
add_library(omni_common STATIC
  metrics.cpp
  trajectory.cpp
  aim.cpp
)

# 包内头文件以扁平风格 include（"types.hpp"），仅暴露本目录
target_include_directories(omni_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(omni_common PUBLIC
  Eigen3::Eigen
  tinympcstatic
)

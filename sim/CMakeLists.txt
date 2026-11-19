# -----------------------------------------------------------------------------
# sim 包：物理运动学与传感器感知模拟 + SimNode 线程
# 独立子工程：include 路径只有 sim/ 与 common/，编译期看不到 tracker 头文件
# -----------------------------------------------------------------------------
add_library(sim_node STATIC
  kinematics.cpp
  camera_sensor.cpp
  sim_node.cpp
)

target_include_directories(sim_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sim_node PUBLIC
  omni_common
  OmniScope::Cpp
  Threads::Threads
)

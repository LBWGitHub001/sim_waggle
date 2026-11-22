# -----------------------------------------------------------------------------
# common 包：消息类型、坐标系、通道（头文件库）
# sim / tracker / aimer 的唯一耦合点；无滤波、无火控
# -----------------------------------------------------------------------------
add_library(omni_common INTERFACE)

target_include_directories(omni_common INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(omni_common INTERFACE
  Eigen3::Eigen
)

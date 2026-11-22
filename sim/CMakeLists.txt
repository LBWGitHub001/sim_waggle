# -----------------------------------------------------------------------------
# sim 包：物理运动学 + SimNode
# include 只有 sim/ 与 common/，编译期看不到 tracker / aimer
# -----------------------------------------------------------------------------
add_library(sim_node STATIC
  kinematics.cpp
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

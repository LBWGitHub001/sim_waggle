# -----------------------------------------------------------------------------
# aimer 包：弹道 Aimer + TinyMPC Planner + AimerNode
# include 只有 aimer/ 与 common/；tinympcstatic 只链本包
# -----------------------------------------------------------------------------
add_library(aimer_node STATIC
  trajectory.cpp
  aim.cpp
  planner.cpp
  aimer_node.cpp
)

target_include_directories(aimer_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(aimer_node PUBLIC
  omni_common
  OmniScope::Cpp
  tinympcstatic
  Threads::Threads
)

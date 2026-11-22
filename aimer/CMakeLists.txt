# -----------------------------------------------------------------------------
# aimer 包：弹道 Aimer + AimerNode
# include 只有 aimer/ 与 common/
# -----------------------------------------------------------------------------
add_library(aimer_node STATIC
  trajectory.cpp
  aim.cpp
  aimer_node.cpp
)

target_include_directories(aimer_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(aimer_node PUBLIC
  omni_common
  OmniScope::Cpp
  Threads::Threads
)

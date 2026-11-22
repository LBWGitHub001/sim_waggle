# TinyMPC solver (vendored from auto_aim). Eigen from the system, not bundled.
add_library(tinympcstatic STATIC
  admm.cpp
  tiny_api.cpp
  rho_benchmark.cpp
)

set_property(TARGET tinympcstatic PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(tinympcstatic PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(tinympcstatic PUBLIC Eigen3::Eigen)

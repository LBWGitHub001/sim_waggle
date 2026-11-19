cmake_minimum_required(VERSION 3.16)
project(sim_waggle CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 依赖库查找
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# -----------------------------------------------------------------------------
# 零侵入引入外部 OmniScope SDK（链接 OmniScope::Cpp 主机端 C++ 遥测库）
# -----------------------------------------------------------------------------

set(OMNISCOPE_LOCAL_SDK "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/OmniScope/sdk")
if(EXISTS "${OMNISCOPE_LOCAL_SDK}/CMakeLists.txt")
  message(STATUS "Using OmniScope SDK from ${OMNISCOPE_LOCAL_SDK}")
  add_subdirectory("${OMNISCOPE_LOCAL_SDK}" ${CMAKE_CURRENT_BINARY_DIR}/omniscope_sdk EXCLUDE_FROM_ALL)
else()
  message(STATUS "OmniScope local SDK not found, fetching from GitHub...")
  include(FetchContent)
  FetchContent_Declare(
    omniscope
    GIT_REPOSITORY https://github.com/LBWGitHub001/OmniScope.git
    GIT_TAG main
    SOURCE_SUBDIR sdk
  )
  FetchContent_MakeAvailable(omniscope)
endif()

# -----------------------------------------------------------------------------
# 三个独立包（类似 ROS workspace 的独立 package）：
#   common/  消息与同步原语（两节点的唯一耦合点）
#   sim/     SimNode：物理仿真线程      —— 编译期看不到 tracker 头文件
#   tracker/ TrackerNode：EKF 追踪线程  —— 编译期看不到 sim 头文件
# -----------------------------------------------------------------------------
add_subdirectory(common)
add_subdirectory(sim)
add_subdirectory(tracker)

# -----------------------------------------------------------------------------
# 主程序：节点接线 + 可视化（render / bridge 为 app 侧私有）
# -----------------------------------------------------------------------------
add_executable(sim_waggle
  app/main.cpp
  render/camera_detector_renderer.cpp
  bridge/omniscope_bridge.cpp
)

# render / bridge 仍使用 "common/..." 根式 include
target_include_directories(sim_waggle PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sim_waggle PRIVATE
  sim_node
  tracker_node
  omni_common
  OmniScope::Cpp
  ${OpenCV_LIBS}
  fmt::fmt
  yaml-cpp
  nlohmann_json::nlohmann_json
  Threads::Threads
)

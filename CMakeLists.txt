cmake_minimum_required(VERSION 3.16)
project(sim_waggle CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
    GIT_REPOSITORY https://github.com/nuaa-rm/OmniScope.git
    GIT_TAG main
    SOURCE_SUBDIR sdk
  )
  FetchContent_MakeAvailable(omniscope)
endif()

# -----------------------------------------------------------------------------
# 四个独立包：
#   common/   消息 / 通道 / 坐标系（头文件）
#   sim/      SimNode
#   tracker/  EKF Target + TrackerNode
#   aimer/    弹道 Aimer + AimerNode
# -----------------------------------------------------------------------------
add_subdirectory(common)
add_subdirectory(sim)
add_subdirectory(tracker)
add_subdirectory(aimer)

# -----------------------------------------------------------------------------
# 主程序：YAML + 起停三线程。禁止 plot / plot_image。
# -----------------------------------------------------------------------------
add_executable(sim_waggle
  app/main.cpp
)

target_include_directories(sim_waggle PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sim_waggle PRIVATE
  sim_node
  tracker_node
  aimer_node
  omni_common
  OmniScope::Cpp
  fmt::fmt
  yaml-cpp
  nlohmann_json::nlohmann_json
  Threads::Threads
)

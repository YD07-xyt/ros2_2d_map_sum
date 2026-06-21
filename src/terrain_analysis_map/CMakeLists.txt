cmake_minimum_required(VERSION 3.10)
project(terrain_analysis_map)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(Boost_MINIMUM_VERSION 1.71.0)
find_package(Boost ${Boost_MINIMUM_VERSION} REQUIRED COMPONENTS date_time)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(spdlog REQUIRED)

# file(GLOB SRC_FILES "src/s.cpp")
# add_library(TerrainAnalysis ${SRC_FILES})

ament_auto_add_executable(terrain_analysis_map_node src/ros2_node.cpp)

target_link_libraries(terrain_analysis_map_node Boost::date_time spdlog::spdlog)

target_include_directories(terrain_analysis_map_node
  PUBLIC
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
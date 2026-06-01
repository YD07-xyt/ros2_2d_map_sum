cmake_minimum_required(VERSION 3.5)
project(uneven_map)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -g)


find_package(ament_cmake_auto REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)


ament_auto_find_build_dependencies()

ament_auto_add_library(uneven_map
  src/uneven_map.cpp
)


target_include_directories(uneven_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(uneven_map
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)


ament_auto_add_executable(uneven_map_node
  src/uneven_map_node.cpp
)


target_link_libraries(uneven_map_node
  uneven_map
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
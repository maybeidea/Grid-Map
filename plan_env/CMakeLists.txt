cmake_minimum_required(VERSION 3.20)

# Prefer Clang for local development and clangd. An explicitly selected
# compiler (for example, from a toolchain file or CMake command line) wins.
if(NOT DEFINED CMAKE_C_COMPILER AND NOT DEFINED CMAKE_CXX_COMPILER)
  find_program(PLAN_ENV_CLANG clang)
  find_program(PLAN_ENV_CLANGXX clang++)
  if(PLAN_ENV_CLANG AND PLAN_ENV_CLANGXX)
    set(CMAKE_C_COMPILER "${PLAN_ENV_CLANG}" CACHE FILEPATH "C compiler")
    set(CMAKE_CXX_COMPILER "${PLAN_ENV_CLANGXX}" CACHE FILEPATH "C++ compiler")
  endif()
endif()

project(plan_env LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 必须放 PCL 前面
find_package(MPI REQUIRED COMPONENTS C)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

add_library(plan_env SHARED src/grid_map.cpp)

target_include_directories(plan_env PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(plan_env PRIVATE
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    Threads::Threads
)

if(COMMAND ament_target_dependencies)
  ament_target_dependencies(plan_env
      geometry_msgs
      pcl_conversions
      rclcpp
      sensor_msgs
      tf2
      tf2_ros
  )
else()
  target_link_libraries(plan_env PRIVATE
      ${geometry_msgs_TARGETS}
      ${pcl_conversions_TARGETS}
      ${rclcpp_TARGETS}
      ${sensor_msgs_TARGETS}
      ${tf2_TARGETS}
      ${tf2_ros_TARGETS}
  )
endif()

install(TARGETS plan_env
  EXPORT export_plan_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
    geometry_msgs pcl_conversions rclcpp sensor_msgs tf2 tf2_ros
)

ament_package()

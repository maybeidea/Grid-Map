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

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PLAN_ENV_ENABLE_CCACHE "Use ccache to speed up local builds" ON)
if(PLAN_ENV_ENABLE_CCACHE)
  find_program(PLAN_ENV_CCACHE_EXECUTABLE NAMES ccache)
  if(PLAN_ENV_CCACHE_EXECUTABLE)
    # Keep explicit launcher settings from the toolchain or command line.
    if(NOT DEFINED CMAKE_C_COMPILER_LAUNCHER)
      set(CMAKE_C_COMPILER_LAUNCHER "${PLAN_ENV_CCACHE_EXECUTABLE}")
    endif()
    if(NOT DEFINED CMAKE_CXX_COMPILER_LAUNCHER)
      set(CMAKE_CXX_COMPILER_LAUNCHER "${PLAN_ENV_CCACHE_EXECUTABLE}")
    endif()
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(bimax_msgs QUIET)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# PCL's VTK dependencies require the MPI C target.
find_package(MPI REQUIRED COMPONENTS C)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

add_library(plan_env SHARED
    src/grid_map.cpp
    src/grid_map_cloud.cpp
    src/grid_map_motion.cpp
    src/grid_map_query.cpp
    src/grid_map_snapshot.cpp
    src/height_map.cpp
    src/occupancy.cpp
    src/ray_casting.cpp
    src/registration.cpp
    src/esdf.cpp
    src/tsdf.cpp
    src/observation_planner.cpp
)

target_include_directories(plan_env PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(plan_env PRIVATE
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    Threads::Threads
)

if(COMMAND ament_target_dependencies)
  ament_target_dependencies(plan_env PUBLIC
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

if(bimax_msgs_FOUND)
  target_compile_definitions(plan_env PUBLIC PLAN_ENV_HAS_BIMAX_MSGS=1)
  ament_target_dependencies(plan_env PUBLIC bimax_msgs)
endif()

add_executable(grid_map_standalone_test src/grid_map_standalone_test.cpp)
target_compile_definitions(grid_map_standalone_test PRIVATE
    PLAN_ENV_VIS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/vis"
    PLAN_ENV_BAG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/bag"
)
target_link_libraries(grid_map_standalone_test
    plan_env
    nlohmann_json::nlohmann_json
    ${PCL_LIBRARIES}
)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(grid_map_standalone_test
      builtin_interfaces
      geometry_msgs
      nav_msgs
      rclcpp
      rosbag2_cpp
      rosbag2_storage
      sensor_msgs
      std_msgs
      pcl_conversions
      tf2
      tf2_msgs
      tf2_ros
  )
else()
  target_link_libraries(grid_map_standalone_test
      ${builtin_interfaces_TARGETS}
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${rclcpp_TARGETS}
      ${rosbag2_cpp_TARGETS}
      ${rosbag2_storage_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${pcl_conversions_TARGETS}
      ${tf2_TARGETS}
      ${tf2_msgs_TARGETS}
      ${tf2_ros_TARGETS}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(grid_map_query_test test/grid_map_query_test.cpp)
  if(COMMAND ament_target_dependencies)
    ament_target_dependencies(grid_map_query_test
        geometry_msgs
        pcl_conversions
        rclcpp
        sensor_msgs
        tf2
        tf2_ros
    )
  else()
    target_link_libraries(grid_map_query_test
        ${geometry_msgs_TARGETS}
        ${pcl_conversions_TARGETS}
        ${rclcpp_TARGETS}
        ${sensor_msgs_TARGETS}
        ${tf2_TARGETS}
        ${tf2_ros_TARGETS}
    )
  endif()
  target_link_libraries(grid_map_query_test plan_env ${PCL_LIBRARIES})
endif()

install(TARGETS plan_env
  EXPORT export_plan_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS grid_map_standalone_test DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/gemini336_depth_scene.py
                scripts/gemini336_mapping_evaluator.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

add_executable(observation_planner_node src/observation_planner_node.cpp)
target_link_libraries(observation_planner_node plan_env nlohmann_json::nlohmann_json usb-1.0)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(observation_planner_node
      geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)
else()
  target_link_libraries(observation_planner_node
      ${geometry_msgs_TARGETS} ${rclcpp_TARGETS} ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS} ${tf2_TARGETS} ${tf2_ros_TARGETS})
endif()
install(TARGETS observation_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
    geometry_msgs pcl_conversions rclcpp sensor_msgs tf2 tf2_ros
)
if(bimax_msgs_FOUND)
  ament_export_dependencies(bimax_msgs)
endif()

ament_package()

cmake_minimum_required(VERSION 3.20)
project(plan_env LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PLAN_ENV_ENABLE_CCACHE "Use ccache to speed up local builds" ON)
if(PLAN_ENV_ENABLE_CCACHE AND NOT DEFINED CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(PLAN_ENV_CCACHE_EXECUTABLE NAMES ccache)
  if(PLAN_ENV_CCACHE_EXECUTABLE)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${PLAN_ENV_CCACHE_EXECUTABLE}")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# PCL I/O imports VTK targets that reference MPI::MPI_C.
find_package(MPI REQUIRED COMPONENTS C)
find_package(PCL REQUIRED COMPONENTS common filters registration io)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(bimax_msgs QUIET)
foreach(dependency builtin_interfaces geometry_msgs nav_msgs pcl_conversions
                   rclcpp rosbag2_cpp rosbag2_storage sensor_msgs std_msgs
                   tf2 tf2_msgs tf2_ros)
  find_package(${dependency} REQUIRED)
endforeach()

# Algorithms and immutable data have no ROS dependency.
add_library(plan_env_algorithms SHARED
  src/algorithms/esdf.cpp
  src/algorithms/occupancy.cpp
  src/algorithms/ray_casting.cpp
  src/algorithms/ray_integration.cpp
  src/algorithms/registration.cpp
  src/algorithms/tsdf.cpp
  src/algorithms/observation_planner.cpp
  src/utils/geometry.cpp
  src/utils/snapshot_query.cpp
)
target_include_directories(plan_env_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src ${PCL_INCLUDE_DIRS}
)
target_link_libraries(plan_env_algorithms PUBLIC Eigen3::Eigen
  PRIVATE ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

# Task orchestration, synchronization and ROS adapters.
add_library(plan_env SHARED
  src/mapping/grid_map.cpp
  src/mapping/grid_map_config.cpp
  src/mapping/grid_map_pipeline.cpp
  src/mapping/grid_map_cloud.cpp
  src/mapping/grid_map_fusion.cpp
  src/mapping/grid_map_motion.cpp
  src/mapping/grid_map_query.cpp
  src/mapping/grid_map_snapshot.cpp
  src/mapping/height_map.cpp
)
target_include_directories(plan_env PRIVATE src ${PCL_INCLUDE_DIRS})
target_link_libraries(plan_env PUBLIC plan_env_algorithms
  PRIVATE ${PCL_LIBRARIES} Threads::Threads)
ament_target_dependencies(plan_env PUBLIC
  geometry_msgs pcl_conversions rclcpp sensor_msgs tf2 tf2_ros)
if(bimax_msgs_FOUND)
  target_compile_definitions(plan_env PUBLIC PLAN_ENV_HAS_BIMAX_MSGS=1)
  ament_target_dependencies(plan_env PUBLIC bimax_msgs)
endif()

add_executable(grid_map_standalone_test src/nodes/grid_map_standalone_test.cpp)
target_compile_definitions(grid_map_standalone_test PRIVATE
  PLAN_ENV_VIS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/vis"
  PLAN_ENV_BAG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/bag")
target_include_directories(grid_map_standalone_test PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(grid_map_standalone_test
  plan_env nlohmann_json::nlohmann_json ${PCL_LIBRARIES})
ament_target_dependencies(grid_map_standalone_test
  builtin_interfaces geometry_msgs nav_msgs rclcpp rosbag2_cpp rosbag2_storage
  sensor_msgs std_msgs pcl_conversions tf2 tf2_msgs tf2_ros)

add_executable(observation_planner_node src/nodes/observation_planner_node.cpp)
target_link_libraries(observation_planner_node plan_env nlohmann_json::nlohmann_json)
ament_target_dependencies(observation_planner_node
  geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(grid_map_query_test test/grid_map_query_test.cpp)
  target_include_directories(grid_map_query_test PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(grid_map_query_test plan_env ${PCL_LIBRARIES})
  ament_target_dependencies(grid_map_query_test pcl_conversions rclcpp)
  ament_add_gtest(esdf_test test/esdf_test.cpp)
  target_link_libraries(esdf_test plan_env_algorithms)
endif()

install(TARGETS plan_env plan_env_algorithms
  EXPORT export_plan_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS grid_map_standalone_test observation_planner_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gemini336_depth_scene.py
                 scripts/gemini336_mapping_evaluator.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3
  geometry_msgs pcl_conversions rclcpp sensor_msgs tf2 tf2_ros)
if(bimax_msgs_FOUND)
  ament_export_dependencies(bimax_msgs)
endif()
ament_package()

option(PLAN_ENV_BUILD_BENCHMARKS "Build deterministic mapping CPU benchmark" OFF)
if(PLAN_ENV_BUILD_BENCHMARKS)
  add_executable(mapping_benchmark test/mapping_benchmark.cpp)
  target_include_directories(mapping_benchmark PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(mapping_benchmark plan_env ${PCL_LIBRARIES})
  ament_target_dependencies(mapping_benchmark pcl_conversions rclcpp)
  target_compile_definitions(mapping_benchmark PRIVATE PLAN_ENV_BENCHMARK_SOURCE_CACHE)
endif()

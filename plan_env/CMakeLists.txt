cmake_minimum_required(VERSION 3.20)

project(my_project LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 必须放 PCL 前面
find_package(MPI REQUIRED COMPONENTS C)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

add_executable(main src/grid_map.cpp)

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(main PRIVATE
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    Threads::Threads
)

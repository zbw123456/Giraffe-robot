cmake_minimum_required(VERSION 3.1)
project(giraffe_robot_cpp)

set(CMAKE_CXX_STANDARD 14)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  include
)

link_directories(${PINOCCHIO_LIBRARY_DIRS})

add_executable(giraffe_control src/main.cpp)
target_link_libraries(giraffe_control ${PINOCCHIO_LIBRARIES})

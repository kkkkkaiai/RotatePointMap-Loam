cmake_minimum_required(VERSION 3.12)


project(rotate)

find_package(Eigen)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(run rotate.cc)
target_link_libraries(run ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

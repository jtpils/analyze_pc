cmake_minimum_required(VERSION 2.8.3)
project(analyze_pc)

find_package(catkin REQUIRED pcl_conversions pcl_ros sensor_msgs roscpp tf_conversions)

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS filesystem system)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pclcube
)

set(pcl_cube_HDRS
    include/createCubePCL/pcl_cube.h)

set(pcl_cube_SRCS
    src/pcl_cube.cpp)

add_library(pclcube ${pcl_cube_HDRS} ${pcl_cube_SRCS})
target_link_libraries(pclcube ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(save_cube src/save_cube.cpp)
target_link_libraries(save_cube pclcube ${catkin_LIBRARIES})
add_executable(publish_cube src/publish_cube.cpp)
target_link_libraries(publish_cube pclcube ${catkin_LIBRARIES})
add_executable(compare_two_cubes src/compare_two_cubes.cpp)
target_link_libraries(compare_two_cubes pclcube ${catkin_LIBRARIES})
add_executable(analyze_pc src/analyze_pc.cpp src/dm_colors.hpp include/createCubePCL/analyze_pc.h)
target_link_libraries(analyze_pc ${VTK_LIBRARIES} ${catkin_LIBRARIES})
add_executable(coverage_pc src/coverage_pc.cpp include/createCubePCL/coverage_pc.h)
target_link_libraries(coverage_pc ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_executable(bridge_sim_process src/bridge_lasersim.cpp)
target_link_libraries(bridge_sim_process ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(density_test src/density_test.cpp)
target_link_libraries(density_test pclcube ${catkin_LIBRARIES})

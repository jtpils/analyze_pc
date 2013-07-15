cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set(pcl_cube_HDRS
    include/createCubePCL/pcl_cube.h)

set(pcl_cube_SRCS
    src/pcl_cube.cpp)

rosbuild_add_library(pclcube ${pcl_cube_HDRS} ${pcl_cube_SRCS})

rosbuild_add_executable(save_cube src/save_cube.cpp)
target_link_libraries(save_cube pclcube)
rosbuild_add_executable(publish_cube src/publish_cube.cpp)
target_link_libraries(publish_cube pclcube)
rosbuild_add_executable(compare_two_cubes src/compare_two_cubes.cpp)
target_link_libraries(compare_two_cubes pclcube)
rosbuild_add_executable(analyze_pc src/analyze_pc.cpp src/dm_colors.hpp include/createCubePCL/analyze_pc.h)
target_link_libraries(analyze_pc ${VTK_LIBRARIES})
rosbuild_add_executable(coverage_pc src/coverage_pc.cpp include/createCubePCL/coverage_pc.h)
rosbuild_link_boost(coverage_pc filesystem system)
rosbuild_add_executable(bridge_sim_process src/bridge_lasersim.cpp)

cmake_minimum_required(VERSION 3.10)
project(set_pose_plugin)

find_package(gz-sim REQUIRED)

add_library(set_pose_plugin SHARED SetPosePlugin.cc)
target_link_libraries(set_pose_plugin gz-sim::gz-sim)

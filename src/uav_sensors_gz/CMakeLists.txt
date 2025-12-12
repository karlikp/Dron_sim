cmake_minimum_required(VERSION 3.8)
project(uav_sensors_gz)

find_package(ament_cmake REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
  include
)

add_library(uav_gimbal_keep_nadir SHARED
  src/gimbal_keep_nadir.cpp
)

target_link_libraries(uav_gimbal_keep_nadir
  gz-sim7::gz-sim7
  gz-math7::gz-math7
  gz-plugin2::gz-plugin2
)

install(TARGETS uav_gimbal_keep_nadir
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

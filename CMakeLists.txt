cmake_minimum_required(VERSION 2.8.12)
project("pangolin_tutorial")

set(CMAKE_BUILD_TYPE "Debug")
# -O3优化代码
# -Wall 显示警告
# -g 生成调试信息
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")
add_subdirectory("slam_visualization")

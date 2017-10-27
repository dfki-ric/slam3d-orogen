project(slam3d)
cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
include(slam3dBase)

add_subdirectory(ruby)


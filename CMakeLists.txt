cmake_minimum_required(VERSION 2.6)
find_package(Rock)

add_definitions(-std=c++0x)
add_definitions(-std=gnu++0x)

rock_init(graph_slam 0.1)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

rock_standard_layout()



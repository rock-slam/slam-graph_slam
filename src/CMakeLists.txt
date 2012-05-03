rock_library(graph_slam
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(graph_slam_bin Main.cpp
    DEPS graph_slam)


#rock_testsuite(test_core unit/core.cpp
#    DEPS graph_slam)

rock_executable(graph_slam_gicp_test_bin test_gicp_graph_slam.cpp
    DEPS graph_slam)
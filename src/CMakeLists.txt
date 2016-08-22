
find_package(PCL 1.7 REQUIRED COMPONENTS registration)
rock_library(graph_slam
    SOURCES 
        VisualPoseGraph.cpp 
        PoseGraph.cpp 
        SensorMaps.cpp 
        VisualSensorMaps.cpp
        MapSegmentPoseGraph.cpp
        Hogman2Eigen.cpp
        edge_se3_gicp.cpp
        vertex_se3_gicp.cpp
        types_velodyne_slam.cpp
        pointcloud_helper.cpp
        extended_sparse_optimizer.cpp
        matrix_helper.cpp
        vertex_grid.cpp
    HEADERS 
        VisualPoseGraph.hpp 
        PoseGraph.hpp 
        SensorMaps.hpp 
        VisualSensorMaps.hpp
        MapSegmentPoseGraph.hpp
        edge_se3_gicp.hpp
        vertex_se3_gicp.hpp
        types_velodyne_slam.hpp
        pointcloud_helper.hpp
        extended_sparse_optimizer.hpp
        matrix_helper.hpp
        vertex_grid.hpp
        graph_slam_config.hpp
    DEPS_PKGCONFIG 
        envire base-types hogman stereo g2o pcl_registration-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
    DEPS_CMAKE
        CSparse Cholmod BLAS LAPACK
    )


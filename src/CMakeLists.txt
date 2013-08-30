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
    DEPS_PKGCONFIG 
        pcl_registration-1.7 envire base-types hogman stereo g2o
    DEPS_CMAKE
        CSparse Cholmod BLAS LAPACK
    )


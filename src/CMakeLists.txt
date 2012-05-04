rock_library(graph_slam
    SOURCES VisualPoseGraph.cpp PoseGraph.cpp SensorMaps.cpp VisualSensorMaps.cpp
    HEADERS VisualPoseGraph.hpp PoseGraph.hpp SensorMaps.hpp VisualSensorMaps.hpp
    DEPS_PKGCONFIG envire base-types hogman stereo
    )


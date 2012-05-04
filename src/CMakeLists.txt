rock_library(graph_slam
    SOURCES PoseGraph.cpp SensorMaps.cpp VisualSensorMaps.cpp
    HEADERS PoseGraph.hpp SensorMaps.hpp VisualSensorMaps.hpp
    DEPS_PKGCONFIG envire base-types hogman stereo
    )


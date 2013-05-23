#include "types_velodyne_slam.hpp"

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

namespace graph_slam 
{
    
    G2O_REGISTER_TYPE_GROUP(graph_slam);

    G2O_REGISTER_TYPE(GRAPH_SLAM_VERTEX_SE3_GICP, VertexSE3_GICP);
    G2O_REGISTER_TYPE(GRAPH_SLAM_EDGE_SE3_GICP, EdgeSE3_GICP);
    
    G2O_REGISTER_ACTION(VertexSE3_GICPWriteGnuplotAction);
    G2O_REGISTER_ACTION(EdgeSE3_GICPWriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
    G2O_REGISTER_ACTION(VertexSE3_GICPDrawAction);
    G2O_REGISTER_ACTION(EdgeSE3_GICPDrawAction);
#endif

}
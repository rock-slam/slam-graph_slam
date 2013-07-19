#ifndef GRAPH_SLAM_VERTEX_SE3_GICP_H
#define GRAPH_SLAM_VERTEX_SE3_GICP_H

#include <g2o/types/slam3d/vertex_se3.h>
#include <envire/maps/Pointcloud.hpp>
#include <base/samples/rigid_body_state.h>

namespace graph_slam 
{

class VertexSE3_GICP : public g2o::VertexSE3
{
public:
    
    VertexSE3_GICP();
    void attachPointCloud(envire::Pointcloud* point_cloud);
    void detachPointCloud();
    envire::EnvironmentItem::Ptr getEnvirePointCloud() const;
    
protected:
    envire::EnvironmentItem::Ptr envire_pointcloud;
};


class VertexSE3_GICPWriteGnuplotAction: public g2o::VertexSE3WriteGnuplotAction 
{
public:
    VertexSE3_GICPWriteGnuplotAction() : VertexSE3WriteGnuplotAction ()
    {
        setTypeName(typeid(VertexSE3_GICP).name());
    }
};
#ifdef G2O_HAVE_OPENGL
class VertexSE3_GICPDrawAction: public g2o::VertexSE3DrawAction
{
public:
    VertexSE3_GICPDrawAction() : VertexSE3DrawAction() 
    {
        setTypeName(typeid(VertexSE3_GICP).name());
    }
};
#endif

} // end namespace

#endif
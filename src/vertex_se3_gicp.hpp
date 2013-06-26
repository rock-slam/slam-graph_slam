#ifndef GRAPH_SLAM_VERTEX_SE3_GICP_H
#define GRAPH_SLAM_VERTEX_SE3_GICP_H

#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <envire/maps/Pointcloud.hpp>
#include <base/samples/rigid_body_state.h>
#include <graph_slam/matrix_helper.hpp>

namespace graph_slam 
{

class VertexSE3_GICP : public g2o::VertexSE3
{
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;
    typedef typename PCLPointCloud::ConstPtr PCLPointCloudConstPtr;
    
    VertexSE3_GICP();
    void attachPointCloud(envire::Pointcloud* point_cloud, double density = 1.0);
    void detachPointCloud();
    PCLPointCloudConstPtr getPCLPointCloud() const;
    bool updateEnvireTransformation();
    
protected:
    envire::EnvironmentItem::Ptr envire_pointcloud;
    PCLPointCloudPtr pcl_point_cloud;
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
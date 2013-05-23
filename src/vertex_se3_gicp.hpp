#ifndef GRAPH_SLAM_VERTEX_SE3_GICP_H
#define GRAPH_SLAM_VERTEX_SE3_GICP_H

#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace graph_slam 
{
    
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;
typedef typename PCLPointCloud::ConstPtr PCLPointCloudConstPtr;

class VertexSE3_GICP : public g2o::VertexSE3
{
public:
    VertexSE3_GICP();
    void attachPointCloud(const std::vector<Eigen::Vector3d> &point_cloud, double density = 1.0);
    void detachPointCloud();
    PCLPointCloudConstPtr getPCLPointCloud();
    
protected:
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
#include "vertex_se3_gicp.hpp"

#include <graph_slam/pointcloud_helper.hpp>

namespace graph_slam
{
    
VertexSE3_GICP::VertexSE3_GICP() : VertexSE3()
{

}

void VertexSE3_GICP::attachPointCloud(envire::Pointcloud* point_cloud)
{
    envire_pointcloud.reset(point_cloud);
}

void VertexSE3_GICP::detachPointCloud()
{
    envire_pointcloud.reset();
}

envire::EnvironmentItem::Ptr VertexSE3_GICP::getEnvirePointCloud() const
{
    return envire_pointcloud;
}

}
#include "vertex_se3_gicp.hpp"
#include "pointcloud_helper.hpp"

namespace graph_slam
{
    
VertexSE3_GICP::VertexSE3_GICP() : VertexSE3()
{

}

void VertexSE3_GICP::attachPointCloud(const std::vector< Eigen::Vector3d >& point_cloud, double density)
{
    pcl_point_cloud.reset(new PCLPointCloud);
    vectorToPCLPointCloud(point_cloud, *pcl_point_cloud, density);
}

void VertexSE3_GICP::detachPointCloud()
{
    pcl_point_cloud.reset();
}

PCLPointCloudConstPtr VertexSE3_GICP::getPCLPointCloud()
{
    return pcl_point_cloud;
}
    
}



























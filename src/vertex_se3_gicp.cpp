#include "vertex_se3_gicp.hpp"

#include <graph_slam/pointcloud_helper.hpp>

namespace graph_slam
{
    
VertexSE3_GICP::VertexSE3_GICP() : VertexSE3()
{

}

void VertexSE3_GICP::attachPointCloud(envire::Pointcloud* point_cloud, double density)
{
    envire_pointcloud.reset(point_cloud);
    pcl_point_cloud.reset(new PCLPointCloud);
    vectorToPCLPointCloud(point_cloud->vertices, *pcl_point_cloud, density);
}

void VertexSE3_GICP::detachPointCloud()
{
    envire_pointcloud.reset();
    pcl_point_cloud.reset();
}

VertexSE3_GICP::PCLPointCloudConstPtr VertexSE3_GICP::getPCLPointCloud() const
{
    return pcl_point_cloud;
}

bool VertexSE3_GICP::updateEnvireTransformation()
{
    envire::CartesianMap* map = static_cast<envire::CartesianMap*>(envire_pointcloud.get());
    if(map != NULL)
    {
        envire::FrameNode* fn = map->getFrameNode();
        if(fn != NULL)
        {
            //TODO: add envire:TransformWithUncertanty to vertex
            fn->setTransform(Eigen::Affine3d(estimate().matrix()));
            return true;
        }
    }
    return false;
}
    
}
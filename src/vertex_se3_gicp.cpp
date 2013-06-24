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

void VertexSE3_GICP::setOdometryPose(const base::samples::RigidBodyState& odometry_pose)
{
    Eigen::Isometry3d pose;
    //note: this is ok, because the affine transformation of RigidBodyState is isometric.
    pose.matrix() = odometry_pose.getTransform().matrix();
    
    Matrix6d covariance = combineToPoseCovariance(odometry_pose.cov_position, odometry_pose.cov_orientation);
    
    setOdometryPose(pose, covariance);
}

    
}
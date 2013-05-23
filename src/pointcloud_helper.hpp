#ifndef GRAPH_SLAM_POINTCLOUD_HELPER_H
#define GRAPH_SLAM_POINTCLOUD_HELPER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace graph_slam 
{

    void vectorToPCLPointCloud(const std::vector<Eigen::Vector3d>& pc, pcl::PointCloud<pcl::PointXYZ> &pcl_pc, double density = 1.0);
    
    void transformPointCloud(const std::vector<Eigen::Vector3d>& pc, std::vector<Eigen::Vector3d>& transformed_pc, const Eigen::Affine3d &transformation);
    void transformPointCloud(std::vector<Eigen::Vector3d>& pc, const Eigen::Affine3d &transformation);
    
} // end namespace

#endif

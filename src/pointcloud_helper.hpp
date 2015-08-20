#ifndef GRAPH_SLAM_POINTCLOUD_HELPER_H
#define GRAPH_SLAM_POINTCLOUD_HELPER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <base/samples/DepthMap.hpp>

namespace graph_slam 
{

    void vectorToPCLPointCloud(const std::vector<Eigen::Vector3d>& pc, pcl::PointCloud<pcl::PointXYZ> &pcl_pc, double density = 1.0);
    
    void transformPointCloud(const std::vector<Eigen::Vector3d>& pc, std::vector<Eigen::Vector3d>& transformed_pc, const Eigen::Affine3d &transformation);
    void transformPointCloud(std::vector<Eigen::Vector3d>& pc, const Eigen::Affine3d &transformation);

    /**
     * This method filters outliers according to a minimum distance from the scan origin.
     * @param laser_scan input scan
     * @param min_range min range the measurement should have
     */
    void filterMinDistance(base::samples::DepthMap &laser_scan, float min_range);

    /**
     * This method filters outliers according to the maximum angle to each of their four (or three) direct neighbors.
     * It mainly aims to remove outliers which arise after edges and have therefore a high angle to the next neighbor.
     * @param laser_scan input scan
     * @param max_deviation_angle the angle to a valid neighbor should be lower than this angle
     * @param min_neighbors minimum number of valid neighbors
     */
    void filterOutliers(base::samples::DepthMap &laser_scan, double max_deviation_angle, unsigned min_neighbors = 1);

    /**
     * It computes the angle alpha if (a) is the longer ray and (b) is the shorter ray.
     * In that case the angle alpha is always the maximum angle in the triangle build
     * by the origin and the tips of both rays.
     */
    double computeMaximumAngle(double angle_between_rays, double dist_ray_1, double dist_ray_2);
    
} // end namespace

#endif

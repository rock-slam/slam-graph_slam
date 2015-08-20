#include "pointcloud_helper.hpp"

namespace graph_slam
{
    
void vectorToPCLPointCloud(const std::vector< Eigen::Vector3d >& pc, pcl::PointCloud< pcl::PointXYZ >& pcl_pc, double density)
{    
    pcl_pc.clear();
    std::vector<bool> mask;
    unsigned sample_count = (unsigned)(density * pc.size());
    
    if(density <= 0.0 || pc.size() == 0)
    {
        return;
    }
    else if(sample_count >= pc.size())
    {
        mask.resize(pc.size(), true);
    }
    else
    {
        mask.resize(pc.size(), false);
        unsigned samples_drawn = 0;
        
        while(samples_drawn < sample_count)
        {
            unsigned index = rand() % pc.size();
            if(mask[index] == false)
            {
                mask[index] = true;
                samples_drawn++;
            }
        }
    }
    
    for(unsigned i = 0; i < pc.size(); i++)
    {
        if(mask[i])
            pcl_pc.push_back(pcl::PointXYZ(pc[i].x(), pc[i].y(), pc[i].z()));
    }
}

void transformPointCloud(const std::vector< Eigen::Vector3d >& pc, std::vector< Eigen::Vector3d >& transformed_pc, const Eigen::Affine3d& transformation)
{
    transformed_pc.clear();
    for(std::vector< Eigen::Vector3d >::const_iterator it = pc.begin(); it != pc.end(); it++)
    {
        transformed_pc.push_back(transformation * (*it));
    }
}

void transformPointCloud(std::vector< Eigen::Vector3d >& pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< Eigen::Vector3d >::iterator it = pc.begin(); it != pc.end(); it++)
    {
        *it = (transformation * (*it));
    }
}

void filterMinDistance(base::samples::DepthMap& laser_scan, float min_range)
{
    for(unsigned i = 0; i < laser_scan.distances.size(); i++)
    {
        // check min range of measurement
        if(laser_scan.distances[i] < min_range)
            laser_scan.distances[i] = 0.0;
    }
}

void filterOutliers(base::samples::DepthMap& laser_scan, double max_deviation_angle, unsigned min_neighbors)
{
    // check for invalid input or nothing to do
    if(max_deviation_angle >= M_PI || max_deviation_angle < 0.0 || laser_scan.distances.size() == 0 ||
        laser_scan.vertical_size == 0 || laser_scan.horizontal_size == 0 ||
        laser_scan.horizontal_projection != base::samples::DepthMap::POLAR ||
        laser_scan.vertical_projection != base::samples::DepthMap::POLAR)
        return;

    // copy range measurements
    std::vector<base::samples::DepthMap::scalar> range_measurements = laser_scan.distances;

    // compute vertical angles
    std::vector<base::Angle> vertical_angles(laser_scan.vertical_size);
    if(laser_scan.vertical_interval.size() == 1)
    {
        vertical_angles.resize(laser_scan.vertical_size, base::Angle::fromRad(laser_scan.vertical_interval[0]));
    }
    else if(laser_scan.vertical_interval.size() == laser_scan.vertical_size)
    {
        for(unsigned v = 0; v < laser_scan.vertical_size; v++)
            vertical_angles[v] = base::Angle::fromRad(laser_scan.vertical_interval[v]);
    }
    else if(laser_scan.vertical_interval.size() == 2)
    {
        double step_resolution = 0.0;
        if(laser_scan.vertical_interval.back() == laser_scan.vertical_interval.front())
            step_resolution = (2.0 * M_PI) / (double)(laser_scan.vertical_size-1);
        else
        {
            double diff = std::abs(base::Angle::normalizeRad(laser_scan.vertical_interval.back() - laser_scan.vertical_interval.front()));
            step_resolution = diff / (double)(laser_scan.vertical_size-1);
        }
        for(unsigned v = 0; v < laser_scan.vertical_size; v++)
            vertical_angles[v] = base::Angle::fromRad(laser_scan.vertical_interval.front() + ((double)v * step_resolution));
    }
    else
        throw std::range_error("The vertical interval size must be equal to vertical_size or 2!");

    // compute horizontal angles
    std::vector<base::Angle> horizontal_angles(laser_scan.horizontal_size);
    if(laser_scan.horizontal_interval.size() == 1)
    {
        horizontal_angles.resize(laser_scan.horizontal_size, base::Angle::fromRad(laser_scan.horizontal_interval[0]));
    }
    else if(laser_scan.horizontal_interval.size() == laser_scan.horizontal_size)
    {
        for(unsigned h = 0; h < laser_scan.horizontal_size; h++)
            horizontal_angles[h] = base::Angle::fromRad(laser_scan.horizontal_interval[h]);
    }
    else if(laser_scan.horizontal_interval.size() == 2)
    {
        double step_resolution = 0.0;
        if(laser_scan.horizontal_interval.back() == laser_scan.horizontal_interval.front())
            step_resolution = (2.0 * M_PI) / (double)(laser_scan.horizontal_size-1);
        else
        {
            double diff = std::abs(base::Angle::normalizeRad(laser_scan.horizontal_interval.back() - laser_scan.horizontal_interval.front()));
            step_resolution = diff / (double)(laser_scan.horizontal_size-1);
        }
        for(unsigned h = 0; h < laser_scan.horizontal_size; h++)
            horizontal_angles[h] = base::Angle::fromRad(laser_scan.horizontal_interval.front() + ((double)h * step_resolution));
    }
    else
        throw std::range_error("The horizontal interval size must be equal to horizontal_size or 2!");


    for(unsigned v = 1; v < laser_scan.vertical_size-1; v++)
    {
        double v_angle_up = std::abs((vertical_angles[v+1] - vertical_angles[v]).getRad());
        double v_angle_down = std::abs((vertical_angles[v] - vertical_angles[v-1]).getRad());

        for(unsigned h = 0; h < laser_scan.horizontal_size; h++)
        {
            // continue if current ray is already invalid
            if(!laser_scan.isMeasurementValid(v,h))
                continue;

            unsigned left_index = (h == 0 ? laser_scan.horizontal_size-1 : h-1);
            unsigned righ_index = (h == laser_scan.horizontal_size-1 ? 0 : h+1);

            double h_angle_left = std::abs((horizontal_angles[h] - horizontal_angles[left_index]).getRad());
            double h_angle_right = std::abs((horizontal_angles[righ_index] - horizontal_angles[h]).getRad());

            base::samples::DepthMap::scalar current_ray_dist = laser_scan.distances[laser_scan.getIndex(v,h)];
            unsigned neighbors_found = 0;

            // find at least one valid neighbor
            if(laser_scan.isMeasurementValid(v,left_index) &&
                max_deviation_angle >= computeMaximumAngle(h_angle_left, current_ray_dist, laser_scan.distances[laser_scan.getIndex(v,left_index)]))
            {
                neighbors_found++;
                if(neighbors_found >= min_neighbors)
                    continue;
            }
            if(laser_scan.isMeasurementValid(v,righ_index) &&
                max_deviation_angle >= computeMaximumAngle(h_angle_right, current_ray_dist, laser_scan.distances[laser_scan.getIndex(v,righ_index)]))
            {
                neighbors_found++;
                if(neighbors_found >= min_neighbors)
                    continue;
            }
            if(laser_scan.isMeasurementValid(v-1,h) &&
                max_deviation_angle >= computeMaximumAngle(v_angle_down, current_ray_dist, laser_scan.distances[laser_scan.getIndex(v-1,h)]))
            {
                neighbors_found++;
                if(neighbors_found >= min_neighbors)
                    continue;
            }
            if(laser_scan.isMeasurementValid(v+1,h) &&
                max_deviation_angle >= computeMaximumAngle(v_angle_up, current_ray_dist, laser_scan.distances[laser_scan.getIndex(v+1,h)]))
            {
                neighbors_found++;
            }

            if(neighbors_found >= min_neighbors)
                continue;

            // invalidate the current ray if too less valid neighbors have been found
            range_measurements[laser_scan.getIndex(v,h)] = base::NaN<base::samples::DepthMap::scalar>();
        }
    }

    laser_scan.distances = range_measurements;
}

double computeMaximumAngle(double angle_between_rays, double dist_ray_1, double dist_ray_2)
{
    if(dist_ray_1 <= 0.0 || dist_ray_2 <= 0.0)
        throw std::range_error("The ray distants have to be positive!");
    else if(angle_between_rays <= 0.0 || angle_between_rays >= M_PI)
        throw std::range_error("The angle between the rays has to be positive and smaller than PI");
    else if(dist_ray_1 == dist_ray_2)
        return (M_PI - angle_between_rays) * 0.5;

    double min_dist = std::min(dist_ray_1, dist_ray_2);
    double max_dist = std::max(dist_ray_1, dist_ray_2);

    double oposite_angle = atan2(sin(angle_between_rays) * min_dist, max_dist - (cos(angle_between_rays) * min_dist));
    return M_PI - (oposite_angle + angle_between_rays);
}
    
}
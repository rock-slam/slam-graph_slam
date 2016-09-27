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

}
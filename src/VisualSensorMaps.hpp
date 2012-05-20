#ifndef GRAPH_SLAM_VISUAL_SENSOR_MAPS__
#define GRAPH_SLAM_VISUAL_SENSOR_MAPS__

#include <graph_slam/SensorMaps.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Featurecloud.hpp>

namespace graph_slam
{
class VisualSensorMaps : public SensorMaps
{
public:
    VisualSensorMaps();
    explicit VisualSensorMaps( envire::FrameNode* fn );
    void updateExtents();
    void updateMaps();
    void associate( SensorMaps *maps, std::vector<envire::TransformWithUncertainty>& constraints );

protected:
    envire::Pointcloud *stereoMap;
    envire::Featurecloud *sparseMap;
    // add more maps that can be associated here

    const size_t min_sparse_correspondences;
    
    /** 
     * try to associate two sparse feature clouds.
     *
     * @return the number of matching interframe features. This can be used as a measure of quality
     * for the match.
     */
    size_t associateSparseMap( envire::Featurecloud *fc1, envire::Featurecloud *fc2, std::vector<envire::TransformWithUncertainty>& constraints );
    void associateStereoMap( envire::Pointcloud* pc1, envire::Pointcloud* pc2, std::vector<envire::TransformWithUncertainty>& constraints );
};
}
#endif

#ifndef GRAPH_SLAM_SENSOR_MAPS_HPP__
#define GRAPH_SLAM_SENSOR_MAPS_HPP__

#include <envire/Core.hpp>

namespace graph_slam
{

/** helper struct that caches some information on the structure of the map
 * graph and associated information (e.g. bounding box).
 */
class SensorMaps
{
public:
    virtual ~SensorMaps();

    void setFrameNode( envire::FrameNode* a );

    void update();

    virtual void updateExtents() = 0;

    virtual void associate( SensorMaps *maps, std::vector<envire::TransformWithUncertainty>& constraints ) = 0;

    // update the bounds of the map using the uncertainty 
    // associated with the framenode 
    void updateBounds( double sigma = 3.0 );

    // store the frameNode pointer
    envire::FrameNode::Ptr frameNode;

    // id of the vertex associated with the frameNode 
    long vertexId;

    // cached local bounds
    Eigen::AlignedBox<double, 3> extents;

    /// The bounding box of the maps in global frame 
    /// including uncertainty
    Eigen::AlignedBox<double, 3> bounds;
};

}
#endif

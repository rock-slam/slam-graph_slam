#ifndef GRAPH_SLAM_VISUAL_POSE_GRAPH__
#define GRAPH_SLAM_VISUAL_POSE_GRAPH__

#include <graph_slam/PoseGraph.hpp>
#include <graph_slam/VisualSensorMaps.hpp>

#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Featurecloud.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/operators/DistanceGridToPointcloud.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <stereo/sparse_stereo.hpp>

#include <envire/operators/MLSProjection.hpp>
#include <envire/maps/MLSGrid.hpp>

namespace graph_slam
{

class VisualPoseGraph : public PoseGraph
{
protected:
    /** minimum number of sparse correspondences required to consider a match
     * successfull.
     */
    size_t min_sparse_correspondences;

protected:
    // body frame of the robot
    envire::FrameNode::Ptr bodyFrame;

    // chain for processing distance images
    envire::FrameNode::Ptr distFrame;
    envire::DistanceGrid::Ptr distGrid;
    envire::ImageRGB24::Ptr textureGrid;
    envire::DistanceGridToPointcloud::Ptr distOp;

    // chain for feature clouds
    envire::FrameNode::Ptr featureFrame;
    envire::Featurecloud::Ptr featurecloud;

    // pointer to relevant nodes
    envire::FrameNode::Ptr prevBodyFrame;
    envire::FrameNode::Ptr currentBodyFrame;

    // MLS map and projection operator
    envire::MLSGrid::Ptr mlsGrid;
    envire::MLSProjection::Ptr mlsOp;

public:
    VisualPoseGraph( envire::Environment* env, int num_levels = 3, int node_distance = 2 );

    void initNode( const envire::TransformWithUncertainty &body2bodyPrev, const envire::TransformWithUncertainty &body2world );

    /** will prepare a new node based on an initial transformation
     */
    void initNode( const envire::TransformWithUncertainty &body2bodyPrev );

    /** adds a sensor reading for a distance image to an initialized node
     */
    void addSensorReading( const base::samples::DistanceImage& distImage, const Eigen::Affine3d& sensor2body, const base::samples::frame::Frame& textureImage );

    /** adds a sensor reading for a feature array to an initialized node
     */
    void addSensorReading( const stereo::StereoFeatureArray& featureArray, const Eigen::Affine3d& sensor2body );

    /** @brief adds an initialized node, with optional sensor readings to the node graph
     *
     * requires a previous call to initNode(), as well addSensorReading() calls
     * for each sensor reading.
     */
    void addNode();

    using PoseGraph::addNode;

protected:
    virtual SensorMaps* createSensorMaps( envire::FrameNode* fn );
};

}
#endif

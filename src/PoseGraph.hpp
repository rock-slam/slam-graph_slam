#ifndef __GRAPH_SLAM_POSE_GRAPH_HPP__
#define __GRAPH_SLAM_POSE_GRAPH_HPP__

#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/Featurecloud.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/operators/DistanceGridToPointcloud.hpp>

#include <aislib/graph_optimizer/graph_optimizer3d_hchol.h>
#include <envire/ransac.hpp>
#include <envire/icpConfigurationTypes.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <stereo/sparse_stereo.hpp>

#include <envire/operators/MLSProjection.hpp>
#include <envire/maps/MLSGrid.hpp>

namespace graph_slam
{

Transformation3 eigen2Hogman( const Eigen::Affine3d& eigen_transform );

Matrix6 eigen2Hogman( const Eigen::Matrix<double,6,6>& eigen_matrix );

Eigen::Matrix<double,6,6> hogman2Eigen( const Matrix6& hogman );

Eigen::Affine3d hogman2Eigen( const Transformation3& hogman_transform );

/** 
 * transform a 6x6 covariance matrix in [r t] order to a hogman 6 inverse
 * covariance matrix in [t r] format.
 */
Matrix6 envireCov2HogmanInf( const Eigen::Matrix<double,6,6>& eigen_matrix );

/** 
 * Transform a 6x6 covariance matrix in [t r] order from hogman Covariance
 * format to the envire [r t] order.
 */
Eigen::Matrix<double,6,6> hogmanCov2EnvireCov( const Matrix6& hogman_matrix );

class SensorMaps;

class PoseGraph
{
protected:
    /** maximum distance between nodes, where we check for potential
     * correspondence 
     */
    double max_node_radius;

    /** minimum number of sparse correspondences required to consider a match
     * successfull.
     */
    size_t min_sparse_correspondences;

protected:
    envire::Environment *env;
    AISNavigation::GraphOptimizer3D *optimizer;
    std::map<long, SensorMaps*> nodeMap;

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
    PoseGraph( envire::Environment* env, int num_levels = 3, int node_distance = 2 );

    ~PoseGraph();

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

    /** adds an initialized node, with optional sensor readings to the node graph
     *
     * requires a previous call to initNode(), as well addSensorReading() calls
     * for each sensor reading.
     */
    void addNode();

    /** will return a sensormaps structure for a given 
     * framenode. creates a new one, if not already existing.
     */
    SensorMaps* getSensorMaps( envire::FrameNode* fn );

    /** 
     * associate two framenodes, if they are within a feasable distance
     * between each other and have overlapping bounding boxes.
     *
     * @return true if an association has been added
     */ 
    bool associateNodes( envire::FrameNode* a, envire::FrameNode* b );

    void associateStereoMap( envire::Pointcloud* pc1, envire::Pointcloud* pc2 );

    /** 
     * try to associate two sparse feature clouds.
     *
     * @return the number of matching interframe features. This can be used as a measure of quality
     * for the match.
     */
    size_t associateSparseMap( envire::Featurecloud *fc1, envire::Featurecloud *fc2 );
};
}

#endif

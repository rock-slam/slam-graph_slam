#ifndef __GRAPH_SLAM_POSE_GRAPH_HPP__
#define __GRAPH_SLAM_POSE_GRAPH_HPP__

#include <envire/Core.hpp>
#include <aislib/graph_optimizer/graph_optimizer3d_hchol.h>
#include <graph_slam/SensorMaps.hpp>

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

protected:
    envire::Environment *env;
    AISNavigation::GraphOptimizer3D *optimizer;
    std::map<std::string, SensorMaps*> nodeMap;

public:
    PoseGraph( envire::Environment* env, int num_levels = 3, int node_distance = 2 );

    virtual ~PoseGraph();

    /** @brief add a new constraint to the pose graph
     *
     * @param fn1 first frameNode for which to add a constraint
     * @param fn2 second frameNode for which to add a constriant
     * @param transform transformation with uncertainty from fn2 to fn1
     */
    void addConstraint( const envire::FrameNode* fn1, const envire::FrameNode* fn2, const envire::TransformWithUncertainty& transform );

    /** @brief add a new vertex to the pose graph
     *
     * @param fn the framenode, which is tracked by the new node
     * @param transform the transform from this node to the root with uncertainty
     */
    void addNode( envire::FrameNode* fn, const envire::TransformWithUncertainty& transform );

    /** @brief add a new vertex to the pose graph
     *
     * Will use the transform associated with the FrameNode for initialising
     * the position and the uncertainty.
     *
     * @param fn the framenode, which is tracked by the new node
     */
    void addNode( envire::FrameNode* fn );

    /** @brief will try to associate the node with other nodes in the graph
     *
     * Which nodes it will try to associate with depends on the internal
     * heuristics. For each association that is found, a new constraint
     * is generated and added to the graph.
     */
    void associateNode( envire::FrameNode* fn );

    /** @brief will run the graph optimization and write the results back 
     * to the FrameNodes
     */
    void optimizeNodes( int iterations = 5 );

    /** will return a sensormaps structure for a given 
     * framenode. creates a new one, if not already existing.
     */
    SensorMaps* getSensorMaps( envire::FrameNode* fn );

protected:
    /** 
     * associate two framenodes, if they are within a feasable distance
     * between each other and have overlapping bounding boxes.
     *
     * @return true if an association has been added
     */ 
    bool associateNodes( envire::FrameNode* a, envire::FrameNode* b );

    virtual SensorMaps* createSensorMaps( envire::FrameNode* fn ) = 0;
};
}

#endif

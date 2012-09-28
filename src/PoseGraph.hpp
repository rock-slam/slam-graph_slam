#ifndef __GRAPH_SLAM_POSE_GRAPH_HPP__
#define __GRAPH_SLAM_POSE_GRAPH_HPP__

#include <envire/Core.hpp>
#include <graph_slam/SensorMaps.hpp>

namespace graph_slam
{

class SensorMaps;
struct OptimizerImpl;

class PoseGraph
{
protected:
    /** maximum distance between nodes, where we check for potential
     * correspondence 
     */
    double max_node_radius;

protected:
    envire::Environment *env;
    OptimizerImpl *optimizer;
    std::map<std::string, SensorMaps*> nodeMap;

public:
    /** @brief Constructor for PoseGraph 
     *
     * @param env the environment the PoseGraph is applied to
     * @param num_levels number of hierarchy levels for the constraint solver
     * @param node_distance @todo describe from hogman
     */
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

    /** @will return a sensormaps structure for a given 
     * framenode. creates a new one, if not already existing.
     */
    SensorMaps* getSensorMaps( envire::FrameNode* fn );

protected:
    /** 
     * @brief associate two framenodes, if they are within a feasable distance
     * between each other and have overlapping bounding boxes.
     *
     * @return true if an association has been added
     */ 
    bool associateNodes( envire::FrameNode* a, envire::FrameNode* b );

    /** 
     * @brief create a new SensorMaps structure for the given FrameNode
     *
     * Implement this method in a subclass, and return an object which is a
     * subclass of SensorMaps.
     */
    virtual SensorMaps* createSensorMaps( envire::FrameNode* fn ) = 0;
};
}

#endif

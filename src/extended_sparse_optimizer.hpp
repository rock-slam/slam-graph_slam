#ifndef GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP
#define GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP

#include <g2o/core/sparse_optimizer.h>
#include <base/samples/rigid_body_state.h>
#include <graph_slam/edge_se3_gicp.hpp>


namespace graph_slam 
{
    
class ExtendedSparseOptimizer : public g2o::SparseOptimizer
{
public:
    
    ExtendedSparseOptimizer();
    virtual ~ExtendedSparseOptimizer();
    
    bool addVertex(const base::samples::RigidBodyState& rigid_body_state, envire::Pointcloud* point_cloud, bool delayed_icp_update = false);
    
    void updateGICPConfiguration(const GICPConfiguration& gicp_config);
    
    virtual int optimize(int iterations, bool online = false);
    
    bool updateEnvireTransformations();
    
    
protected:
    
private:
    void setupOptimizer();
    
    bool initialized;
    uint64_t next_vertex_id;
    g2o::HyperGraph::VertexSet vertices_to_add;
    g2o::HyperGraph::EdgeSet edges_to_add;
    GICPConfiguration gicp_config;
};
    
} // end namespace

#endif
#ifndef GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP
#define GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP

#include <g2o/core/sparse_optimizer.h>
#include <base/samples/rigid_body_state.h>
#include <graph_slam/edge_se3_gicp.hpp>
#include <graph_slam/matrix_helper.hpp>
#include <envire/core/Transform.hpp>


namespace graph_slam 
{
    
class ExtendedSparseOptimizer : public g2o::SparseOptimizer
{
public:
    
    ExtendedSparseOptimizer();
    virtual ~ExtendedSparseOptimizer();
    
    bool addVertex(const envire::TransformWithUncertainty& transformation, envire::Pointcloud* point_cloud, bool delayed_icp_update = false);
    
    void findNewEdges(int vertex_id);
    void findNewEdgesForLastN(int last_n_vertices);
    
    bool getVertexCovariance(Matrix6d& covariance, const Vertex* vertex);
    envire::TransformWithUncertainty getEnvireTransformWithUncertainty(const g2o::VertexSE3* vertex);
    
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
    Eigen::Isometry3d odometry_pose_last_vertex;
    Matrix6d odometry_covariance_last_vertex;
};
    
} // end namespace

#endif
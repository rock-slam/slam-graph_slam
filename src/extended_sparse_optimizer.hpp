#ifndef GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP
#define GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP

#include <g2o/core/sparse_optimizer.h>
#include <base/samples/rigid_body_state.h>
#include <graph_slam/edge_se3_gicp.hpp>
#include <graph_slam/matrix_helper.hpp>
#include <graph_slam/vertex_grid.hpp>
#include <envire/core/Transform.hpp>
#include <boost/shared_ptr.hpp>
#include <envire/core/Environment.hpp>
#include <envire/operators/MLSProjection.hpp>

namespace graph_slam 
{
    
class ExtendedSparseOptimizer : public g2o::SparseOptimizer
{
public:
    
    ExtendedSparseOptimizer();
    virtual ~ExtendedSparseOptimizer();

    virtual int optimize(int iterations, bool online = false);
    virtual void clear();
    
    bool addVertex(const envire::TransformWithUncertainty& transformation, std::vector<Eigen::Vector3d>& pointcloud, 
                   const Eigen::Affine3d& sensor_origin = Eigen::Affine3d::Identity(), bool delayed_icp_update = false);
    bool removePointcloudFromVertex(int vertex_id);
    bool removeVertex(int vertex_id);

    void setupMaxVertexGrid(unsigned max_vertices_per_cell, double grid_size_x, double grid_size_y, double cell_resolution);
    void removeVerticesFromGrid();
    
    void findEdgeCandidates(int vertex_id, const g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv);
    void findEdgeCandidates();
    void tryBestEdgeCandidates(unsigned count = 1);
    
    bool getVertexCovariance(Matrix6d& covariance, const g2o::OptimizableGraph::Vertex* vertex);
    bool getVertexCovariance(Matrix6d& covariance, const g2o::OptimizableGraph::Vertex* vertex, const g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv);
    envire::TransformWithUncertainty getEnvireTransformWithUncertainty(const g2o::OptimizableGraph::Vertex* vertex, const g2o::SparseBlockMatrix<Eigen::MatrixXd>* spinv = 0);

    void setMLSMapConfiguration(bool use_mls, double grid_size_x, double grid_size_y, double cell_resolution_x, double cell_resolution_y, double min_z, double max_z);
    bool updateEnvire();
    boost::shared_ptr<envire::Environment> getEnvironment() {return env;};
    
    void updateGICPConfiguration(const GICPConfiguration& gicp_config);
    
    bool adjustOdometryPose(const base::samples::RigidBodyState& odometry_pose, base::samples::RigidBodyState& adjusted_odometry_pose) const;
    
    void dumpGraphViz(std::ostream& os);
    
protected:
    
private:
    void setupOptimizer();
    void initValues();
    bool isHandledByOptimizer(const g2o::OptimizableGraph::Vertex* vertex) const {return vertex->hessianIndex() >= 0;};
    
    bool initialized;
    int next_vertex_id;
    g2o::HyperGraph::VertexSet vertices_to_add;
    g2o::HyperGraph::EdgeSet edges_to_add;
    GICPConfiguration gicp_config;
    Eigen::Isometry3d odometry_pose_last_vertex;
    Matrix6d odometry_covariance_last_vertex;
    graph_slam::VertexSE3_GICP* last_vertex;
    bool new_edges_added;
    boost::shared_ptr<envire::Environment> env;
    boost::shared_ptr<envire::MLSProjection> projection;
    boost::shared_ptr<VertexGrid> vertex_grid;
    bool use_mls;
    bool use_vertex_grid;
    bool map_update_necessary;
};
    
} // end namespace

#endif
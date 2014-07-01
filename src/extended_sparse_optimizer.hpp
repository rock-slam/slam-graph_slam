#ifndef GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP
#define GRAPH_SLAM_EXTENDED_SPARSE_OPTIMIZER_HPP

#include <g2o/core/sparse_optimizer.h>
#include <base/samples/rigid_body_state.h>
#include <graph_slam/edge_se3_gicp.hpp>
#include <graph_slam/matrix_helper.hpp>
#include <graph_slam/vertex_grid.hpp>
#include <graph_slam/graph_slam_config.hpp>
#include <envire/core/Transform.hpp>
#include <boost/shared_ptr.hpp>
#include <envire/core/Environment.hpp>
#include <envire/operators/MLSProjection.hpp>

namespace graph_slam 
{
    
class ExtendedSparseOptimizer : public g2o::SparseOptimizer
{
public:
    
    /** Available linear solvers */
    enum LinearSolver
    {
	CSparse = 0,
	Cholmod
    };

    /** Available optimization algorithms */
    enum OptimizationAlgorithm
    {
	GaussNewton = 0,
	LevenbergMarquardt
    };
    
    
    ExtendedSparseOptimizer(OptimizationAlgorithm optimizer = GaussNewton, LinearSolver solver = CSparse);
    virtual ~ExtendedSparseOptimizer();

    
    /** Runs the graph optimization.
     * 
     * @param iterations number of iterations
     * @param online do on-line optimization
     */
    virtual int optimize(int iterations, bool online = false);
    
    /** Removes all vertices and edges */
    virtual void clear();

    
    /** Add a a-priori set of vertices and edges from a given environment
     * Note: The graph structure (e.g. vertices and edges) are generated from 
     * the transformations in the envire environment. 
     * The a-priori part of the graph will be handled separately until a 
     * connection (edge) to the current part of the map is found.
     * 
     * @param apriori_env envire environment
     */
    bool setAPrioriMap(const boost::shared_ptr<envire::Environment>& apriori_env);
    
    
    /** Adds a new vertex to the graph.
     * The delta transformation from the previous vertex will be computed, using
     * the given absolute transformation to the new vertex.
     * 
     * @param transformation initial, i.e. odometry based, pose of the vertex
     * @param pointcloud range measurements
     * @param sensor_origin origin of the range measurements
     * @param delayed_icp_update run icp optimization at the same time as the graph optimization 
     */
    bool addVertex(const envire::TransformWithUncertainty& transformation, std::vector<Eigen::Vector3d>& pointcloud, 
                   const Eigen::Affine3d& sensor_origin = Eigen::Affine3d::Identity(), bool delayed_icp_update = false);
    
    /** Adds the initial vertex to the graph.
     * The pose of this vertex is the fixed reference and will therefore not be optimized.
     * This method is called by addVertex() automatically if it's the first vertex. 
     * 
     * @param transformation initial, i.e. odometry based, pose of the vertex
     * @param pointcloud range measurements
     * @param sensor_origin origin of the range measurements
     */
    bool addInitalVertex(const envire::TransformWithUncertainty& transformation, std::vector<Eigen::Vector3d>& pointcloud, const Eigen::Affine3d& sensor_origin = Eigen::Affine3d::Identity());
    
    /** Removes the range measurements attached to a given vertex.
     * Doesn't remove the vertex itself.
     * 
     * @param vertex_id id of the vertex
     */
    bool removePointcloudFromVertex(int vertex_id);
    
    /** NOTE This is currently the same as removePointcloudFromVertex().
     * It should remove a vertex later completely.
     * 
     * @param vertex_id id of the vertex
     */
    bool removeVertex(int vertex_id);

    
    /** With this method a 2D grid based removal of old vertices can be activated.
     * This 2D grid is independend from any generated maps.
     * 
     * @param max_vertices_per_cell allowed vertices per cell
     * @param grid_size_x size of the grid in x direction in meters
     * @param grid_size_y size of the grid in y direction in meters
     * @param cell_resolution size of each grid cell in meters
     */
    void setupMaxVertexGrid(unsigned max_vertices_per_cell, double grid_size_x, double grid_size_y, double cell_resolution);
    
    /** Deletes obsolete vertices on basis of a 2D grid, setup with setupMaxVertexGrid() */
    void removeVerticesFromGrid();
    
    
    /** Does a search for edge candidates for all vertices, for which
     * it hasn't been done yet. Using this method, a search for candidates 
     * is only triggered once per vertex in its lifetime.
     */
    void findEdgeCandidates();
    
    /** Uses the mahalanobis distance to find possible candidates 
     * for additional edges from a given vertex to all other vertices.
     * 
     * @param vertex_id id of a vertex
     * @param spinv combined covariance matrix of all vertices
     */
    void findEdgeCandidates(int vertex_id, const g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv);

    /** Tries to optimize a transformation between two vertices using GICP.
     * This is done based on the edge candidates, beginning with the shortest 
     * mahalanobis distance. New edges are only added to the graph if the found 
     * GICP transformation is considered as valid.
     * 
     * @param count amount of candidates that should be optimized
     */
    void tryBestEdgeCandidates(unsigned count = 1);
    
    
    /** Returns the covariance matrix for a given vertex.
     * 
     * @param covariance 6x6 covariance matrix
     * @param vertex
     */
    bool getVertexCovariance(Matrix6d& covariance, const g2o::OptimizableGraph::Vertex* vertex);
    
    /** Returns envire transformation with uncertainty.
     * 
     * @param vertex
     * @param spinv (optional) combined covariance matrix of all vertices
     */
    envire::TransformWithUncertainty getEnvireTransformWithUncertainty(const g2o::OptimizableGraph::Vertex* vertex, const g2o::SparseBlockMatrix<Eigen::MatrixXd>* spinv = 0);

    
    /** Sets up a multi-level sureface grid map with the given configuration.
     * All measurements are projected in this map. On default configuration
     * the MLS map is not used.
     * 
     * @param use_mls create a MLS map and project measurements
     * @param mls_config MLS map specific configuration
     * @param mls_id envire id of the mls map
     * @param grid_size_x size of the mls map in x direction in meters
     * @param grid_size_y size of the mls map in y direction in meters
     * @param cell_resolution_x size of each grid cell in x direction in meters
     * @param cell_resolution_y size of each grid cell in y direction in meters
     * @param min_z lower boundary of the map in z direction
     * @param max_z upper boundary of the map in z direction
     */
    void setMLSMapConfiguration(bool use_mls, const envire::MLSConfiguration& mls_config, const std::string& mls_id, 
				double grid_size_x, double grid_size_y, double cell_resolution_x, double cell_resolution_y, double min_z, double max_z);
    
    /** Updates the transformations of all pointclouds in envire and 
     * project them to the multi-level sureface map.
     */
    bool updateEnvire();
    
    /** Returns a shared pointer to the environment representation. */
    boost::shared_ptr<envire::Environment> getEnvironment() {return env;};
    
    
    /** Updates the configuration of the GICP algorithm.
     * 
     * @gicp_config GICP specific configuration
     */
    void updateGICPConfiguration(const GICPConfiguration& gicp_config);
    
    /** Sets the transformation to the map offset in the world frame.
     * The offset of the map is in its center.
     */
    void setMap2WorldTransformation(const Eigen::Isometry3d& map2world) {this->map2world = map2world; map2world_frame->setTransform(Eigen::Affine3d(map2world.matrix()));}
    
    /** Returns the map offset in the world frame. */
    const Eigen::Isometry3d& getMap2WorldTransformation() const {return map2world;}
    
    /** Sets the robot start pose in the world frame. */
    void setRobotStart2WorldTransformation(const Eigen::Isometry3d& robot_start2world) {this->robot_start2world = robot_start2world;}
    
    /** Returns the robot start pose in the world frame. */
    const Eigen::Isometry3d& getRobotStart2WorldTransformation() const {return robot_start2world;}
    
    
    /** Returns the current most likely pose of the robot.
     * The most likely pose is the pose of the newest vertex 
     * plus the local odometry delta transformation.
     * 
     * @param odometry_pose the current odometry based pose of the robot
     * @param adjusted_odometry_pose most likely pose of the robot
     */
    bool adjustOdometryPose(const base::samples::RigidBodyState& odometry_pose, base::samples::RigidBodyState& adjusted_odometry_pose) const;
    
    
    /** Writes the current graph configuration in Graphviz representation to the
     * given output stream.
     * 
     * @param os output stream
     */
    void dumpGraphViz(std::ostream& os);
    
protected:
    /** Attaches an a-priori existing set of pointclouds to the current graph.
     * This method will be called if a transformation between an a-priori existing state
     * and a new state has been found.
     */
    bool attachAPrioriMap();
    
    /** Returns the covariance matrix for a given vertex.
     * 
     * @param covariance 6x6 covariance matrix
     * @param vertex
     * @param spinv combined covariance matrix of all vertices
     */
    bool getVertexCovariance(Matrix6d& covariance, const g2o::OptimizableGraph::Vertex* vertex, const g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv);
    
private:
    /** Sets up the optimizer and the linear matrix solver */
    void setupOptimizer(OptimizationAlgorithm optimizer, LinearSolver solver);
    /** Initializes all member variables with valid values. */
    void initValues();
    /** Checks if a vertex is already part of the optimization process. */
    bool isHandledByOptimizer(const g2o::OptimizableGraph::Vertex* vertex) const {return vertex->hessianIndex() >= 0;};
    
    bool initialized;
    int next_vertex_id;
    g2o::HyperGraph::VertexSet vertices_to_add;
    g2o::HyperGraph::EdgeSet edges_to_add;
    GICPConfiguration gicp_config;
    Eigen::Isometry3d odometry_pose_last_vertex;
    Matrix6d odometry_covariance_last_vertex;
    Matrix6d covariance_last_optimized_vertex;
    graph_slam::VertexSE3_GICP* last_vertex;
    bool new_edges_added;
    boost::shared_ptr<envire::Environment> env;
    envire::MLSProjection::Ptr projection;
    envire::FrameNode* map2world_frame;
    boost::shared_ptr<VertexGrid> vertex_grid;
    bool use_mls;
    bool use_vertex_grid;
    bool map_update_necessary;
    g2o::SparseOptimizer cov_graph;
    Eigen::Isometry3d map2world;
    Eigen::Isometry3d robot_start2world;
    std::vector<graph_slam::VertexSE3_GICP*> apriori_vertices;
};
    
} // end namespace

#endif
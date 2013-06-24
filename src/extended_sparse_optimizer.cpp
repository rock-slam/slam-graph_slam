#include "extended_sparse_optimizer.hpp"

#include <limits>
#include <graph_slam/vertex_se3_gicp.hpp>

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>


namespace graph_slam 
{
    
ExtendedSparseOptimizer::ExtendedSparseOptimizer() : SparseOptimizer(), next_vertex_id(0), initialized(false)
{
    setupOptimizer();
}

ExtendedSparseOptimizer::~ExtendedSparseOptimizer()
{

}

void ExtendedSparseOptimizer::setupOptimizer()
{
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    //typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    //typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    //typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    
    // allocating the optimizer
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    
    setAlgorithm(solver);
}

void ExtendedSparseOptimizer::updateGICPConfiguration(const GICPConfiguration& gicp_config)
{
    this->gicp_config = gicp_config;
    
    for(g2o::HyperGraph::EdgeSet::iterator it = _edges.begin(); it != _edges.end(); it++)
    {
        graph_slam::EdgeSE3_GICP* edge = static_cast<graph_slam::EdgeSE3_GICP*>(*it);
        if(edge)
            edge->setGICPConfiguration(gicp_config);
    }
}

bool ExtendedSparseOptimizer::addVertex(const base::samples::RigidBodyState& rigid_body_state, envire::Pointcloud* point_cloud, bool delayed_icp_update)
{
    if(next_vertex_id == std::numeric_limits<uint64_t>::max())
    {
        // this should not happen under normal circumstances
        throw std::runtime_error("Can't add any new vertex. Max id count has been reached.");
        return false;
    }
    
    // create new vertex
    graph_slam::VertexSE3_GICP* vertex = new graph_slam::VertexSE3_GICP();
    vertex->setId(next_vertex_id);
    
    // save odometry pose in vertex
    vertex->setOdometryPose(rigid_body_state);
    
    // attach point cloud to vertex
    vertex->attachPointCloud(point_cloud, gicp_config.point_cloud_density);
    
    // added vertex to the graph
    if(!g2o::SparseOptimizer::addVertex(vertex))
    {
        std::cerr << "failed to add a new vertex." << std::endl;
        delete vertex;
        return false;
    }
    
    if(next_vertex_id == 0)
    {
        // set first vertex fixed
        vertex->setFixed(true);
        
        // set odometry pose as inital pose
        vertex->setEstimate(vertex->getOdometryPose());
    }
    else
    {
        graph_slam::VertexSE3_GICP *source_vertex = static_cast<graph_slam::VertexSE3_GICP*>(this->vertex(next_vertex_id-1));
        
        // set pose of the source vertex times odometry delta as inital pose
        vertex->setEstimate(source_vertex->estimate() * (source_vertex->getOdometryPose().inverse() * vertex->getOdometryPose()));
        
        // create a edge between the last and the new vertex
        graph_slam::EdgeSE3_GICP* edge = new graph_slam::EdgeSE3_GICP();
        edge->setSourceVertex(source_vertex);
        edge->setTargetVertex(vertex);
        
        if(delayed_icp_update)
            if(!edge->setMeasurementFromOdometry())
                throw std::runtime_error("compute transformation from odometry failed!");
        else
            if(!edge->setMeasurementFromGICP())
                throw std::runtime_error("compute transformation using gicp failed!");
        
        
        if(!g2o::SparseOptimizer::addEdge(edge))
        {
            std::cerr << "failed to add a new edge." << std::endl;
            g2o::SparseOptimizer::removeVertex(vertex);
            delete edge;
            delete vertex;
            return false;
        }
        edges_to_add.insert(edge);
    }
    
    vertices_to_add.insert(vertex);
    
    next_vertex_id++;
    return true;
}

int ExtendedSparseOptimizer::optimize(int iterations, bool online)
{
    // update hessian matrix
    if(vertices_to_add.size())
    {
        if(initialized)
        {
            if(!updateInitialization(vertices_to_add, edges_to_add))
                throw std::runtime_error("update optimization failed!");
        }
        else
        {
            if(!initializeOptimization())
                throw std::runtime_error("initialize optimization failed!");
            initialized = true;
        }
        vertices_to_add.clear();
        edges_to_add.clear();
    }
    
    
    // do optimization
    return g2o::SparseOptimizer::optimize(iterations, online);
}

bool ExtendedSparseOptimizer::updateEnvireTransformations()
{
    unsigned err_counter = 0;
    for(VertexIDMap::const_iterator it = _vertices.begin(); it != _vertices.end(); it++)
    {
        graph_slam::VertexSE3_GICP *vertex = static_cast<graph_slam::VertexSE3_GICP*>(it->second);
        if(vertex && !vertex->updateEnvireTransformation())
            err_counter++;
    }
    return !err_counter;
}
    
}
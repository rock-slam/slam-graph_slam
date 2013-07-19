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
    
ExtendedSparseOptimizer::ExtendedSparseOptimizer() : SparseOptimizer(), next_vertex_id(0), initialized(false), 
        odometry_pose_last_vertex(Eigen::Isometry3d::Identity()), odometry_covariance_last_vertex(Matrix6d::Identity()), last_vertex(NULL),
        new_edges_added(false)
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
        graph_slam::EdgeSE3_GICP* edge = dynamic_cast<graph_slam::EdgeSE3_GICP*>(*it);
        if(edge)
            edge->setGICPConfiguration(gicp_config);
    }
}

bool ExtendedSparseOptimizer::addVertex(const envire::TransformWithUncertainty& transformation, envire::Pointcloud* point_cloud, bool delayed_icp_update)
{
    if(next_vertex_id == std::numeric_limits<int>::max())
    {
        // this should not happen under normal circumstances
        throw std::runtime_error("Can't add any new vertex. Max id count has been reached.");
        return false;
    }
    
    // get odometry pose and covariance
    Eigen::Isometry3d odometry_pose(transformation.getTransform().matrix());
    Matrix6d odometry_covariance = switchEnvireG2oCov(transformation.getCovariance());
    
    // check for nan values
    if(is_nan(odometry_pose.matrix()))
    {
        throw std::runtime_error("Odometry pose matrix contains not numerical entries!");
        return false;
    }
    else if(is_nan(odometry_covariance))
    {
        throw std::runtime_error("Odometry covaraince matrix contains not numerical entries!");
        return false;
    }
    
    // create new vertex
    graph_slam::VertexSE3_GICP* vertex = new graph_slam::VertexSE3_GICP();
    vertex->setId(next_vertex_id);
    
    // attach point cloud to vertex
    vertex->attachPointCloud(point_cloud);
    
    // added vertex to the graph
    if(!g2o::SparseOptimizer::addVertex(vertex))
    {
        std::cerr << "failed to add a new vertex." << std::endl;
        delete vertex;
        return false;
    }
    
    if(next_vertex_id == 0 || last_vertex == NULL)
    {
        // set first vertex fixed
        vertex->setFixed(true);
        
        // set odometry pose as inital pose
        vertex->setEstimate(odometry_pose);
    }
    else
    {
        Eigen::Isometry3d odometry_pose_delta = odometry_pose_last_vertex.inverse() * odometry_pose;
        Matrix6d odometry_covariance_delta = odometry_covariance_last_vertex.inverse() * odometry_covariance;
        
        // set pose of the source vertex times odometry delta as inital pose
        vertex->setEstimate(last_vertex->estimate() * odometry_pose_delta);
 
        // create an edge between the last and the new vertex
        graph_slam::EdgeSE3_GICP* edge = new graph_slam::EdgeSE3_GICP();
        edge->setSourceVertex(last_vertex);
        edge->setTargetVertex(vertex);
        edge->setGICPConfiguration(gicp_config);
        
        edge->setMeasurement(odometry_pose_delta);
        edge->setInformation(odometry_covariance_delta.inverse());

        if(!edge->setMeasurementFromGICP(delayed_icp_update))
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
    odometry_pose_last_vertex = odometry_pose;
    odometry_covariance_last_vertex = odometry_covariance;
    last_vertex = vertex;
    
    next_vertex_id++;
    return true;
}

void ExtendedSparseOptimizer::findEdgeCandidates()
{
    for(g2o::OptimizableGraph::VertexContainer::const_iterator it = _activeVertices.begin(); it != _activeVertices.end(); it++)
    {
        graph_slam::VertexSE3_GICP *vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(*it);
        if(vertex && !vertex->getEdgeSearchState().has_run)
        {
            findEdgeCandidates(vertex->id());
        }
        // TODO add a check for vertecies, if the pose has significantly changed
    }
}

void ExtendedSparseOptimizer::findEdgeCandidates(int vertex_id)
{
    graph_slam::VertexSE3_GICP *source_vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(this->vertex(vertex_id));
    Matrix6d source_covariance;
    if(source_vertex && getVertexCovariance(source_covariance, source_vertex))
    {
        for(g2o::OptimizableGraph::VertexContainer::iterator it = _activeVertices.begin(); it != _activeVertices.end(); it++)
        {
            graph_slam::VertexSE3_GICP *target_vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(*it);
            if(target_vertex && (vertex_id < target_vertex->id()-1 || vertex_id > target_vertex->id()+1))
            {
                // check if vertecies have already an edge
                unsigned equal_edges = 0;
                for(g2o::HyperGraph::EdgeSet::const_iterator sv_edge = source_vertex->edges().begin(); sv_edge != source_vertex->edges().end(); sv_edge++)
                {
                    equal_edges += target_vertex->edges().count(*sv_edge);
                }
                
                // there should never be more than one edge between two vertecies
                assert(equal_edges <= 1);
                
                Matrix6d target_covariance;
                if(equal_edges == 0 && getVertexCovariance(target_covariance, target_vertex))
                {
                    // try to add a new edge
                    Eigen::Matrix3d position_covariance = source_covariance.topLeftCorner<3,3>() + target_covariance.topLeftCorner<3,3>();
                    // TODO: get correct covariance, for now use the identity
                    position_covariance = Eigen::Matrix3d::Identity();
                    
                    double distance = computeMahalanobisDistance<double, 3>(source_vertex->estimate().translation(), 
                                                                            position_covariance, 
                                                                            target_vertex->estimate().translation());
                    
                    if(distance <= gicp_config.max_sensor_distance)
                    {
                        source_vertex->addEdgeCandidate(target_vertex->id(), distance);
                        target_vertex->addEdgeCandidate(source_vertex->id(), distance);
                        new_edges_added = true;
                    }
                }
            }
        }
        // save search pose
        source_vertex->setEdgeSearchState(true, source_vertex->estimate());
    }
}

void ExtendedSparseOptimizer::tryBestEdgeCandidate()
{
    if(!new_edges_added)
        return;
    
    double vertex_error = 0.0;
    graph_slam::VertexSE3_GICP* source_vertex = NULL;
    for(g2o::OptimizableGraph::VertexContainer::iterator it = _activeVertices.begin(); it != _activeVertices.end(); it++)
    {
        graph_slam::VertexSE3_GICP* vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(*it);
        if(vertex && vertex->getMissingEdgesError() > vertex_error)
        {
            vertex_error = vertex->getMissingEdgesError();
            source_vertex = vertex;
        }
    }
    
    if(vertex_error == 0.0)
    {
        new_edges_added = false;
        return;
    }
    
    VertexSE3_GICP::EdgeCandidate candidate;
    int target_id;
    if(source_vertex && source_vertex->getBestEdgeCandidate(candidate, target_id))
    {
        graph_slam::VertexSE3_GICP *target_vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(this->vertex(target_id));
        if(!target_vertex)
            return;
        
        graph_slam::EdgeSE3_GICP* edge = new graph_slam::EdgeSE3_GICP();
        edge->setSourceVertex(source_vertex);
        edge->setTargetVertex(target_vertex);
        edge->setGICPConfiguration(gicp_config);

        if(!edge->setMeasurementFromGICP())
            throw std::runtime_error("compute transformation using gicp failed!");
        
        // add the new edge to the graph if the icp allignment was successful
        if(edge->hasValidGICPMeasurement())
        {
            if(!g2o::SparseOptimizer::addEdge(edge))
            {
                std::cerr << "failed to add a new edge." << std::endl;
                delete edge;
            }
            else if(_verbose)
                std::cerr << "Added new edge between vertex " << source_vertex->id() << " and " << target_vertex->id() 
                            << ". Mahalanobis distance was " << candidate.mahalanobis_distance << ", edge error was " << candidate.error << std::endl;
            
            edges_to_add.insert(edge);
            source_vertex->removeEdgeCandidate(target_id);
            target_vertex->removeEdgeCandidate(source_vertex->id());
        }
        else
        {
            delete edge;
            source_vertex->updateEdgeCandidate(target_id, true);
            target_vertex->updateEdgeCandidate(source_vertex->id(), true);
        }
    }
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
        graph_slam::VertexSE3_GICP *vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(it->second);
        if(vertex)
        {
            envire::CartesianMap* map = dynamic_cast<envire::CartesianMap*>(vertex->getEnvirePointCloud().get());
            if(map)
            {
                envire::FrameNode* framenode = map->getFrameNode();
                if(framenode)
                {
                    framenode->setTransform(getEnvireTransformWithUncertainty(vertex));
                    continue;
                }
            }
        }
        err_counter++;
    }
    return !err_counter;
}

bool ExtendedSparseOptimizer::getVertexCovariance(Matrix6d& covariance, const g2o::OptimizableGraph::Vertex* vertex)
{
    if(vertex && vertex->hessianIndex() >= 0)
    {
        g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;
        computeMarginals(spinv, vertex);
        if(vertex->hessianIndex() >= spinv.blockCols().size())
            return false;
        Eigen::MatrixXd* vertex_cov = spinv.blockCols()[vertex->hessianIndex()].at(vertex->hessianIndex());
        if(!vertex_cov)
            return false;
        covariance = Matrix6d(*vertex_cov);
        return true;
    }
    return false;
}

envire::TransformWithUncertainty ExtendedSparseOptimizer::getEnvireTransformWithUncertainty(const g2o::VertexSE3* vertex)
{
    envire::TransformWithUncertainty transform = envire::TransformWithUncertainty::Identity();
    transform.setTransform(Eigen::Affine3d(vertex->estimate().matrix()));
    Matrix6d covariance;
    if(getVertexCovariance(covariance, vertex))
        transform.setCovariance(switchEnvireG2oCov(covariance));
    return transform;
}

bool ExtendedSparseOptimizer::adjustOdometryPose(const base::samples::RigidBodyState& odometry_pose, base::samples::RigidBodyState& adjusted_odometry_pose) const
{
    if(!last_vertex)
        return false;
    
    Eigen::Isometry3d adjusted_pose = last_vertex->estimate() * (odometry_pose_last_vertex.inverse() * Eigen::Isometry3d(odometry_pose.getTransform().matrix()));
    adjusted_odometry_pose.initUnknown();
    adjusted_odometry_pose.position = adjusted_pose.translation();
    adjusted_odometry_pose.orientation = Eigen::Quaterniond(adjusted_pose.linear());
    // TODO handle also covariance
    
    return true;
}
    
}
#include <iostream>
#include <cmath>

#include <boost/math/distributions/normal.hpp>

#include <graph_slam/vertex_se3_gicp.hpp>
#include <graph_slam/edge_se3_gicp.hpp>
#include <graph_slam/pointcloud_helper.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <envire/maps/Pointcloud.hpp>

int main()
{
    // init laser2robot parameter
    Eigen::Isometry3d laserInRobot = Eigen::Isometry3d::Identity();
    laserInRobot.translation() = Eigen::Vector3d(0.0, 0.0, 2.0);
    g2o::ParameterSE3Offset* laserInRobotParameter = new g2o::ParameterSE3Offset();
    laserInRobotParameter->setOffset(laserInRobot);
    laserInRobotParameter->setId(50);
    
    // creating the optimization problem
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 6> >  SlamBlockSolver;
    typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    //typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    //typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    
    // allocating the optimizer
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    //linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.addParameter(laserInRobotParameter);

    unsigned num_poses = 10;
    
    // create reference pointcloud
    std::vector<Eigen::Vector3d> ref_pointcloud;
    ref_pointcloud.push_back(Eigen::Vector3d(-1.0,0.0,0.0));
    ref_pointcloud.push_back(Eigen::Vector3d(-1.0,1.0,0.0));
    ref_pointcloud.push_back(Eigen::Vector3d(-1.0,1.0,1.0));
    ref_pointcloud.push_back(Eigen::Vector3d(0.0,1.0,0.0));
    
    // add poses
    Eigen::Quaterniond true_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d true_translation = Eigen::Vector3d(1.0, 0.0, 0.0);
    double translation_sigma = 0.1;
    double rotation_sigma = 0.05;
    boost::math::normal_distribution<double> translation_noise(0.0, translation_sigma);
    boost::math::normal_distribution<double> rotation_noise(0.0, rotation_sigma);
    Eigen::Isometry3d vertex_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d true_vertex_pose = Eigen::Isometry3d::Identity();
    for(int i = 0; i < num_poses; i++)
    {
        // add vertex
        graph_slam::VertexSE3_GICP* vertex = new graph_slam::VertexSE3_GICP();
        vertex->setId(i);
        vertex->setEstimate(vertex_pose);
        envire::Pointcloud* point_cloud = new envire::Pointcloud();
        graph_slam::transformPointCloud(ref_pointcloud, point_cloud->vertices, true_vertex_pose);
        vertex->attachPointCloud(point_cloud);
        optimizer.addVertex(vertex);
        
        // create new vertex pose with noise
        Eigen::Isometry3d vertex_delta;
        vertex_delta.setIdentity();
        vertex_delta.rotate(true_rotation * (Eigen::AngleAxisd(rotation_noise.standard_deviation(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rotation_noise.standard_deviation(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.02+rotation_noise.standard_deviation(), Eigen::Vector3d::UnitZ())));
        vertex_delta.translation() = true_translation + Eigen::Vector3d(translation_noise.standard_deviation(), translation_noise.standard_deviation(), translation_noise.standard_deviation());
        vertex_pose = vertex_pose * vertex_delta;
        
        // create new vertex pose without noise
        Eigen::Isometry3d true_vertex_delta;
        true_vertex_delta.setIdentity();
        true_vertex_delta.rotate(true_rotation);
        true_vertex_delta.translation() = true_translation;
        true_vertex_pose = true_vertex_pose * true_vertex_delta;
    }
    
    // add odometry constraints
    Eigen::Matrix<double, 6, 6> edge_cov;
    edge_cov.setZero();
    for(int i = 0; i < 3; i++)
    {
        edge_cov(i,i) = translation_sigma * translation_sigma;
    }
    for(int i = 3; i < 6; i++)
    {
        edge_cov(i,i) = rotation_sigma * rotation_sigma;
    }
    edge_cov.normalize();
    Eigen::Matrix<double, 6, 6> acc_edge_cov;
    acc_edge_cov.setZero();
    for(int i = 0; i < num_poses-1; i++)
    {
        graph_slam::VertexSE3_GICP *from = static_cast<graph_slam::VertexSE3_GICP*>(optimizer.vertex(i));
        graph_slam::VertexSE3_GICP *to   = static_cast<graph_slam::VertexSE3_GICP*>(optimizer.vertex(i+1));
        Eigen::Isometry3d nextPoseInCurrentPose = from->estimate().inverse() * to->estimate();
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        edge->setMeasurement(nextPoseInCurrentPose);
        acc_edge_cov += edge_cov;
        edge->setInformation(acc_edge_cov.inverse());
        edge->vertices()[0] = from;
        edge->vertices()[1] = to;
        optimizer.addEdge(edge);
    }
    
    // add gicp constraints
    for(int i = 0; i < num_poses-1; i++)
    {
        graph_slam::VertexSE3_GICP *from = static_cast<graph_slam::VertexSE3_GICP*>(optimizer.vertex(i));
        graph_slam::VertexSE3_GICP *to   = static_cast<graph_slam::VertexSE3_GICP*>(optimizer.vertex(i+1));
        graph_slam::EdgeSE3_GICP* edge = new graph_slam::EdgeSE3_GICP();
        edge->setSourceVertex(from);
        edge->setTargetVertex(to);
        edge->setMeasurementFromState();
        edge->setInformation(10 * Eigen::Matrix<double, 6, 6>::Identity());
        optimizer.addEdge(edge);
    }

    
    optimizer.save("gicp_graph_slam_before.g2o");
    
    // prepare and run the optimization
    // fix the first robot pose to account for gauge freedom
    graph_slam::VertexSE3_GICP* firstRobotPose = dynamic_cast<graph_slam::VertexSE3_GICP*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);
    optimizer.setVerbose(true);

    std::cerr << "Optimizing" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::cerr << "done." << std::endl;

    optimizer.save("gicp_graph_slam_after.g2o");

    // freeing the graph memory
    optimizer.clear();

    // destroy all the singletons
    g2o::Factory::destroy();
    g2o::OptimizationAlgorithmFactory::destroy();
    g2o::HyperGraphActionLibrary::destroy();
    

    return 0;
}

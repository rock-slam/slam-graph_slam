#include "edge_se3_gicp.hpp"
#include <graph_slam/matrix_helper.hpp>
#include <graph_slam/pointcloud_helper.hpp>
#include <pcl/registration/gicp.h>
#include <limits> 

namespace graph_slam
{
    
EdgeSE3_GICP::EdgeSE3_GICP() : EdgeSE3(), run_gicp(true), use_guess_from_state(false), valid_gicp_measurement(false), icp_fitness_score(std::numeric_limits<double>::max())
{
    setGICPConfiguration(GICPConfiguration());
    
    _measurement = Eigen::Isometry3d::Identity();
    _inverseMeasurement = Eigen::Isometry3d::Identity();
    _information = Matrix6d::Zero();
}

void EdgeSE3_GICP::setGICPConfiguration(const GICPConfiguration& gicp_config)
{
    this->gicp_config = gicp_config;
}

bool EdgeSE3_GICP::setMeasurementFromGICP(bool delayed)
{
    if(delayed)
    {
        run_gicp = true;
        return true;
    }
    
    graph_slam::VertexSE3_GICP *source_vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(_vertices[0]);
    graph_slam::VertexSE3_GICP *target_vertex = dynamic_cast<graph_slam::VertexSE3_GICP*>(_vertices[1]);
    
    if(source_vertex == NULL || target_vertex == NULL)
        return false;
    
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;
    PCLPointCloudPtr source_cloud(new PCLPointCloud);
    PCLPointCloudPtr target_cloud(new PCLPointCloud);
    envire::Pointcloud* envire_source_cloud = dynamic_cast<envire::Pointcloud*>(source_vertex->getEnvirePointCloud().get());
    envire::Pointcloud* envire_target_cloud = dynamic_cast<envire::Pointcloud*>(target_vertex->getEnvirePointCloud().get());
    if(envire_source_cloud == NULL || envire_target_cloud == NULL)
        return false;
    
    // compute current transformation guess
    Eigen::Isometry3d transfomation_guess = source_vertex->estimate().inverse() * target_vertex->estimate();
    
    // config gicp
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(gicp_config.max_correspondence_distance);
    icp.setMaximumIterations(gicp_config.maximum_iterations);
    icp.setTransformationEpsilon(gicp_config.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(gicp_config.euclidean_fitness_epsilon);
    icp.setCorrespondenceRandomness(gicp_config.correspondence_randomness);
    icp.setMaximumOptimizerIterations(gicp_config.maximum_optimizer_iterations);
    icp.setRotationEpsilon(gicp_config.rotation_epsilon);

    // set source cloud
    vectorToPCLPointCloud(envire_source_cloud->vertices, *source_cloud.get(), gicp_config.point_cloud_density);
    icp.setInputCloud(source_cloud);
    
    // transform target cloud in the source cloud frame
    std::vector<Eigen::Vector3d> transformed_target_cloud;
    transformPointCloud(envire_target_cloud->vertices, transformed_target_cloud, transfomation_guess);
    
    // set target cloud
    vectorToPCLPointCloud(transformed_target_cloud, *target_cloud.get());
    icp.setInputTarget(target_cloud);
    
    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
    icp.align(cloud_source_registered);
    double fitness_score = icp.getFitnessScore();
    
    if(icp.hasConverged() && fitness_score <= gicp_config.max_fitness_score)
    {
        Eigen::Isometry3f transformation(icp.getFinalTransformation());

        // check for nan values
        if(is_nan(transformation.matrix()))
        {
            std::cerr << "Messurement from ICP contains not numerical values." << std::endl;
            return false;
        }
        
        _measurement = Eigen::Isometry3d(transformation).inverse() * transfomation_guess;
        _inverseMeasurement = _measurement.inverse();
        _information = ((fitness_score / gicp_config.max_fitness_score) * combineToPoseCovariance(0.1*Eigen::Matrix3d::Identity(), 0.005*Eigen::Matrix3d::Identity())).inverse();
        
        valid_gicp_measurement = true;
        icp_fitness_score = fitness_score;
    }
    
    run_gicp = false;
    return true;
}

void EdgeSE3_GICP::computeError()
{
    if(run_gicp)
        setMeasurementFromGICP();
    EdgeSE3::computeError();
}

void EdgeSE3_GICP::initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to)
{
    if(run_gicp)
        setMeasurementFromGICP();
    EdgeSE3::initialEstimate(from, to);
}

void EdgeSE3_GICP::linearizeOplus()
{
    if(run_gicp)
        setMeasurementFromGICP();
    EdgeSE3::linearizeOplus();
}

void EdgeSE3_GICP::setSourceVertex(VertexSE3_GICP* source)
{
    _vertices[0]=source;
}

void EdgeSE3_GICP::setTargetVertex(VertexSE3_GICP* target)
{
    _vertices[1]=target;
}

}
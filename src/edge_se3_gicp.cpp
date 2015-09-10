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
    
    if(!source_vertex->hasPointcloudAttached() || !target_vertex->hasPointcloudAttached())
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
    icp.setInputCloud(source_vertex->getPCLPointCloud());

    // set target cloud
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;
    PCLPointCloudPtr transformed_target_cloud(new PCLPointCloud);
    pcl::transformPointCloud(*target_vertex->getPCLPointCloud(), *transformed_target_cloud, Eigen::Affine3d(transfomation_guess));
    icp.setInputTarget(transformed_target_cloud);
    
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
	
	// TODO use sampled gicp based covariance per default
	Eigen::Matrix3d translation_cov = 0.1 * Eigen::Matrix3d::Identity();
	translation_cov(2,2) = 0.01;
        _information = (combineToPoseCovariance(translation_cov, 0.005*Eigen::Matrix3d::Identity())).inverse();
        
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
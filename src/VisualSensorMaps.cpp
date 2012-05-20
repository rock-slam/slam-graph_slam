#include "VisualSensorMaps.hpp"
#include <stereo/sparse_stereo.hpp>
#include <envire/icpConfigurationTypes.hpp>
#include <envire/ransac.hpp>

namespace graph_slam
{
VisualSensorMaps::VisualSensorMaps()
    : stereoMap(NULL), 
    sparseMap(NULL), 
    min_sparse_correspondences( 7 )
{
}

VisualSensorMaps::VisualSensorMaps( envire::FrameNode* fn )
    : stereoMap(NULL), 
    sparseMap(NULL), 
    min_sparse_correspondences( 7 )
{
    setFrameNode( fn );
    updateMaps();
}

void VisualSensorMaps::updateMaps()
{
    // go through all the maps in the framenode and see if they fit a sensor map 
    std::list<envire::CartesianMap*> la = frameNode->getMaps();
    for( std::list<envire::CartesianMap*>::iterator it = la.begin();
	    it != la.end(); it ++ )
    {
	if( (*it)->getLabel() == "dense" )
	    stereoMap = dynamic_cast<envire::Pointcloud*>( *it );
	else if( (*it)->getLabel() == "sparse" )
	    sparseMap = dynamic_cast<envire::Featurecloud*>( *it );
    }
}

void VisualSensorMaps::updateExtents()
{
    // get the extents from the individual maps first
    if( extents.isEmpty() )
    {
	// cache the extends, since they won't change 
	// locally
	if( stereoMap )
	    extents.extend( stereoMap->getExtents() );
	if( sparseMap )
	    extents.extend( sparseMap->getExtents() );
    }
}

void VisualSensorMaps::associateStereoMap( envire::Pointcloud* pc1, envire::Pointcloud* pc2, std::vector<envire::TransformWithUncertainty>& constraints ) 
{
    return;

    // perform an icp fitting of the two pointclouds
    // given a currently static parameter set
    envire::icp::ICPConfiguration conf;
    conf.model_density = 0.01;
    conf.measurement_density = 0.01;
    conf.max_iterations = 10;
    conf.min_mse = 0.005;
    conf.min_mse_diff = 0.0001;
    conf.overlap = 0.7;

    // use the envire ICP implementation
    // could think about using the PCL registration framework
    envire::icp::TrimmedKD icp;
    icp.addToModel( envire::icp::PointcloudAdapter( pc1, conf.model_density ) );
    icp.align( envire::icp::PointcloudAdapter( pc2, conf.measurement_density ),  
	    conf.max_iterations, conf.min_mse, conf.min_mse_diff, conf.overlap );

    // come up with a covariance here
    // TODO replace with calculated covariance values 
    const double trans_error = 0.1;
    const double rot_error = 10.0/180.0*M_PI;

    Eigen::Matrix<double,6,1> cov_diag;
    cov_diag << Eigen::Vector3d::Ones() * rot_error, 
	     Eigen::Vector3d::Ones() * trans_error;

    Eigen::Matrix<double,6,6> cov = 
	cov_diag.array().square().matrix().asDiagonal();

    Eigen::Affine3d bodyBtoBodyA = 
	pc2->getFrameNode()->relativeTransform( pc1->getFrameNode() );

    constraints.push_back( envire::TransformWithUncertainty( bodyBtoBodyA, cov ) );
}

size_t VisualSensorMaps::associateSparseMap( envire::Featurecloud *fc1, envire::Featurecloud *fc2, std::vector<envire::TransformWithUncertainty>& constraints )
{
    stereo::StereoFeatures f;
    stereo::FeatureConfiguration config;
    config.isometryFilterThreshold = 1.5;
    config.distanceFactor = 1.5;
    config.isometryFilterMaxSteps = 1000;
    f.setConfiguration( config );

    f.calculateInterFrameCorrespondences( fc1, fc2, stereo::FILTER_ISOMETRY );
    std::vector<std::pair<long, long> > matches = f.getInterFrameCorrespondences();

    if( f.getInterFrameCorrespondences().size() >= min_sparse_correspondences )
    {
	Eigen::Affine3d bodyBtoBodyA = f.getInterFrameCorrespondenceTransform();

	// come up with a covariance here
	// TODO replace with calculated covariance values 
	const double trans_error = 1.5;
	const double rot_error = 30.0/180.0*M_PI;

	Eigen::Matrix<double,6,1> cov_diag;
	cov_diag << Eigen::Vector3d::Ones() * rot_error, 
		 Eigen::Vector3d::Ones() * trans_error;

	Eigen::Matrix<double,6,6> cov = 
	    cov_diag.array().square().matrix().asDiagonal();

	constraints.push_back( envire::TransformWithUncertainty( bodyBtoBodyA, cov ) );
    }

    return f.getInterFrameCorrespondences().size();
}

void VisualSensorMaps::associate( SensorMaps *maps, std::vector<envire::TransformWithUncertainty>& constraints )
{
    VisualSensorMaps *sma = this, *smb = dynamic_cast<VisualSensorMaps*>(maps);
    assert( smb );

    // call the individual association methods
    if( sma->sparseMap && smb->sparseMap )
    {
	if( associateSparseMap( sma->sparseMap, smb->sparseMap, constraints ) >= min_sparse_correspondences )
	{
	    if( sma->stereoMap && smb->stereoMap )
		associateStereoMap( sma->stereoMap, smb->stereoMap, constraints );
	}
    }
    else if( sma->stereoMap && smb->stereoMap )
    {
	associateStereoMap( sma->stereoMap, smb->stereoMap, constraints );
    }
    // TODO store both successful and unsuccessful associations
    // since it doesn't make sense to try to associate twice
}


}

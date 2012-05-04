#include "VisualPoseGraph.hpp"
#include "VisualSensorMaps.hpp"

namespace graph_slam
{

VisualPoseGraph::VisualPoseGraph( envire::Environment* env, int num_levels, int node_distance ) 
    : PoseGraph( env, num_levels, node_distance ),
    min_sparse_correspondences( 7 )
{
    // set-up body frame
    bodyFrame = new envire::FrameNode();
    env->addChild( env->getRootNode(), bodyFrame.get() );

    // set-up chain for distance images
    distFrame = new envire::FrameNode();
    env->addChild( bodyFrame.get(), distFrame.get() );

    distOp = new envire::DistanceGridToPointcloud();
    distOp->setMaxDistance( 5.0 );
    env->attachItem( distOp.get() );

    // set-up the chain for the feature clouds
    featureFrame = new envire::FrameNode();
    env->addChild( bodyFrame.get(), featureFrame.get() );
    featurecloud = new envire::Featurecloud();
    env->setFrameNode( featurecloud.get(), featureFrame.get() );

    // set-up the target mls
    const double cell_size = 0.2;
    const double grid_size = 150;
    mlsGrid = new envire::MLSGrid( 
	    grid_size / cell_size, grid_size / cell_size, 
	    cell_size, cell_size, -grid_size / 2.0, -grid_size / 2.0 );
    mlsGrid->setHorizontalPatchThickness( 0.5 );
    mlsGrid->setGapSize( 1.0 );
    env->setFrameNode( mlsGrid.get(), env->getRootNode() );
    mlsOp = new envire::MLSProjection();
    mlsOp->useUncertainty( false );
    env->attachItem( mlsOp.get() );
    mlsOp->addOutput( mlsGrid.get() );
}

SensorMaps* VisualPoseGraph::createSensorMaps( envire::FrameNode* fn )
{
    return new VisualSensorMaps( fn );
}

void VisualPoseGraph::initNode( const envire::TransformWithUncertainty &body2bodyPrev, const envire::TransformWithUncertainty &body2world )
{
    // handle relative poses
    initNode( body2bodyPrev );

    // create a copy of the body2world transform, and set the covariance
    // for the position very large
    envire::TransformWithUncertainty body2root = body2world;
    Eigen::Matrix<double,6,6> cov = body2root.getCovariance();
    cov.bottomRightCorner<3,3>() = Eigen::Vector3d( 1e8, 1e8, 1e8 ).asDiagonal();
    body2root.setCovariance( cov );

    // add the edget constraint to the root node
    // this assumes the root node is already registered
    addConstraint( env->getRootNode(), currentBodyFrame.get(), body2root );
}

/** will prepare a new node based on an initial transformation
*/
void VisualPoseGraph::initNode( const envire::TransformWithUncertainty &body2bodyPrev )
{
    // apply the relative transform to the current bodyFrame position
    bodyFrame->setTransform( bodyFrame->getTransformWithUncertainty() *
	    body2bodyPrev );

    // copy the bodyFrame node, and use it to store any additional map information
    // that is added through the other add functions
    currentBodyFrame = new envire::FrameNode(
	    bodyFrame->getTransformWithUncertainty() );

    env->addChild( bodyFrame->getParent(), currentBodyFrame.get() );

    // register the new node with the graph
    addNode( currentBodyFrame.get() );

    // in case there is a previous node create a vertex based on odometry
    // information
    if( prevBodyFrame )
    {
	addConstraint( prevBodyFrame.get(), currentBodyFrame.get(), body2bodyPrev );
    }
}

/** adds a sensor reading for a distance image to an initialized node
*/
void VisualPoseGraph::addSensorReading( const base::samples::DistanceImage& distImage, const Eigen::Affine3d& sensor2body, const base::samples::frame::Frame& textureImage )
{
    // configure the processing chain for the distance image
    assert( currentBodyFrame );

    // create new distance grid object
    distFrame->setTransform( sensor2body );
    if( !distGrid )
    {
	// create new grid using the parameters from the distance image
	distGrid = new envire::DistanceGrid( distImage );

	// distGrid has just been created and needs to be attached
	distOp->addInput( distGrid.get() );
	distGrid->setFrameNode( distFrame.get() );
    }
    distGrid->copyFromDistanceImage( distImage );

    // create new imagegrid object if not available
    if( !textureGrid )
    {
	// copy the scaling properties from distanceImage
	// TODO this is a hack!
	textureGrid = new envire::ImageRGB24( 
		distImage.width, distImage.height, 
		distImage.scale_x, distImage.scale_y, 
		distImage.center_x, distImage.center_y );
	env->attachItem( textureGrid.get() );

	distOp->addInput( textureGrid.get() );
	textureGrid->setFrameNode( distFrame.get() );
    }
    textureGrid->copyFromFrame( textureImage );

    envire::Pointcloud::Ptr distPc = new envire::Pointcloud();
    env->setFrameNode( distPc.get(), currentBodyFrame.get() );
    distPc->setLabel("dense");
    distOp->removeOutputs();
    distOp->addOutput( distPc.get() );

    distOp->updateAll();

    mlsOp->addInput( distPc.get() );

    /*
       envire::GraphViz gv;
       gv.writeToFile( env, "/tmp/gv.dot" );
       */
}	

/** adds a sensor reading for a feature array to an initialized node
*/
void VisualPoseGraph::addSensorReading( const stereo::StereoFeatureArray& featureArray, const Eigen::Affine3d& sensor2body )
{
    // configure the processing chain for the feature image 
    assert( currentBodyFrame );
    featureFrame->setTransform( sensor2body );
    // copy only up to a certain distance to omit the features further out
    featureArray.copyTo( *featurecloud.get(), 5.0 );

    envire::Featurecloud::Ptr featurePc = new envire::Featurecloud();
    env->setFrameNode( featurePc.get(), currentBodyFrame.get() );
    featurePc->setLabel("sparse");

    featurePc->copyFrom( featurecloud.get() );
    featurePc->itemModified();
}

/** adds an initialized node, with optional sensor readings to the node graph
 *
 * requires a previous call to initNode(), as well addSensorReading() calls
 * for each sensor reading.
 */
void VisualPoseGraph::addNode()
{
    assert( currentBodyFrame );

    associateNode( currentBodyFrame.get() );
    optimizeNodes();

    mlsGrid->clear();
    mlsOp->updateAll();

    // TODO see if we need to reassociate here 
    bodyFrame->setTransform( currentBodyFrame->getTransform() );
    prevBodyFrame = currentBodyFrame;
    currentBodyFrame = NULL;
}

}

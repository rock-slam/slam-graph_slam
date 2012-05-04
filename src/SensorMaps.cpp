#include "SensorMaps.hpp"

namespace graph_slam
{

SensorMaps::~SensorMaps()
{
}

void SensorMaps::setFrameNode( envire::FrameNode* a )
{
    this->frameNode = a;
}

void SensorMaps::update()
{
    updateExtents();
    updateBounds();
}

// update the bounds of the map using the uncertainty 
// associated with the framenode 
void SensorMaps::updateBounds( double sigma )
{
    // the strategy is now to take the corner points, 
    // and transform them including the uncertainty 
    // provided. These points should all be included
    // in the final bounds and should roughly provide
    // the bounding box for the map including uncertainty

    // reset the bounds
    bounds.setEmpty();

    // go through all 8 corners of the extents
    for( int i=0; i<8; i++ )
    {
	Eigen::Vector3d corner;
	for( int j=0; j<3; j++ )
	    corner[j] = (i>>j)&1 ? extents.min()[j] : extents.max()[j];

	// Transform the points with uncertainty
	envire::PointWithUncertainty uncertain_corner = 
	    frameNode->getTransformWithUncertainty() * envire::PointWithUncertainty(corner);

	// do a cholesky decomposition of the covariance matrix
	// in order to get the sigma points
	Eigen::LLT<Eigen::Matrix3d> llt;
	llt.compute( uncertain_corner.getCovariance() );
	Eigen::Matrix3d sigma_points = llt.matrixL();

	// extend the bounds by the sigma points
	for( int j=0; j<3; j++ )
	{
	    bounds.extend( 
		    uncertain_corner.getPoint() + sigma_points.col(j) );
	    bounds.extend( 
		    uncertain_corner.getPoint() - sigma_points.col(j) );
	}
    }
}
}

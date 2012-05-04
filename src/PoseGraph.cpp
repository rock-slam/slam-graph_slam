#include "PoseGraph.hpp"
#include <base/logging.h>

namespace graph_slam 
{

Transformation3 eigen2Hogman( const Eigen::Affine3d& eigen_transform )
{
    Eigen::Quaternionf eigen_quat(eigen_transform.rotation().cast<float>());
    Eigen::Matrix4d eigen_mat( eigen_transform.matrix() );
    Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
    Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
	    eigen_quat.w());
    Transformation3 result(translation, rotation);

    return result;
}

Matrix6 eigen2Hogman( const Eigen::Matrix<double,6,6>& eigen_matrix )
{
    Matrix6 result;
    // TODO might be more efficient to perform a memcopy here
    for( int m=0; m<6; m++ )
	for( int n=0; n<6; n++ )
	    result[m][n] = eigen_matrix(m,n);

    return result;
}

Eigen::Matrix<double,6,6> hogman2Eigen( const Matrix6& hogman )
{
    Eigen::Matrix<double,6,6> result;

    for( int m=0; m<6; m++ )
	for( int n=0; n<6; n++ )
	    result(m,n) = hogman[m][n];

    return result;
}

Eigen::Affine3d hogman2Eigen( const Transformation3& hogman_transform )
{
    Eigen::Quaterniond rotation( 
	    hogman_transform.rotation().w(),
	    hogman_transform.rotation().x(),
	    hogman_transform.rotation().y(),
	    hogman_transform.rotation().z() );

    Eigen::Translation3d translation(
	    hogman_transform.translation().x(),
	    hogman_transform.translation().y(),
	    hogman_transform.translation().z() );

    return translation * rotation;
}

/** 
 * transform a 6x6 covariance matrix in [r t] order to a hogman 6 inverse
 * covariance matrix in [t r] format.
 */
Matrix6 envireCov2HogmanInf( const Eigen::Matrix<double,6,6>& eigen_matrix )
{
    Eigen::Matrix<double,6,6> t;
    t << eigen_matrix.bottomRightCorner<3,3>(), eigen_matrix.bottomLeftCorner<3,3>(),
      eigen_matrix.topRightCorner<3,3>(), eigen_matrix.topLeftCorner<3,3>();

    return eigen2Hogman( Eigen::Matrix<double,6,6>(t.inverse()) );
}

/** 
 * Transform a 6x6 covariance matrix in [t r] order from hogman Covariance
 * format to the envire [r t] order.
 */
Eigen::Matrix<double,6,6> hogmanCov2EnvireCov( const Matrix6& hogman_matrix )
{
    Eigen::Matrix<double,6,6> t = hogman2Eigen( hogman_matrix );
    Eigen::Matrix<double,6,6> t1;
    t1 << t.bottomRightCorner<3,3>(), t.bottomLeftCorner<3,3>(),
      t.topRightCorner<3,3>(), t.topLeftCorner<3,3>();
    return t1;
}

PoseGraph::PoseGraph( envire::Environment* env, int num_levels, int node_distance ) 
    : max_node_radius( 25.0 ),
    env( env ), optimizer( new AISNavigation::HCholOptimizer3D( num_levels, node_distance ) ) 
{
}

PoseGraph::~PoseGraph()
{
    // do some cleanup
    delete optimizer;

    // delete the nodeMap objects
    for( std::map<string, SensorMaps*>::iterator it = nodeMap.begin(); it != nodeMap.end(); it++ )
	delete it->second;
}

void PoseGraph::addNode( envire::FrameNode* fn, const envire::TransformWithUncertainty& transform )
{
    // get the vertex id
    long id = fn->getUniqueIdNumericalSuffix();

    // make sure the node is not already registered
    AISNavigation::PoseGraph3D::Vertex 
	*v = optimizer->vertex( id );
    assert( !v );

    // create a sensor maps object
    getSensorMaps( fn );

    // create a new hogman vertex for this node
    optimizer->addVertex( 
	    id,
	    eigen2Hogman( transform.getTransform() ),
	    envireCov2HogmanInf( transform.getCovariance() ) );
}

void PoseGraph::addNode( envire::FrameNode* fn )
{
    addNode( fn, fn->getTransformWithUncertainty() );
}

void PoseGraph::addConstraint( const envire::FrameNode* fn1, const envire::FrameNode* fn2, const envire::TransformWithUncertainty& transform )
{
    AISNavigation::PoseGraph3D::Vertex 
	*v1 = optimizer->vertex( fn1->getUniqueIdNumericalSuffix() ),
	*v2 = optimizer->vertex( fn2->getUniqueIdNumericalSuffix() );

    assert( v1 && v2 );

    optimizer->addEdge( 
	    v1,
	    v2,
	    eigen2Hogman( transform.getTransform() ),
	    envireCov2HogmanInf( transform.getCovariance() )
	    );
}

void PoseGraph::associateNode( envire::FrameNode* update_fn )
{
    for( std::map<std::string, SensorMaps*>::iterator it = nodeMap.begin();
	    it != nodeMap.end(); it++ )
    {
	envire::FrameNode *fn = it->second->frameNode.get();
	// don't associate with self
	if( update_fn != fn )
	{
	    bool result = associateNodes( fn, update_fn ); 

	    LOG_INFO_S << "associate node " 
		<< fn->getUniqueIdNumericalSuffix() 
		<< " with " 
		<< update_fn->getUniqueIdNumericalSuffix() << "... "
		<< (result ? "match" : "no match");
	}
    }
}

void PoseGraph::optimizeNodes( int iterations )
{
    // perform the graph optimization
    optimizer->optimize( iterations, true );

    // write the poses back to the environment
    // for now, write all the poses back, could add an updated flag
    for( std::map<std::string, SensorMaps*>::iterator it = nodeMap.begin();
	    it != nodeMap.end(); it++ )
    {
	SensorMaps *sm = it->second;
	AISNavigation::PoseGraph3D::Vertex 
	    *vertex = optimizer->vertex( sm->vertexId );

	// get pose with uncertainty from Hogman
	envire::TransformWithUncertainty tu( 
		hogman2Eigen( vertex->transformation ), 
		hogmanCov2EnvireCov( vertex->covariance ) );

	envire::FrameNode::Ptr fn = sm->frameNode; 
	fn->setTransform( tu );

	// update the bounds 
	// TODO this could be optimized, as it may be to expensive to 
	// update the bounds everytime we have a small change in position
	sm->updateBounds();
    }
}


/** will return a sensormaps structure for a given 
 * framenode. creates a new one, if not already existing.
 */
SensorMaps* PoseGraph::getSensorMaps( envire::FrameNode* fn )
{
    std::string id = fn->getUniqueId();

    // see if we can return a cached object
    std::map<std::string, SensorMaps*>::iterator 
	f = nodeMap.find( id );

    if( f != nodeMap.end() )
	return f->second;

    // otherwise create a new node
    SensorMaps* sm = createSensorMaps( fn );
    sm->vertexId = fn->getUniqueIdNumericalSuffix();
    nodeMap.insert( make_pair( id, sm ) );

    // call update bounds once, so we have an initial
    // idea of the bounds
    sm->updateBounds();

    return sm;
}

/** 
 * associate two framenodes, if they are within a feasable distance
 * between each other and have overlapping bounding boxes.
 *
 * @return true if an association has been added
 */ 
bool PoseGraph::associateNodes( envire::FrameNode* a, envire::FrameNode* b )
{
    // discard if distance between is too high
    // TODO: this is potentially dangerous as it doesn't take the
    // uncertainty into account... see how to make this safer, but still
    // fast.
    if( (a->getTransform().translation() - b->getTransform().translation()).norm() > max_node_radius )
	return false;

    // get the sensor map objects for both frameNodes
    SensorMaps 
	*sma = getSensorMaps( a ), 
	*smb = getSensorMaps( b );

    // check if the bounding boxes have a common intersection
    // and return false if not
    if( sma->bounds.intersection( smb->bounds ).isEmpty() )
	return false;

    std::vector<envire::TransformWithUncertainty> constraints;
    sma->associate( smb, constraints );
    for( size_t i = 0; i < constraints.size(); i++ )
    {
	// add the egde to the optimization framework 
	// this will update an existing edge
	optimizer->addEdge( 
		optimizer->vertex( a->getUniqueIdNumericalSuffix() ),
		optimizer->vertex( b->getUniqueIdNumericalSuffix() ),
		eigen2Hogman( constraints[i].getTransform() ),
		envireCov2HogmanInf( constraints[i].getCovariance() )
		);
    }

    // TODO store both successful and unsuccessful associations
    // since it doesn't make sense to try to associate twice
    return !constraints.empty();
}

}

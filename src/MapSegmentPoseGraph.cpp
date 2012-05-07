#include "MapSegmentPoseGraph.hpp"

namespace graph_slam
{
    class MapSegmentSensorMaps : public SensorMaps
    {
    public:
	MapSegmentSensorMaps()
	    : segment( NULL )
	{
	};

	envire::MapSegment *segment;

	void updateExtents()
	{
	    if( extents.isEmpty() )
	    {
		extents.extend( segment->getExtents() );
	    }
	};

	void associate( SensorMaps *maps, std::vector<envire::TransformWithUncertainty>& constraints )
	{
	    // TODO
	}
    };

    MapSegmentPoseGraph::MapSegmentPoseGraph( envire::Environment* env )
	: PoseGraph( env )
    {
    }

    void MapSegmentPoseGraph::addSegment( envire::MapSegment* segment )
    {
	// update the pose information based on the parts of the segment
	segment->update();

	// register new segment with posegraph
	// TODO check if inital transform is ok
	addNode( segment->getFrameNode() );

	// associate with previous segment if available
	if( !segments.empty() )
	{
	    envire::MapSegment *prevSegment = segments.back().get();
	    addConstraint( 
		    prevSegment->getFrameNode(), 
		    segment->getFrameNode(), 
		    prevSegment->getTransform() );
	}

	// and add this segment to the list
	segments.push_back( segment );

	// now try to associate segments with other segments in the graph
	associateNode( segment->getFrameNode() );
    }

    void MapSegmentPoseGraph::update()
    {
	// run optimization on the nodes
	// TODO make iterations configurable
	optimizeNodes( 5 );

	// in case the mergeOperator is set
	if( mergeOperator )
	{
	    // clear inputs first
	    env->removeInputs( mergeOperator.get() );

	    Eigen::Affine3d prevPose;
	    for( size_t i=0; i<segments.size(); i++ )
	    {
		Eigen::Affine3d pose = 
		    segments[i]->getFrameNode()->getTransform();

		if( i > 0 )
		{
		    // get relative transform between poses
		    Eigen::Affine3d pose2prevPose = 
			prevPose.inverse() * pose;

		    // get the map, which matches that transform the 
		    // closest
		    envire::CartesianMap* map = 
			segments[i-1]->getMapForPose( pose2prevPose );

		    // and add as input
		    mergeOperator->addInput( map );

		    // TODO check if the FrameNode of the map is 
		    // the right one
		}

		prevPose = pose;
	    }

	    // for the last segment, pick the one with the heighest weight
	    if( !segments.empty() )
	    {
		envire::CartesianMap* map = 
		    segments.back()->getBestMap();
		mergeOperator->addInput( map ); 
	    }
	}
    }

    void MapSegmentPoseGraph::setMergeOperator( envire::Operator* op )
    {
	mergeOperator = op;
    }

    SensorMaps* MapSegmentPoseGraph::createSensorMaps( envire::FrameNode* fn )
    {
	MapSegmentSensorMaps *sm = new MapSegmentSensorMaps();
	std::list<envire::CartesianMap*> maps =
	    fn->getMaps();
	for( std::list<envire::CartesianMap*>::iterator it = maps.begin();
		it != maps.end(); it++ )
	{
	    envire::MapSegment *segment = dynamic_cast<envire::MapSegment*>(*it); 
	    if( segment )
		sm->segment = segment;
	}
	if( !sm->segment )
	    throw std::runtime_error("FrameNode does not have a MapSegment map.");

	sm->setFrameNode( fn );
	return sm;
    }
}

#include "MapSegmentPoseGraph.hpp"
#include "VisualSensorMaps.hpp"

#include <envire/maps/MLSGrid.hpp>

namespace graph_slam
{
    void mergeMap( 
	    envire::Operator* mergeOperator,
	    envire::Layer* map_,
	    envire::FrameNode* fn,
	    double z_error = 0.0,
	    size_t num_steps = 0 )
    {
	envire::MLSGrid* map = dynamic_cast<envire::MLSGrid*>( map_ );
	assert( map );

	envire::Environment *env = fn->getEnvironment();
	envire::MLSGrid* grid = map->clone();

	// attach to framenode and operator
	env->setFrameNode( grid, fn );
	mergeOperator->addInput( grid );

	if( num_steps > 0 )
	{
	    // perform z-height correction
	    // on all grid cells
	    for(size_t m=0;m<grid->getWidth();m++)
	    {
		for(size_t n=0;n<grid->getHeight();n++)
		{
		    for( envire::MLSGrid::iterator cit = grid->beginCell(m,n); cit != grid->endCell(); cit++ )
		    {
			envire::MLSGrid::SurfacePatch p( *cit );
			size_t uidx = p.update_idx;
			// linear distribution of error
			double error = z_error * (double)uidx / (double)num_steps;
			p.mean -= error;
		    }
		}
	    }
	}

	grid->itemModified();
    }

    class MapSegmentSensorMaps : public SensorMaps
    {
	VisualSensorMaps vm;

    public:
	MapSegmentSensorMaps() :
	    segment( NULL )
	{
	};

	void setFrameNode( envire::FrameNode *fn )
	{
	    SensorMaps::setFrameNode( fn );
	    vm.setFrameNode( fn );
	    vm.updateMaps();
	}

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
	    MapSegmentSensorMaps  /*  *sma = this, */ *smb = dynamic_cast<MapSegmentSensorMaps*>(maps);
	    assert( smb );
	    // TODO

	    // for now just use visual association
	    vm.associate( &smb->vm, constraints );
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
	optimizeNodes( 15 );

	// in case the mergeOperator is set
	if( mergeOperator )
	{
	    // clear inputs first
	    std::list<envire::Layer*> inputs =
		env->getInputs( mergeOperator.get() );
	    for( std::list<envire::Layer*>::iterator it = inputs.begin(); it != inputs.end(); it++ )
		(*it)->detach();

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
		    base::Affine3d map_pose;
		    size_t traj_size;
		    envire::CartesianMap* map = 
			segments[i-1]->getMapForPose( pose2prevPose, map_pose, traj_size );

		    // and add as input
		    //mergeOperator->addInput( map );
		    mergeMap( 
			    mergeOperator.get(),
			    map,
			    segments[i-1]->getFrameNode(),
			    pose2prevPose.translation().z() - map_pose.translation().z(),
			    traj_size );

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
		//mergeOperator->addInput( map ); 
		mergeMap( 
			mergeOperator.get(),
			map,
			segments.back()->getFrameNode() );
	    }

	    mergeOperator->updateAll();
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

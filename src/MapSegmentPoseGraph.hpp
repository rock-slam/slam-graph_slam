#ifndef GRAPH_SLAM_MAP_SEGMENT_POSE_GRAPH__
#define GRAPH_SLAM_MAP_SEGMENT_POSE_GRAPH__

#include <graph_slam/PoseGraph.hpp>
#include <envire/maps/MapSegment.hpp>

namespace graph_slam
{

class MapSegmentPoseGraph : public PoseGraph
{
public:
    MapSegmentPoseGraph( envire::Environment* env );

    /** @brief register a new map segment
     */
    void addSegment( envire::MapSegment* segment );

    /** @brief run optimization and apply to environment
     *
     * in case the merge operator is set, the parts of the map segments which
     * have been selected will be added as inputs to the merge operator.
     */
    void update();

    /** @brief set the merge operator
     *
     * when this operator is set, the relevant map parts from the segment which
     * have been selected in the optimization process will be added as inputs.
     */
    void setMergeOperator( envire::Operator* op );

protected:
    std::vector<envire::MapSegment::Ptr> segments; 
    SensorMaps* createSensorMaps( envire::FrameNode* fn );

    envire::Operator::Ptr mergeOperator;
};

}
#endif

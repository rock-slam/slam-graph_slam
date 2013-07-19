#ifndef GRAPH_SLAM_VERTEX_SE3_GICP_H
#define GRAPH_SLAM_VERTEX_SE3_GICP_H

#include <g2o/types/slam3d/vertex_se3.h>
#include <envire/maps/Pointcloud.hpp>
#include <base/samples/rigid_body_state.h>

namespace graph_slam 
{

class VertexSE3_GICP : public g2o::VertexSE3
{
public:
    struct EdgeCandidate
    {
        bool icp_failed;
        double error;
        double mahalanobis_distance;
        EdgeCandidate() : icp_failed(false), error(0.0), mahalanobis_distance(0.0) {};
    };
    
    struct EdgeSearchState
    {
        bool has_run;
        Eigen::Isometry3d pose;
        EdgeSearchState() : has_run(false), pose(Eigen::Matrix4d::Ones() * base::NaN<double>()) {};
    };
    
    typedef std::map<int, EdgeCandidate> EdgeCandidates;
    
    VertexSE3_GICP();
    void attachPointCloud(envire::Pointcloud* point_cloud);
    void detachPointCloud();
    envire::EnvironmentItem::Ptr getEnvirePointCloud() const;
    
    void addEdgeCandidate(int vertex_id, double mahalanobis_distance);
    void removeEdgeCandidate(int vertex_id);
    void updateEdgeCandidate(int vertex_id, bool icp_failed);
    bool getBestEdgeCandidate(EdgeCandidate& candidate, int& vertex_id);
    double getMissingEdgesError() const;
    
    void setEdgeSearchState(bool has_run, const Eigen::Isometry3d& search_pose);
    const EdgeSearchState& getEdgeSearchState();
    
protected:
    envire::EnvironmentItem::Ptr envire_pointcloud;
    EdgeCandidates edge_candidates;
    EdgeSearchState search_state;
    double missing_edges_error;
};


class VertexSE3_GICPWriteGnuplotAction: public g2o::VertexSE3WriteGnuplotAction 
{
public:
    VertexSE3_GICPWriteGnuplotAction() : VertexSE3WriteGnuplotAction ()
    {
        setTypeName(typeid(VertexSE3_GICP).name());
    }
};
#ifdef G2O_HAVE_OPENGL
class VertexSE3_GICPDrawAction: public g2o::VertexSE3DrawAction
{
public:
    VertexSE3_GICPDrawAction() : VertexSE3DrawAction() 
    {
        setTypeName(typeid(VertexSE3_GICP).name());
    }
};
#endif

} // end namespace

#endif
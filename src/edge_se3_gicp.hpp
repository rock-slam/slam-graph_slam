#ifndef GRAPH_SLAM_EDGE_SE3_GICP_H
#define GRAPH_SLAM_EDGE_SE3_GICP_H

#include "vertex_se3_gicp.hpp"

#include <g2o/types/slam3d/edge_se3.h>
#include <pcl/registration/gicp.h>

namespace graph_slam 
{

class EdgeSE3_GICP : public g2o::EdgeSE3
{
public:
    EdgeSE3_GICP();
    
    void computeError();
    
    virtual bool setMeasurementFromState();
    
    void linearizeOplus();
    
    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to);
    
    void setTargetVertex(VertexSE3_GICP* target);
    void setSourceVertex(VertexSE3_GICP* source);
    
    void runGICP();
    
protected:
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    bool run_gicp;
};


class EdgeSE3_GICPWriteGnuplotAction : public g2o::EdgeSE3WriteGnuplotAction 
{
public:
    EdgeSE3_GICPWriteGnuplotAction() : EdgeSE3WriteGnuplotAction() 
    {
        setTypeName(typeid(EdgeSE3_GICP).name());
    }
};
#ifdef G2O_HAVE_OPENGL
class EdgeSE3_GICPDrawAction : public g2o::EdgeSE3DrawAction 
{
public:
    EdgeSE3_GICPDrawAction() : EdgeSE3DrawAction() 
    {
        setTypeName(typeid(EdgeSE3_GICP).name());
    }
};
#endif

} // end namespace

#endif

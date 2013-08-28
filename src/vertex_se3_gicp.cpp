#include "vertex_se3_gicp.hpp"

#include <graph_slam/pointcloud_helper.hpp>

namespace graph_slam
{
    
VertexSE3_GICP::VertexSE3_GICP() : VertexSE3(), missing_edges_error(0.0), pointcloud_attached(false)
{

}

void VertexSE3_GICP::attachPointCloud(envire::Pointcloud* point_cloud)
{
    envire_pointcloud.reset(point_cloud);
    pointcloud_attached = true;
}

void VertexSE3_GICP::detachPointCloud()
{
    envire_pointcloud.reset();
    pointcloud_attached = false;
}

envire::EnvironmentItem::Ptr VertexSE3_GICP::getEnvirePointCloud() const
{
    return envire_pointcloud;
}

bool VertexSE3_GICP::getBestEdgeCandidate(VertexSE3_GICP::EdgeCandidate& candidate, int& vertex_id)
{
    double error = 0.0;
    for(EdgeCandidates::iterator it = edge_candidates.begin(); it != edge_candidates.end(); it++)
    {
        if(!it->second.icp_failed && it->second.error > error)
        {
            error = it->second.error;
            vertex_id = it->first;
        }
    }
    
    if(error <= 0.0)
    {
        missing_edges_error = 0.0;
        return false;
    }
    
    candidate = edge_candidates[vertex_id];
    return true;
}

void VertexSE3_GICP::addEdgeCandidate(int vertex_id, double mahalanobis_distance)
{
    if(mahalanobis_distance < 0.0 || vertex_id == _id)
        return;
    
    double error = 1.0 / (mahalanobis_distance + 1.0);
    
    if(!edge_candidates[vertex_id].icp_failed)
    {
        // add new error
        missing_edges_error += error;
        edge_candidates[vertex_id].error += error;
    }
    else
    {
        edge_candidates[vertex_id].error = error;
    }

    // update mahalanobis distance
    edge_candidates[vertex_id].mahalanobis_distance = mahalanobis_distance;
}

void VertexSE3_GICP::removeEdgeCandidate(int vertex_id)
{
    if(edge_candidates.count(vertex_id))
    {
        missing_edges_error -= edge_candidates[vertex_id].error;
        if(missing_edges_error < 0.0)
            missing_edges_error = 0.0;
        edge_candidates.erase(vertex_id);
    }
}

void VertexSE3_GICP::updateEdgeCandidate(int vertex_id, bool icp_failed)
{
    if(edge_candidates.count(vertex_id))
    {
        if(!edge_candidates[vertex_id].icp_failed && icp_failed)
        {
            // remove current error if icp failed
            missing_edges_error -= edge_candidates[vertex_id].error;
            if(missing_edges_error < 0.0)
                missing_edges_error = 0.0;
        }
        else if(edge_candidates[vertex_id].icp_failed && !icp_failed)
        {
            missing_edges_error += edge_candidates[vertex_id].error;
        }
        
        edge_candidates[vertex_id].icp_failed = icp_failed;
    }
}

double VertexSE3_GICP::getMissingEdgesError() const
{
    return missing_edges_error;
}

const VertexSE3_GICP::EdgeSearchState& VertexSE3_GICP::getEdgeSearchState()
{
    return search_state;
}

void VertexSE3_GICP::setEdgeSearchState(bool has_run, const Eigen::Isometry3d& search_pose)
{
    search_state.has_run = has_run;
    search_state.pose = search_pose;
}

}
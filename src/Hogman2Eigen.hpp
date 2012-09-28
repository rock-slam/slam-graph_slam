#ifndef GRAPH_SLAM_HOGMAN2EIGEN_HPP__
#define GRAPH_SLAM_HOGMAN2EIGEN_HPP__

#include <aislib/graph_optimizer/graph_optimizer3d_hchol.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace graph_slam
{

Transformation3 eigen2Hogman( const Eigen::Affine3d& eigen_transform );

Matrix6 eigen2Hogman( const Eigen::Matrix<double,6,6>& eigen_matrix );

Eigen::Matrix<double,6,6> hogman2Eigen( const Matrix6& hogman );

Eigen::Affine3d hogman2Eigen( const Transformation3& hogman_transform );

/** 
 * transform a 6x6 covariance matrix in [r t] order to a hogman 6 inverse
 * covariance matrix in [t r] format.
 */
Matrix6 envireCov2HogmanInf( const Eigen::Matrix<double,6,6>& eigen_matrix );

/** 
 * Transform a 6x6 covariance matrix in [t r] order from hogman Covariance
 * format to the envire [r t] order.
 */
Eigen::Matrix<double,6,6> hogmanCov2EnvireCov( const Matrix6& hogman_matrix );

}

#endif

#ifndef GRAPH_SLAM_MATRIX_HELPER_H
#define GRAPH_SLAM_MATRIX_HELPER_H

#include <Eigen/Core>

namespace graph_slam 
{
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    
    Matrix6d combineToPoseCovariance(const Eigen::Matrix3d& position_cov, const Eigen::Matrix3d& orientation_cov);
    
} // end namespace

#endif

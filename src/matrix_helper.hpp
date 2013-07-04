#ifndef GRAPH_SLAM_MATRIX_HELPER_H
#define GRAPH_SLAM_MATRIX_HELPER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

namespace graph_slam 
{
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    
    Matrix6d combineToPoseCovariance(const Eigen::Matrix3d& position_cov, const Eigen::Matrix3d& orientation_cov);
    
    Matrix6d switchEnvireG2oCov(const Matrix6d& cov);
    
    template<typename ScalarType, int Dim>
    double computeMahalanobisDistance(const Eigen::Matrix<ScalarType, Dim, 1>& mu, const Eigen::Matrix<ScalarType, Dim, Dim>& covariance, const Eigen::Matrix<ScalarType, Dim, 1>& x)
    {
        Eigen::Matrix<ScalarType, Dim, 1> diff = x - mu;
        return sqrt((diff.transpose() * covariance.inverse() * diff)(0,0));
    }
    
} // end namespace

#endif

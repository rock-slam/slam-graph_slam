#include "matrix_helper.hpp"

namespace graph_slam
{
    
Matrix6d combineToPoseCovariance(const Eigen::Matrix3d& position_cov, const Eigen::Matrix3d& orientation_cov)
{
    Matrix6d cov_matrix = Matrix6d::Zero();
    cov_matrix << position_cov, Eigen::Matrix3d::Zero(),
                 Eigen::Matrix3d::Zero(), orientation_cov;
    return cov_matrix;
}

Matrix6d switchEnvireG2oCov(const Matrix6d& cov)
{
    Matrix6d cov_matrix = Matrix6d::Zero();
    cov_matrix << cov.bottomRightCorner<3,3>(), cov.bottomLeftCorner<3,3>(),
                    cov.topRightCorner<3,3>(), cov.topLeftCorner<3,3>();
    return cov_matrix;
}
    
}
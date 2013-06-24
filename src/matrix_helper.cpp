#include "matrix_helper.hpp"
#include <stdexcept>

namespace graph_slam
{
    
Matrix6d combineToPoseCovariance(const Eigen::Matrix3d& position_cov, const Eigen::Matrix3d& orientation_cov)
{
    Matrix6d cov_matrix = Matrix6d::Zero();
    
    mergeSubMatrix(cov_matrix, position_cov, 0, 0);
    mergeSubMatrix(cov_matrix, orientation_cov, 3, 3);
    
    return cov_matrix;
}

void mergeSubMatrix(Matrix6d& matrix, const Eigen::Matrix3d& sub_matrix, unsigned int row_upper_left_offset, unsigned int col_upper_left_offset)
{    
    if((int)row_upper_left_offset > ((int)matrix.rows() - (int)sub_matrix.rows()) || 
       (int)col_upper_left_offset > ((int)matrix.cols() - (int)sub_matrix.cols()))
    {
        throw std::runtime_error("mergeSubMatrix: Can't merge sub matrix. Index is out of range.");
    }
    
    for(unsigned i = 0; i < sub_matrix.rows(); i++)
    {
        for(unsigned j = 0; j < sub_matrix.cols(); j++)
        {
            matrix(row_upper_left_offset+i, col_upper_left_offset+j) = sub_matrix(i, j);
        }
    }
}
    
}
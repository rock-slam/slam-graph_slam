#include "Hogman2Eigen.hpp"

namespace graph_slam 
{

Transformation3 eigen2Hogman( const Eigen::Affine3d& eigen_transform )
{
    Eigen::Quaternionf eigen_quat(eigen_transform.rotation().cast<float>());
    Eigen::Matrix4d eigen_mat( eigen_transform.matrix() );
    Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
    Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
	    eigen_quat.w());
    Transformation3 result(translation, rotation);

    return result;
}

Matrix6 eigen2Hogman( const Eigen::Matrix<double,6,6>& eigen_matrix )
{
    Matrix6 result;
    // TODO might be more efficient to perform a memcopy here
    for( int m=0; m<6; m++ )
	for( int n=0; n<6; n++ )
	    result[m][n] = eigen_matrix(m,n);

    return result;
}

Eigen::Matrix<double,6,6> hogman2Eigen( const Matrix6& hogman )
{
    Eigen::Matrix<double,6,6> result;

    for( int m=0; m<6; m++ )
	for( int n=0; n<6; n++ )
	    result(m,n) = hogman[m][n];

    return result;
}

Eigen::Affine3d hogman2Eigen( const Transformation3& hogman_transform )
{
    Eigen::Quaterniond rotation( 
	    hogman_transform.rotation().w(),
	    hogman_transform.rotation().x(),
	    hogman_transform.rotation().y(),
	    hogman_transform.rotation().z() );

    Eigen::Translation3d translation(
	    hogman_transform.translation().x(),
	    hogman_transform.translation().y(),
	    hogman_transform.translation().z() );

    return translation * rotation;
}

/** 
 * transform a 6x6 covariance matrix in [r t] order to a hogman 6 inverse
 * covariance matrix in [t r] format.
 */
Matrix6 envireCov2HogmanInf( const Eigen::Matrix<double,6,6>& eigen_matrix )
{
    Eigen::Matrix<double,6,6> t;
    t << eigen_matrix.bottomRightCorner<3,3>(), eigen_matrix.bottomLeftCorner<3,3>(),
      eigen_matrix.topRightCorner<3,3>(), eigen_matrix.topLeftCorner<3,3>();

    return eigen2Hogman( Eigen::Matrix<double,6,6>(t.inverse()) );
}

/** 
 * Transform a 6x6 covariance matrix in [t r] order from hogman Covariance
 * format to the envire [r t] order.
 */
Eigen::Matrix<double,6,6> hogmanCov2EnvireCov( const Matrix6& hogman_matrix )
{
    Eigen::Matrix<double,6,6> t = hogman2Eigen( hogman_matrix );
    Eigen::Matrix<double,6,6> t1;
    t1 << t.bottomRightCorner<3,3>(), t.bottomLeftCorner<3,3>(),
      t.topRightCorner<3,3>(), t.topLeftCorner<3,3>();
    return t1;
}

}



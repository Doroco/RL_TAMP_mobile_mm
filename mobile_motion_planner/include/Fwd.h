#pragma once
#include <Eigen/Dense>
using namespace Eigen;

typedef const Ref<const MatrixXd>   & Cref_matrixXd;
typedef const Ref<const VectorXd>   & Cref_vectorXd;
typedef const Ref<const Matrix2d>   & Cref_matrix2d;

typedef Ref<MatrixXd>   Ref_matrixXd;
typedef Ref<VectorXd>   Ref_vectorXd;
typedef Ref<Matrix2d>   Ref_matrix2d;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor> MatrixRowMajor;
typedef Transform<double, 3, Eigen::Affine> Transform3d;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3Xd;

typedef std::size_t Index;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6x;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
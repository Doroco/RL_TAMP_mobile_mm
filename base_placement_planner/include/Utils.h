#pragma once
#include "Fwd.h"
#include <rbdl/rbdl.h>

Matrix3d rotateXaxis(const double rAngle);
Matrix3d rotateYaxis(const double rAngle);
Matrix3d rotateZaxis(const double rAngle);
Matrix3f rotateZaxis_f(const float rAngle);

Vector3d getPhi(Matrix3d Rot, Matrix3d Rotd);
double cubicSpline(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);


std::vector<Eigen::MatrixXf> TransToRp(const Eigen::MatrixXf& T);
Eigen::MatrixXf TransInv(const Eigen::MatrixXf& transform);
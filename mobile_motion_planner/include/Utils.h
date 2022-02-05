#pragma once
#include "Fwd.h"

Matrix3d rotateXaxis(const double rAngle);
Matrix3d rotateYaxis(const double rAngle);
Matrix3d rotateZaxis(const double rAngle);

Vector3d getPhi(Matrix3d Rot, Matrix3d Rotd);
double cubicSpline(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);

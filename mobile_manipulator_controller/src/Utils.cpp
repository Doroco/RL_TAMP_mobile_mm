#pragma once
#include "Utils.h"

#include <iostream>
using namespace Eigen;
using namespace std;

Matrix3d rotateXaxis(const double rAngle)
{
	Matrix3d _Rotate_wth_X;

	_Rotate_wth_X(0, 0) = 1.0;
	_Rotate_wth_X(1, 0) = 0.0;
	_Rotate_wth_X(2, 0) = 0.0;

	_Rotate_wth_X(0, 1) = 0.0;
	_Rotate_wth_X(1, 1) = cos(rAngle);
	_Rotate_wth_X(2, 1) = sin(rAngle);

	_Rotate_wth_X(0, 2) = 0.0;
	_Rotate_wth_X(1, 2) = -sin(rAngle);
	_Rotate_wth_X(2, 2) = cos(rAngle);

	return(_Rotate_wth_X);
}

Matrix3d rotateYaxis(const double rAngle)
{
	Matrix3d _Rotate_wth_Y(3, 3);

	_Rotate_wth_Y(0, 0) = cos(rAngle);
	_Rotate_wth_Y(1, 0) = 0.0;
	_Rotate_wth_Y(2, 0) = -sin(rAngle);

	_Rotate_wth_Y(0, 1) = 0.0;
	_Rotate_wth_Y(1, 1) = 1.0;
	_Rotate_wth_Y(2, 1) = 0.0;

	_Rotate_wth_Y(0, 2) = sin(rAngle);
	_Rotate_wth_Y(1, 2) = 0.0;
	_Rotate_wth_Y(2, 2) = cos(rAngle);

	return(_Rotate_wth_Y);
}

Matrix3d rotateZaxis(const double rAngle)
{
	Matrix3d _Rotate_wth_Z(3, 3);

	_Rotate_wth_Z(0, 0) = cos(rAngle);
	_Rotate_wth_Z(1, 0) = sin(rAngle);
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sin(rAngle);
	_Rotate_wth_Z(1, 1) = cos(rAngle);
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return(_Rotate_wth_Z);
}

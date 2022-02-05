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

Matrix3f rotateZaxis_f(const float rAngle)
{
	Matrix3f _Rotate_wth_Z(3, 3);

	_Rotate_wth_Z(0, 0) = cosf(rAngle);
	_Rotate_wth_Z(1, 0) = sinf(rAngle);
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sinf(rAngle);
	_Rotate_wth_Z(1, 1) = cosf(rAngle);
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return(_Rotate_wth_Z);
}


Eigen::Quaterniond Rot_to_Quat(Matrix3d Rot){
	double trace = Rot.trace();
	double q0, q1, q2, q3;
	q0 = sqrt((trace + 1)/4.0); 
	q1 = sqrt( Rot(0,0)/2.0 + (1.0 - trace)/4.0 );
	q2 = sqrt( Rot(1,1)/2.0 + (1.0 - trace)/4.0 );
	q3 = sqrt( Rot(2,2)/2.0 + (1.0 - trace)/4.0 );

	//cout << q0 << "\t" << q1 << "\t" << q2 << "\t" << q3 << endl;
	Eigen::Quaterniond quat(q0, q1, q2, q3);
	//cout << quat.w() << endl;
	return quat;
}


Vector3d getPhi(Matrix3d Rot, Matrix3d Rotd) {


	Vector3d phi;
	Vector3d s[3], v[3], w[3];

	for (int i = 0; i < 3; i++) {
		v[i] = Rot.block(0, i, 3, 1);
		w[i] = Rotd.block(0, i, 3, 1);
		s[i] = v[i].cross(w[i]);
	}
	phi = s[0] + s[1] + s[2];
	phi = -0.5* phi;

	return phi;
}

double cubicSpline(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{

	double rx_t;
	if (rT<rT_0)
	{
		rx_t = rx_0;
	}
	else if (rT >= rT_0 && rT<rT_f)
	{
		rx_t = rx_0 + rx_dot_0 * (rT - rT_0)
			+ (3 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2 * rx_dot_0 / ((rT_f - rT_0) * (rT_f - rT_0)) - rx_dot_f / ((rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)
			+ (-2 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)*(rT - rT_0);
	}
	else
	{
		rx_t = rx_f;
	}
	return (rx_t);
}


std::vector<Eigen::MatrixXf> TransToRp(const Eigen::MatrixXf& T) 
{
    std::vector<Eigen::MatrixXf> Rp_ret;
    Eigen::Matrix3f R_ret;
    // Get top left 3x3 corner
    R_ret = T.block<3, 3>(0, 0);

    Eigen::Vector3f p_ret(T(0, 3), T(1, 3), T(2, 3));

    Rp_ret.push_back(R_ret);
    Rp_ret.push_back(p_ret);

    return Rp_ret;
}


Eigen::MatrixXf TransInv(const Eigen::MatrixXf& transform) {
    auto rp = TransToRp(transform);
    auto Rt = rp.at(0).transpose();
    auto t = -(Rt * rp.at(1));
    Eigen::MatrixXf inv(4, 4);
    inv = Eigen::MatrixXf::Zero(4,4);
    inv.block(0, 0, 3, 3) = Rt;
    inv.block(0, 3, 3, 1) = t;
    inv(3, 3) = 1;
    return inv;
}
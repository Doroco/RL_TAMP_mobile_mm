#ifndef RRTFUNCTION_H_
#define RRTFUNCTION_H_

#include "NodeTree.h"
#include <rbdl/rbdl.h>
#include <memory>
#include "Utils.h"


using namespace RigidBodyDynamics;
using namespace std;

struct Robotmodel {
	std::vector<int> body_id_vec;
	std::vector<Vector3d> body_com_pos; 
	VectorXd q;
	Model model_;
};

struct ST_OBB
{
	std::string id_name;
	Vector3d vCenter;
	Vector3d vPos;
	Vector3d vAxis[3];
	Vector3d fAxis; 
	Matrix3d vRot;
};

class RRT
{
public:
	virtual ~RRT() {};


	std::vector<double> extendNode(std::vector<double> &node, RRTNodePtr &near);
	std::vector<double> extendNodeWithConstraints(std::vector<double> &node, RRTNodePtr &near);

	std::vector<double> generateRandomConfig();

	bool solveCRRT(Robotmodel& model, std::ostream& sout);
	bool solveRRT(Robotmodel& model, std::ostream& sout);

	// Project configurations into Constrained manifold
	std::vector<double> projectConfig(Robotmodel model, std::vector<double> qold, std::vector<double> &qs);
	//std::vector<double> ProjectConfigForRandomConfig(Robotmodel model, std::vector<double> &qs); deprecated

	// Check Constraints
	bool checkJointLimit(std::vector<double> q);
	bool checkExternalCollision(Robotmodel model, std::vector<double> &config);
	bool checkSelfCollision(Robotmodel model, std::vector<double> &config);

	// Smooth the path
	bool smoothPath(std::ostream& sout, std::istream& sinput);
	bool shortcutSmoothing();
	bool checkTraj(std::vector<double> &a, std::vector<double> &b);
	
	double getDistance(std::vector<double> &a, std::vector<double> &b);
	std::vector<double> getUnitVector(std::vector<double> &a, std::vector<double> &b);

	MatrixXd MergeRRTResults(MatrixXd joint_target1, MatrixXd joint_target2, int case_);

	// Member parameters
	NodeTree *nt_pointer;
	NodeTree nt_start;
	NodeTree nt_goal;

	std::vector<double> initConfig;
	std::vector<double> torso_config;
	std::vector<VectorXd> qgoals;
	VectorXd lower_limit, upper_limit, qinit, qgoal;

	std::vector<double> DOF_weights;
	std::vector<std::vector<double> > path;

	VectorXd dx_from_init_to_goal;
	VectorXd dx_from_extend_to_goal;

	double step_size;
	int t_turn;
	bool isGoal;
	int dofSize;
	MatrixXd C; // for constraint matrix

	Vector3d refer_pos;
	Matrix3d refer_rot;
	Matrix3d E_rpy;
	Vector3d rpy;
	Vector3d local_pos;
	Matrix3d local_rot;

	Robotmodel _model;	
	ST_OBB Box_obs[10], Box_link[10];
	std::vector<ST_OBB> Box_env, Box_robot;
	int box_num_obs, box_num_link;

	//Collision Check
	bool checkOBBCollision(ST_OBB* Box1, ST_OBB* Box2);
	float FDotProduct(const Vector3d v0, const Vector3d v1)
	{
		return v0(0) * v1(0) + v0(1) * v1(1) + v0(2) * v1(2);
	}
	static const Matrix3d Vec_2_Rot(const Vector3d& vec) {
		Matrix3d Rot;
		double angle = vec.norm();

		if (angle == 0.0)
			Rot = Matrix3d::Identity();
		else {
			Vector3d axis = vec / angle;
			Matrix3d V;
			V.setZero();
			V(0, 1) = -axis(2);
			V(0, 2) = axis(1);
			V(1, 0) = axis(2);
			V(1, 2) = -axis(0);
			V(2, 0) = -axis(1);
			V(2, 1) = axis(0);

			Rot = Matrix3d::Identity() + V * sin(angle) + (1 - cos(angle))* V* V;
		}
		return Rot;
	}
	static const double getEuclideanNorm(const std::vector<double> a, const std::vector<double> b, int size) {
		double res = 0.0;
		for (int i = 0; i < size; i++)
			res += pow(a[i] - b[i], 2);

		res = sqrt(res);
		return res;
	}
		double vector_norm(const VectorXd a, const VectorXd b, int size) {
		double res = 0.0;
		for (int i = 0; i < size; i++)
			res += pow(a(i) - b(i), 2);

		res = sqrt(res);
		return res;
	}
};


#endif
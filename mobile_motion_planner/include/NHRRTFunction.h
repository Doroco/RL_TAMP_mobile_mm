#ifndef NHRRTFUNCTION_H_
#define NHRRTFUNCTION_H_
#include "Utils.h"
#include <vector>


using namespace std;

struct ObstacleInfo
{
	Vector2d pos;
	double radius;
};

struct Node
{
	int id;
	Vector3d pose;
	std::vector<Vector3d> edgeq;
	int parent_id; // parent id
	double cost;

};
class NonHolonomicRRT
{
public:
	virtual ~NonHolonomicRRT() {};

	//NH-RRT 
	bool solveNHRRT(std::ostream& sout);

	Node extendNode(Node & v_near, Vector3d &q_rand);
	double computeCost(std::vector<Vector3d> & path, Vector3d &q_0);
	Vector3d generateRandomConfig();

	bool checkCollision2D(ObstacleInfo* Obs, Vector3d q);
	int checkEdge(ObstacleInfo* Obs, Node vnew_i);

	std::vector<Vector3d> positionCtrlGlobal(Vector3d & q_start, Vector3d & q_end, int dir, double delta_T, double b);
	void positionCtrlLocal(double t, Vector3d q_current, Vector3d q_end, int dir, double b, double & vl, double & vr, int & eot, double &vm, double &vd);
	double normAngle(double& angle, double min);
	VectorXd qinit, qgoal, pos_lower_limit, pos_upper_limit, goal_bias_lower, goal_bias_upper;

	ObstacleInfo obs_info_[10];
	int cntId;
	int obs_num;
	int addedVert;
	double mobile_width, mobile_length;




};


#endif

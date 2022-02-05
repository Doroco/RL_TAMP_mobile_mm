#include <iostream>
#include "NHRRTFunction.h"
#include <deque>
#include <string>
#include <cstring>

using namespace std;
using namespace Eigen;



bool NonHolonomicRRT::solveNHRRT(std::ostream& sout) {

	////////////////////////////////////// Initial Setting ///////////////////////////
	//cout << "restart!" << endl;

	bool finished = false;
	int iter = 1;
	addedVert = 0;
	Node v_to_extend;
	cntId = 1;
	Node v_near;
	Node tau[3000];
	tau[0].id = 1;
	tau[0].pose = qinit;
	tau[0].edgeq.clear();
	tau[0].parent_id = 0;
	tau[0].cost = 0;
	double radius_goal = 0.05;
	std::deque<std::vector<Vector3d>> goal_path;

	int rootreached = 1;
	Node set_parents_vnear[3000];
	int counter_parents_vnear;
	int counter_parents_checked = 1;
	int collision_ = 0;
	int collision2_ = 0;

	////////////////////////////////////// Main algorithm ////////////////////////
	while (iter < 10000) {
		// random sampling 
		Vector3d q_rand = generateRandomConfig();
		// goal-biased sampling

		// Check collision for sampling
		for (int i = 0;i < obs_num;i++) {

			// if dist > center + radius/2 : true
			if (!checkCollision2D(&obs_info_[i], q_rand)) {
				collision_++;
			}
		}
		if (collision_ > 0) {
			collision_ = 0;
			continue;
		}


		// Get the nearest tree from sampling
		double mind = 100.0;
		int min = 0;
		for (int i = 0;i <= addedVert;i++) {
			double d = sqrt(pow(q_rand(0) - tau[i].pose(0), 2) + pow(q_rand(1) - tau[i].pose(1), 2));
			if (d < mind) {
				min = i; // index for vertex
				mind = d; // distance
			}
		}

		v_near = tau[min]; // update for vnear


		rootreached = 1;
		counter_parents_checked = 1;
		counter_parents_vnear = 0;

		// backward search from current node to root 
		while (rootreached > 0) {

			if (counter_parents_checked == 1) {
				v_to_extend = v_near; // 
			}
			else {
				if (v_to_extend.parent_id == 0) {
					// parent id �� q_init�̸� ����
					// parent id ���� extend �� �� �ִ��� Ȯ�� 
					rootreached = 0;
					break;
				}
				else {
					// search vertex
					for (int i = 0;i <= addedVert;i++) {
						if (tau[i].id == v_to_extend.parent_id) {
							v_to_extend = tau[i];
						}
						// tau�� ��ƽ �Ŀ��� "q_init�� parent_id��" ����ų�״� v_to_extend�� q_init���� �ѹ� extend�� ��尡 ��. 
					}
				}
			}

			// Extend the tree from v_to_extend to q_rand
			Node v_new_i = extendNode(v_to_extend, q_rand);

			// check collision for every edge of v_new_i
			for (int i = 0;i < obs_num;i++) {
				collision2_ += checkEdge(&obs_info_[i], v_new_i);
			}

			counter_parents_checked++;

			if (collision2_ > 0) {
				collision2_ = 0;
				continue;
			}
			else {
				// copy collision-free v_new_i
				set_parents_vnear[counter_parents_vnear].id = v_new_i.id;
				set_parents_vnear[counter_parents_vnear].pose = v_new_i.pose;
				set_parents_vnear[counter_parents_vnear].edgeq = v_new_i.edgeq;
				set_parents_vnear[counter_parents_vnear].parent_id = v_new_i.parent_id;
				set_parents_vnear[counter_parents_vnear].cost = v_new_i.cost;

				counter_parents_vnear++;
			}
		}

		int nv = counter_parents_vnear;
		if (nv == 0)
			continue;

		Node v_min;
		double min_cost;
		v_min = set_parents_vnear[counter_parents_vnear - 1];
		min_cost = set_parents_vnear[counter_parents_vnear - 1].cost;

		// check from v_new_i to parent for getting minimum cost node
		for (int i = 0;i < nv;i++) {
			if (set_parents_vnear[i].parent_id == 0 || set_parents_vnear[i].parent_id == 1) {
				v_min = set_parents_vnear[i];
				break;
			}

			if (set_parents_vnear[i].cost < min_cost) {
				v_min = set_parents_vnear[i];
				min_cost = set_parents_vnear[i].cost;
			}
		}
		addedVert++;
		cntId++;
		//cout<<"added" << addedVert << endl;
		v_min.id = cntId;
		tau[addedVert].id = v_min.id;
		tau[addedVert].pose = v_min.pose;
		tau[addedVert].edgeq = v_min.edgeq;
		tau[addedVert].parent_id = v_min.parent_id;
		tau[addedVert].cost = v_min.cost;

		if (sqrt(pow(v_min.pose(0) - qgoal(0), 2) + pow(v_min.pose(1) - qgoal(1), 2) + pow(v_min.pose(2) - qgoal(2), 2)) < radius_goal) {
			int id;
			//cout << "goal reached" << endl;
			id = v_min.parent_id;
			while (id > 0) {
				goal_path.push_front(v_min.edgeq);
				//for (int i = 0;i < v_min.edgeq.size();i++) {
				//	cout << v_min.edgeq[i].transpose() << endl;
				//}
				for (int i = 0;i <= addedVert;i++) {
					if (tau[i].id == id) {
						v_min = tau[i];
						id = v_min.parent_id;
						break;

					}
				}
			}


			//	std::vector<std::vector<Vector3d>>::size_type i;

			//for (i = 0; i<goal_path.size();i++) {
			//	std::vector<Vector3d> & a = goal_path[i];
			//	for (auto & aa : a)
			//	{
			//		cout << aa.transpose() << endl;
			//	}
			//
			//	//for (int j=0; j<a.size();i++)
			//	//{
			//	//	cout << a[j].transpose() << endl;
			//	//}
			//		//cout << path_[i](2) << endl;
			//}

			//getchar();
			finished = true;
			cout << "# iter" << addedVert << endl;
			break;
		}



		iter++;
		//cout << iter << endl;
	}
	if (finished) {
		for (int i = 0; i < goal_path.size(); i++) {
			std::vector<Vector3d> node = goal_path[i];
			for (int j = 0; j < node.size(); j++) {
				Vector3d traj = node[j];
				for (int k = 0;k < 3;k++) {
					if (k == 2)
						sout << traj(k) << "\n";
					else
						sout << traj(k) << ",";

				}
			}
		}
		return true;
	}
	else {

		return false;
	}

}


Vector3d NonHolonomicRRT::generateRandomConfig() {
	Vector3d R;
	for (int i = 0; i < R.size(); i++) {
		double jointrange = pos_upper_limit(i) - pos_lower_limit(i);
		double r = ((double)rand() / (double)RAND_MAX)*jointrange;
		R(i) = pos_lower_limit(i) + r;
	}

	if (cntId > 50) {
		double goal_bias = (double)rand() / RAND_MAX;
		if (goal_bias > 0.9) {
			for (int i = 0; i < R.size(); i++) {
				double jointrange = goal_bias_upper(i) - goal_bias_lower(i);
				double r = ((double)rand() / (double)RAND_MAX)*jointrange;
				R(i) = goal_bias_lower(i) + r;
			}
		}
	}
	return R;
}
Node NonHolonomicRRT::extendNode(Node & v_near, Vector3d &q_rand) {

	Vector3d q0;
	q0 = v_near.pose;
	Node v_new;
	std::vector<Vector3d> Q_new;
	Q_new = positionCtrlGlobal(q0, q_rand, 0, 0.05, 0.6);

	v_new.id = cntId;
	v_new.pose = Q_new.back();
	v_new.edgeq = Q_new;
	v_new.parent_id = v_near.id;
	v_new.cost = computeCost(Q_new, q_rand) + v_near.cost;

	return v_new;
}
double NonHolonomicRRT::computeCost(std::vector<Vector3d>& path, Vector3d &q_0) {
	double cost;
	cost = 0.0;
	std::vector<Vector3d> path_ = path;

	std::vector<Vector3d>::size_type i;
	for (i = 1; i<path_.size();i++) {
		cost = cost + 10*abs((normAngle(path_[i](2), 0) - normAngle(path_[i-1](2), 0)));
	}
	Vector3d end_ = path_.back();

	cost += 0.5*sqrt(pow(end_(0) - q_0(0), 2) + pow(end_(1) - q_0(1), 2));
	return cost;
}

bool NonHolonomicRRT::checkCollision2D(ObstacleInfo* Obs, Vector3d q) {

	double dist;
	dist = pow((Obs->pos(0) - q(0)), 2) + pow((Obs->pos(1) - q(1)), 2);
	dist = sqrt(dist);

	double robot_max_size = max(mobile_length, mobile_width);

	if ((dist - (Obs->radius + robot_max_size / 2.0)) > 0.0)
		return true; // collision-free
	else
		return false; // collision
}
int NonHolonomicRRT::checkEdge(ObstacleInfo* Obs, Node vnew_i) {
	Vector3d q0;
	int flag = 0;
	for (std::vector<Vector3d>::size_type i = 0;i < vnew_i.edgeq.size();i++) {
		q0 = vnew_i.edgeq[i];
		if (!checkCollision2D(Obs, q0)) {
			flag++;
			//cout << q0.transpose() << endl;
		}
	}
	return flag;
}

std::vector<Vector3d> NonHolonomicRRT::positionCtrlGlobal(Vector3d & q_start, Vector3d &q_end, int dir, double delta_T, double b) {

	double sl = 0.0;
	double sr = 0.0;
	double oldSl = 0.0;
	double oldSr = 0.0;
	std::vector<Eigen::Vector3d> x_vec;
	x_vec.clear();

	Vector2d encoders;
	encoders.setZero();
	double ti = 0.0;
	int eot = 0;
	Vector3d x_current;

	x_current = q_start;

	////////////////////////////////////////////////////

	double dSl, dSr, dSm, dSd;
	double vl, vr, vm, vd;


	Vector2d speeds;
	speeds.setZero();

	Vector2d vm_vd;
	vm_vd.setZero();

	Vector3d x_end;
	x_end = q_end;
	//cout << "x_current" << x_current.transpose() << endl;
	//cout << "x_end" << x_end.transpose() << endl;

	while (!eot) {
		dSl = sl - oldSl; // difference of wheel angle
		dSr = sr - oldSr;
		dSm = (dSl + dSr) / 2.0;
		dSd = (dSr - dSl) / b;

		x_current(0) = x_current(0) + dSm * cos(x_current(2) + dSd / 2.0);
		x_current(1) = x_current(1) + dSm * sin(x_current(2) + dSd / 2.0);
		double temp = x_current(2) + dSd;
		x_current(2) = normAngle(temp, -M_PI);

		//cout << "x_current" << x_current.transpose() << endl;
		positionCtrlLocal(ti, x_current, x_end, dir, b, vl, vr, eot, vm_vd(0), vm_vd(1));
		speeds(0) = vl;
		speeds(1) = vr;

		x_vec.push_back(x_current);
		if (eot) {
			x_vec.push_back(x_end);
		}

		ti = ti + delta_T;

		encoders(0) = encoders(0) + speeds(0)*delta_T;
		encoders(1) = encoders(1) + speeds(1)*delta_T;

		oldSl = sl;
		oldSr = sr;

		sl = encoders(0);
		sr = encoders(1);

	}

	return x_vec;
}
void NonHolonomicRRT::positionCtrlLocal(double t, Vector3d q_current, Vector3d q_end, int dir, double b, double& vl, double& vr, int& eot, double &vm, double& vd) {
	double Kv = 3.8; // 3.8
	double Krho = 1.0; // 3.0 // condition : Kalpha + 5/3*Kbeta - 2/pi*Krho > 0
	double Kalpha = 8.0; //6.0; // 8.0
	double Kbeta = -1.5;

	double Vmax = 1.0;

	double RhoEndCondition = 0.05;
	double PhiEndCondition = 20 * M_PI / 180.0;

	double oldBeta;
	double t_;
	t_ = t;
	if (t_ == 0) {
		oldBeta = 0.0;

	}


	double tc = q_current(2);
	double te = q_end(2);

	double dx = q_end(0) - q_current(0);
	double dy = q_end(1) - q_current(1);
	double rho = sqrt(pow(dx, 2) + pow(dy, 2));
	double fRho = rho;

	if (fRho > (Vmax / Krho))
		fRho = Vmax / Krho;

	double alpha = atan2(dy, dx) - tc;
	alpha = normAngle(alpha, -M_PI);

	if (dir == 0) {
		if (alpha > M_PI / 2.0) {
			fRho = -fRho;
			alpha = alpha - M_PI;
		}
		else if (alpha <= -M_PI / 2.0) {
			fRho = -fRho;
			alpha = alpha + M_PI;
		}
	}
	else if (dir == -1) {
		fRho = -fRho;
		alpha = alpha + M_PI;
		if (alpha > M_PI)
			alpha = alpha - 2.0*M_PI;
	}

	double phi = te - tc;
	phi = normAngle(phi, -M_PI);
	double beta = phi - alpha;
	beta = normAngle(beta, -M_PI);

	if (abs(oldBeta - beta) > M_PI)
		beta = oldBeta;

	oldBeta = beta;

	vm = Krho * tanh(fRho*Kv);
	vd = Kalpha * alpha + Kbeta * beta;

	if (rho < RhoEndCondition && abs(phi) < PhiEndCondition) {
		//cout << "rho" << rho << endl;
		//cout << "phi" << phi << endl;
		//cout << q_end.transpose() << endl;
		eot = 1;
	}
	else
	{
		eot = 0;
	}
	vl = vm - vd * b / 2.0;
	if (abs(vl) > Vmax)
		vl = Vmax * abs(vl) / vl;

	vr = vm + vd * b / 2.0;
	if (abs(vr) > Vmax)
		vr = Vmax * abs(vr) / vr;



}
double NonHolonomicRRT::normAngle(double& angle, double min) {
	double temp_angle = angle;
	while (temp_angle >= min + 2 * M_PI) {
		temp_angle -= 2 * M_PI;
	}

	while (temp_angle < min) {
		temp_angle += 2 * M_PI;
	}

	double a = temp_angle;
	//cout << a << endl;

	return a;
}


#include <iostream>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include "RRTFunction.h"
#include <string>
#include <cstring>

using namespace std;
using namespace Eigen;

#define REACHED 0
#define ADVANCED 1
#define TRAPPED -1

bool RRT::solveRRT(Robotmodel& model, std::ostream& sout) {

	_model = model;

	nt_start._nodes.clear(); // NodeTree
	nt_goal._nodes.clear(); // NodeTree

	initConfig.clear();// std::vector<double>
	DOF_weights.clear();// std::vector<double>
	int ActiveDoFs = dofSize;
	step_size = 0.15 / M_PI *180.0;
	std::string start_string[dofSize];

	for (int i = 0; i < ActiveDoFs; i++) {
		initConfig.push_back(qinit(i) * 180.0 / M_PI);
		DOF_weights.push_back(1.0);
	}


	auto start_node = std::make_shared<RRTNode>(initConfig);

	nt_start.addNode(start_node);
	nt_pointer = &nt_start;

	// mulitple seed
	for (int i = 0; i < qgoals.size(); i++)
	{
		std::vector<double> goalConfig; 
		for (int j = 0; j < ActiveDoFs; j++)
		{
			goalConfig.push_back(qgoals[i][j] * 180.0 / M_PI);
		}

		auto goal_node = std::make_shared<RRTNode>(goalConfig);

		nt_goal.addNode(goal_node);
	}

	bool finished = false;
	int iter = 1;

    std::vector<double> q_rand;
    RRTNodePtr q_a_near;
    std::vector<double> q_a_reached ;
    RRTNodePtr q_b_near;
    std::vector<double> q_b_reached ;
	int root_tree_index, goal_tree_index;

	while (iter < 500) {

		nt_pointer = &nt_start;								  //		// start tree
		q_rand = generateRandomConfig();
		q_a_near = nt_pointer->getNearest(q_rand, DOF_weights);  

		q_a_reached = this->extendNode(q_rand, q_a_near);

		nt_pointer = &nt_goal;
		q_b_near = nt_pointer->getNearest(q_a_reached, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��
		q_b_reached = this->extendNode(q_a_reached, q_b_near); // near��� ���� random config�� Extend

		if (getEuclideanNorm(q_a_reached, q_b_reached, dofSize) < step_size)
		{
			nt_pointer = &nt_start; //		// start tree
			root_tree_index = nt_pointer->getNearestIndex(q_a_reached, DOF_weights);
			nt_pointer = &nt_goal;
			goal_tree_index = nt_pointer->getNearestIndex(q_b_reached, DOF_weights);

			// q_a_reached가 tree내에 있는 노드가 아니여서 불연속이 생김..
			//cout << "t_turn \t" << t_turn << endl;
			//cout << q_a_reached[0] << "\t" << q_a_reached[1] << "\t" << q_a_reached[2] << "\t" << q_a_reached[3] << "\t" << q_a_reached[4] << "\t" << q_a_reached[5] << "\t" << q_a_reached[6] << endl;
			//cout << q_b_reached[0] << "\t" << q_b_reached[1] << "\t" << q_b_reached[2] << "\t" << q_b_reached[3] << "\t" << q_b_reached[4] << "\t" << q_b_reached[5] << "\t" << q_b_reached[6] << endl;

			//cout << getEuclideanNorm(q_a_reached, q_b_reached, dofSize) << endl;
			//cout << "reached" << endl;
			finished = true;

			break;
		}
		iter++;
	}
	if (finished) {

		path = nt_start.getPathWithIndex(root_tree_index);
		vector<vector<double>> p2;
		p2 = nt_goal.getPathWithIndex(goal_tree_index);


		reverse(p2.begin(), p2.end());
		path.insert(path.end(), p2.begin(), p2.end());

		for (int i = 0; i < path.size(); i++) {
			std::vector<double> node = path[i];
			for (int j = 0; j<node.size() - 1; j++)
				sout << node[j] << ",";

			sout << node[node.size() - 1] << "\n";
		}
	}
	else {
		return false;
	}

	return true;
}
bool RRT::solveCRRT(Robotmodel& model, std::ostream& sout) {
	_model = model;

	nt_start._nodes.clear();
	nt_goal._nodes.clear();

	initConfig.clear();
	DOF_weights.clear();

	int ActiveDoFs = dofSize;
	step_size = 2.0/M_PI*180.0;

	std::string start_string[dofSize];

	for (int i = 0; i < ActiveDoFs; i++) {
		initConfig.push_back(qinit(i) * 180.0 / M_PI);
		DOF_weights.push_back(1.0);
	}


	auto start_node = std::make_shared<RRTNode>(initConfig);

	nt_start.addNode(start_node);
	nt_pointer = &nt_start;
	t_turn = 1;

	// mulitple seed
	for (int i = 0; i < qgoals.size(); i++)
	{
		std::vector<double> goalConfig; 
		for (int j = 0; j < ActiveDoFs; j++)
		{
			goalConfig.push_back(qgoals[i][j] * 180.0 / M_PI);
		}
		auto goal_node = std::make_shared<RRTNode>(goalConfig);

		nt_goal.addNode(goal_node);
	}
	/// single seed
	// RRTNode g(goal, 0); // ���� RRT
	// gTree.addNode(g);


	dx_from_init_to_goal.resize(6);
	Vector3d pos_init = CalcBodyToBaseCoordinates(_model.model_, qinit, _model.body_id_vec.back(), _model.body_com_pos.back());
	Matrix3d Rot_init = CalcBodyWorldOrientation(_model.model_, qinit, _model.body_id_vec.back(), true).transpose();
	dx_from_init_to_goal.head(3) = refer_pos - pos_init;
	dx_from_init_to_goal.tail(3) = getPhi(Rot_init, refer_rot);

	dx_from_extend_to_goal.resize(6);


	bool finished = false;
	int iter = 1;
    int a = 1;
    std::vector<double> q_rand;
    RRTNodePtr q_a_near;
    std::vector<double> q_a_reached ;
    RRTNodePtr q_b_near;
    std::vector<double> q_b_reached ;
	int root_tree_index, goal_tree_index;
	while (iter < 1000)
	{
		if (t_turn == 1)
		{
			nt_pointer = &nt_start;								  //		// start tree
			q_rand = generateRandomConfig(); //ConstrainedRandomConfig(_model);					  // last node of goal tree(g2) or random config

			//cout << "start tree -> goal tree" << "\t" <<t_turn << endl;
			q_a_near = nt_pointer->getNearest(q_rand, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��
			q_a_reached = this->extendNodeWithConstraints(q_rand, q_a_near);   // near��� ���� random config�� Extend

			nt_pointer = &nt_goal;
			q_b_near = nt_pointer->getNearest(q_a_reached, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��
			
			//cout << q_b_near->getConfiguration()[0] << "\t" <<  q_b_near->getConfiguration()[1] << endl;
			//cout << "goal tree -> start tree" << endl;
			q_b_reached = this->extendNodeWithConstraints(q_a_reached, q_b_near); // near��� ���� random config�� Extend

			if (getEuclideanNorm(q_a_reached, q_b_reached, dofSize) < step_size)
			{
				nt_pointer = &nt_start;								  //		// start tree
				root_tree_index = nt_pointer->getNearestIndex(q_a_reached, DOF_weights);
				nt_pointer = &nt_goal;
				goal_tree_index = nt_pointer->getNearestIndex(q_b_reached, DOF_weights);

				// q_a_reached가 tree내에 있는 노드가 아니여서 불연속이 생김..
				//cout << "t_turn \t" << t_turn << endl;
				//cout << q_a_reached[0] << "\t" << q_a_reached[1] << "\t" << q_a_reached[2] << "\t" << q_a_reached[3] << "\t" << q_a_reached[4] << "\t" << q_a_reached[5] << "\t" << q_a_reached[6] << endl;
				//cout << q_b_reached[0] << "\t" << q_b_reached[1] << "\t" << q_b_reached[2] << "\t" << q_b_reached[3] << "\t" << q_b_reached[4] << "\t" << q_b_reached[5] << "\t" << q_b_reached[6] << endl;
				//cout << getEuclideanNorm(q_a_reached, q_b_reached, dofSize) << endl;
				//cout << "reached" << endl;
				a = REACHED;
				finished = true;

				break;
			}
			//t_turn = 2;
		}
		// if (t_turn == 2)
		// {
		// 	c_tree = &gTree;	// goal tree
		// 	q_rand = RandomConfig();// ConstrainedRandomConfig(_model);					  // last node of goal tree(g2) or random config

		// 	//cout << "goal tree -> start _tree " << "\t" <<t_turn << endl;
		// 	q_a_near = c_tree->getNearest(q_rand, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��

		// 	q_a_reached = this->ConstrainedExtend(q_rand, q_a_near); // near��� ���� random config�� Extend

		// 	c_tree = &tree;
		// 	q_b_near = c_tree->getNearest(q_a_reached, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��
		// 	//cout << "start tree -> goal tree" << endl;

		// 	q_b_reached = this->ConstrainedExtend(q_a_reached, q_b_near); // near��� ���� random config�� Extend
		// 	if (getEuclideanNorm(q_a_reached, q_b_reached, dofSize) < step_size)
		// 	{
		// 		c_tree = &gTree;								  //		// start tree
		// 		root_tree_index = c_tree->getNearestIndex(q_a_reached, DOF_weights);
		// 		c_tree = &tree;
		// 		goal_tree_index = c_tree->getNearestIndex(q_b_reached, DOF_weights);

		// 		// q_a_reached가 tree내에 있는 노드가 아니여서 불연속이 생김..
		// 		cout << "t_turn \t" << t_turn << endl;
		// 		cout << q_a_reached[0] << "\t" << q_a_reached[1] << "\t" << q_a_reached[2] << "\t" << q_a_reached[3] << "\t" << q_a_reached[4] << "\t" << q_a_reached[5] << "\t" << q_a_reached[6] << endl;
		// 		cout << q_b_reached[0] << "\t" << q_b_reached[1] << "\t" << q_b_reached[2] << "\t" << q_b_reached[3] << "\t" << q_b_reached[4] << "\t" << q_b_reached[5] << "\t" << q_b_reached[6] << endl;

		// 		cout << getEuclideanNorm(q_a_reached, q_b_reached, dofSize) << endl;
		// 		cout << "reached" << endl;
		// 		a = REACHED;
		// 		finished = true;
		// 	}
		// 	t_turn = 1;

		// }
		iter++;
	}

	if (finished) {

		path = nt_start.getPathWithIndex(root_tree_index);
		vector<vector<double>> p2;
		p2 = nt_goal.getPathWithIndex(goal_tree_index);

		reverse(p2.begin(), p2.end());
		path.insert(path.end(), p2.begin(), p2.end());
		
		std::vector<double> end;
		end = path[path.size()-1];

		for (int i = 0; i < path.size(); i++) {
			std::vector<double> node = path[i];
			for (int j = 0; j < node.size() - 1; j++){
				sout << node[j] << ",";
			}
			sout << node[node.size() - 1] << "\n";

		}//error

	}
	else {

		return false;
	}

	return true;
}
std::vector<double> RRT::generateRandomConfig() {

 	std::vector<double> R;
	do {
		for (int i = 0; i < initConfig.size(); i++) {
			double jointrange = upper_limit(i) - lower_limit(i); // angle
			double r = ((double)rand() / (double)RAND_MAX)*jointrange;
			R.push_back(lower_limit(i) + r);
		}
	} while (R.size() != initConfig.size());


	return R;
}
// std::vector<double> RRT::ConstrainedRandomConfig(Robotmodel model){
// 	std::vector<double> R;
// 	std::vector<double> R_project;

// 	while (R_project.empty())
// 	{
// 		do
// 		{
// 			for (int i = 0; i < start.size(); i++)
// 			{
// 				double jointrange = upper_limit(i) - lower_limit(i); // angle
// 				double r = ((double)rand() / (double)RAND_MAX) * jointrange;
// 				R.push_back(lower_limit(i) + r);
// 			}
// 		} while (R.size() != start.size());

// 		R_project = ProjectConfigForRandomConfig(model, R);
// 	}


// 	return R_project;
//}

std::vector<double> RRT::extendNode(std::vector<double> &node, RRTNodePtr &near){
	std::vector<double> qs, qs_old;
	qs.clear();
	qs_old.clear();
	for (int i = 0; i < dofSize; i++)
	{
		qs.push_back(near->getConfiguration()[i]);
		qs_old.push_back(near->getConfiguration()[i]);
	}

	while(true){
		if (getEuclideanNorm(node, qs, dofSize) < step_size)
		{
			return qs;
		}
		else if (getEuclideanNorm(node, qs, dofSize) - getEuclideanNorm(qs_old, node, dofSize) > 0.0)
		{
			return qs_old; // qs_old is close to node
		}

		for (int i = 0; i < initConfig.size(); i++)
		{
			qs[i] = (qs[i] + min(step_size, getEuclideanNorm(node, qs, dofSize)) * (node[i] - qs[i]) / getEuclideanNorm(node, qs, dofSize));
		}

		if (!checkExternalCollision(_model, qs) && !checkSelfCollision(_model, qs))
		{
			RRTNodePtr old = near;
			near = std::make_shared<RRTNode>(qs, old);
			nt_pointer->addNode(near);
			qs_old = qs;
		}
		else
		{
			//cout << "collision config \t" << qs[0] << "\t" <<qs[1] << "\t" <<qs[2] << "\t" <<qs[3] << "\t" <<qs[4] << "\t" <<qs[5] << "\t" <<qs[6] << "\t" << endl;
			return qs_old;
		}

	}
}



std::vector<double> RRT::extendNodeWithConstraints(std::vector<double> &node, RRTNodePtr &near)
{
	// node = q_rand, near : q_near
	std::vector<double> qs, qs_old;
	qs.clear();
	qs_old.clear();
	bool project_flag = false;
	int iteration_ = 0;

	for (int i = 0; i < dofSize; i++)
	{
		qs.push_back(near->getConfiguration()[i]);
		qs_old.push_back(near->getConfiguration()[i]);
	}
	//cout << "input" << node[0] << "\t" << node[1] << "\t" << node[2] << "\t" << node[3] << "\t" << node[4] << "\t" << node[5] << "\t" << node[6] << endl;
	//cout << "q_near" << qs[0] << "\t"<< qs[1] << "\t"<< qs[2] << "\t"<< qs[3] << "\t"<< qs[4] << "\t"<< qs[5] << "\t"<< qs[6] <<endl;
	while (true)
	{
		if (getEuclideanNorm(node, qs, dofSize) < step_size)
		{
			return qs;
		}
		else if (getEuclideanNorm(node, qs, dofSize) - getEuclideanNorm(qs_old, node, dofSize) > 0.0)
		{
			return qs_old; // qs_old is close to node
		}

		for (int i = 0; i < initConfig.size(); i++) // Move from 'qs' to 'node'
			qs[i] = (qs[i] + min(step_size, getEuclideanNorm(node, qs, dofSize)) * (node[i] - qs[i]) / getEuclideanNorm(node, qs, dofSize));

		//cout << "desired_dir" <<qs[0] << "\t" << qs[1] << "\t" << qs[2] << "\t" << qs[3] << "\t" << qs[4] << "\t" << qs[5] << "\t" << qs[6] << endl;

		// qs_old : near / qs : extend
		qs = projectConfig(_model, qs_old, qs); // project qs onto constraint manifold


		// if (!qs.empty() && (std_norm(qs, qs_old, dofSize) > 3.0*step_size)){
		// 	cout << "output1" <<qs_old[0] << "\t" << qs_old[1] << "\t" << qs_old[2] << "\t" << qs_old[3] << "\t" << qs_old[4] << "\t" << qs_old[5] << "\t" << qs_old[6] << endl;
		// 	cout << "output2" <<qs[0] << "\t" << qs[1] << "\t" << qs[2] << "\t" << qs[3] << "\t" << qs[4] << "\t" << qs[5] << "\t" << qs[6] << endl;

		// 	return qs_old;
		// }

		if (!qs.empty() && !checkExternalCollision(_model, qs))
		{
			//			VectorXd q_project_(7);
			//cout << "extend!" << endl;
			RRTNodePtr old = near;		   //qs_old
			near = std::make_shared<RRTNode>(qs, old) ; //(new RRTNode(qs, old)); // qs : configuration, old : parent node
			nt_pointer->addNode(near);

			if((getEuclideanNorm(qs, qs_old, dofSize) < step_size / 2.0)){
				return qs;
			}
			qs_old = qs;

		}
		else
		{	
			//cout << "qs_old" << qs_old[0] << "\t"<< qs_old[1] << "\t"<< qs_old[2] << "\t"<< qs_old[3] << "\t"<< qs_old[4] << "\t"<< qs_old[5] << "\t"<< qs_old[6] <<endl;
			//cout << "return NULL" << endl;
			//cout << "collision config \t" << qs[0] << "\t" <<qs[1] << "\t" <<qs[2] << "\t" <<qs[3] << "\t" <<qs[4] << "\t" <<qs[5] << "\t" <<qs[6] << "\t" << endl;

			return qs_old;
		}
	}
}
std::vector<double> RRT::projectConfig(Robotmodel model, std::vector<double> qold, std::vector<double> &qs)
{
	model.q.resize(dofSize);

	// Tc
	Matrix4d T0_c, T0_obj, Tc_obj;
	T0_c.setIdentity();
	//T0_c.topLeftCorner(3,3) = refer_rot;
	T0_c.topRightCorner(3, 1) = local_rot.transpose()*(refer_pos - local_pos); // global to local pos

	MatrixXd J_temp(6, dofSize), J(6, dofSize);
	VectorXd d_c(6), dx(6), q_error(dofSize), q_old(dofSize);
	dx.setZero();
	Vector3d phi;
	Vector3d s[3], v[3], w[3];

	for (int i = 0; i < dofSize; i++)
	{ // deg -> rad
		qs[i] = qs[i] * M_PI / 180.0;
		qold[i] = qold[i] * M_PI / 180.0;

		q_old(i) = qold[i] * M_PI / 180.0;
	}

	while (true)
	{
		for (int i = 0; i < dofSize; i++)
			model.q(i) = qs[i];

		Matrix3d Rot_temp = CalcBodyWorldOrientation(model.model_, model.q, model.body_id_vec.back(), true).transpose();
		Vector3d pos_temp = CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec.back(), model.body_com_pos.back()); // get position and rotation in EE frame;

		T0_obj.setIdentity();
		T0_obj.topLeftCorner(3, 3) = Rot_temp;
		T0_obj.topRightCorner(3, 1) = pos_temp;
		Tc_obj = T0_c.inverse() * T0_obj;

		d_c.head(3) = Tc_obj.topRightCorner(3, 1);

		Matrix3d Tc_obj_rot = refer_rot.transpose()*Rot_temp;
		d_c(3) = atan2(Tc_obj_rot(2, 1), Tc_obj_rot(2, 2));
		d_c(4) = -asin(Tc_obj_rot(2, 0));
		d_c(5) = atan2(Tc_obj_rot(1, 0), Tc_obj_rot(0, 0));

		// 위치가 베이스 프레임 기준이 아니게 됨.
		for (int i = 0; i < 6; i++)
		{
			if (d_c(i) > C(i, 0)) // max
				dx(i) = d_c(i) - C(i, 0);
			else if (d_c(i) < C(i, 1)) // min
				dx(i) = d_c(i) - C(i, 1) ;
			else
				dx(i) = 0.0;
		}

		// Algorithm 4 - line 3
		if (dx.norm() < 0.01)
		{
			// Vector3d q_old_pos = CalcBodyToBaseCoordinates(model.model_, q_old, model.body_id_vec.back(), model.body_com_pos.back());
			// double dot_product;

			// dx_from_extend_to_goal.head(3) = refer_pos - pos_temp;
			// dx_from_extend_to_goal.tail(3) = getPhi(Rot_temp, refer_rot);

			// dot_product = (dx_from_init_to_goal.head(3)/dx_from_init_to_goal.head(3).norm()).dot((pos_temp - q_old_pos) / (pos_temp - q_old_pos) .norm());

			// // TODO : focal point - init position, final positon.  projected point is inside the ellipsoid
			// if ((dx_from_init_to_goal.head(3).norm() < dx_from_extend_to_goal.head(3).norm()) || (dot_product < 0.0))
			// {
			// 	qs.clear();
			// 	return qs;
			// }

			for (int i = 0; i < dofSize; i++)
			{
				qs[i] = qs[i] / M_PI * 180.0;
			}
				return qs; // convert rad to angle
		}

		// Algorithm 4 - line 4
		CalcPointJacobian6D(model.model_, model.q, model.body_id_vec.back(), model.body_com_pos.back(), J_temp, true);
		J.topLeftCorner(3, dofSize) = J_temp.bottomLeftCorner(3, dofSize);

		rpy(0) = atan2(Rot_temp(2, 1), Rot_temp(2, 2));
		rpy(1) = -asin(Rot_temp(2, 0));
		rpy(2) = atan2(Rot_temp(1, 0), Rot_temp(0, 0));
		
		E_rpy(0,0) = cos(rpy(2))/cos(rpy(1));
		E_rpy(0,1) = sin(rpy(2))/cos(rpy(1));
		E_rpy(0,2) = 0.0;

		E_rpy(1,0) = sin(rpy(2));
		E_rpy(1,1) = cos(rpy(2));
		E_rpy(1,2) = 0.0;

		E_rpy(2,0) = -cos(rpy(2))*sin(rpy(1))/cos(rpy(1));
		E_rpy(2,1) = sin(rpy(2))*sin(rpy(1))/cos(rpy(1));
		E_rpy(2,2) = 1.0;
		J.bottomLeftCorner(3, dofSize) = E_rpy*J_temp.topLeftCorner(3, dofSize);
		

		// Algorithm 4 - line 5
		q_error = J.transpose() * (J * J.transpose()).inverse() * dx * 0.3;

		// Algorithm 4 -  line 6
		for (int i = 0; i < dofSize; i++)
			qs[i] -= q_error(i);

		// Algorithm 4 - line 7 : stuck here
		if (!checkJointLimit(qs) || (getEuclideanNorm(qs, qold, dofSize) > 3.0*step_size/180.0*M_PI))
		{
			//cout << "joint limit" << "\t" << qs[0]/M_PI*180.0<< "\t" << qs[1]/M_PI*180.0<< "\t" << qs[2]/M_PI*180.0<< "\t" << qs[3]/M_PI*180.0<< "\t" << qs[4]/M_PI*180.0<< "\t" << qs[5]/M_PI*180.0<< "\t" << qs[6]/M_PI*180.0 << endl;
			qs.clear();
			return qs;
			// flag = false;
			// break;
		}
	}
}

// std::vector<double> RRT::ProjectConfigForRandomConfig(Robotmodel model, std::vector<double> &qs)
// {
// 	model.q.resize(dofSize);

// 	// Tc
// 	Matrix4d T0_c, T0_obj, Tc_obj;
// 	T0_c.setIdentity();
// 	//T0_c.topLeftCorner(3,3) = refer_rot;
// 	T0_c.topRightCorner(3, 1) = local_rot*(refer_pos - local_pos); // global to local pos

// 	MatrixXd J_temp(6, dofSize), J(6, dofSize);
// 	VectorXd d_c(6), dx(6), q_error(dofSize);
// 	dx.setZero();
// 	Vector3d phi;
// 	Vector3d s[3], v[3], w[3];

// 	for (int i = 0; i < dofSize; i++)
// 	{ // deg -> rad
// 		qs[i] = qs[i] * M_PI / 180.0;
// 	}

// 	while (true)
// 	{
// 		for (int i = 0; i < dofSize; i++)
// 			model.q(i) = qs[i];

// 		Matrix3d Rot_temp = CalcBodyWorldOrientation(model.model_, model.q, model.body_id_vec.back(), true).transpose();
// 		Vector3d pos_temp = CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec.back(), model.body_com_pos.back()); // get position and rotation in EE frame;

// 		T0_obj.setIdentity();
// 		T0_obj.topLeftCorner(3, 3) = Rot_temp;
// 		T0_obj.topRightCorner(3, 1) = pos_temp;
// 		Tc_obj = T0_c.inverse() * T0_obj;

// 		d_c.head(3) = Tc_obj.topRightCorner(3, 1);

// 		Matrix3d Tc_obj_rot = refer_rot.transpose()*Rot_temp;
// 		d_c(3) = atan2(Tc_obj_rot(2, 1), Tc_obj_rot(2, 2));
// 		d_c(4) = -asin(Tc_obj_rot(2, 0));
// 		d_c(5) = atan2(Tc_obj_rot(1, 0), Tc_obj_rot(0, 0));

// 		for (int i = 0; i < 6; i++)
// 		{
// 			if (d_c(i) > C(i, 0)) // max
// 				dx(i) = d_c(i) - C(i, 0);
// 			else if (d_c(i) < C(i, 1)) // min
// 				dx(i) = d_c(i) - C(i, 1) ;
// 			else
// 				dx(i) = 0.0;
// 		}

// 		// Algorithm 4 - line 3
// 		if (dx.norm() < 0.01)
// 		{
// 			dx_from_extend_to_goal.head(3) = refer_pos - pos_temp;
// 			dx_from_extend_to_goal.tail(3) = getPhi(Rot_temp, refer_rot);

// 			if (dx_from_init_to_goal.norm() < dx_from_extend_to_goal.norm()){
// 				qs.clear();
// 				return qs;
// 			}

// 			for (int i = 0; i < dofSize; i++)
// 			{
// 				qs[i] = qs[i] / M_PI * 180.0;
// 			}
// 				return qs; // convert rad to angle
// 		}

// 		// Algorithm 4 - line 4
// 		CalcPointJacobian6D(model.model_, model.q, model.body_id_vec.back(), model.body_com_pos.back(), J_temp, true);
// 		J.topLeftCorner(3, dofSize) = J_temp.bottomLeftCorner(3, dofSize);

// 		rpy(0) = atan2(Rot_temp(2, 1), Rot_temp(2, 2));
// 		rpy(1) = -asin(Rot_temp(2, 0));
// 		rpy(2) = atan2(Rot_temp(1, 0), Rot_temp(0, 0));
		
// 		E_rpy(0,0) = cos(rpy(2))/cos(rpy(1));
// 		E_rpy(0,1) = sin(rpy(2))/cos(rpy(1));
// 		E_rpy(0,2) = 0.0;

// 		E_rpy(1,0) = sin(rpy(2));
// 		E_rpy(1,1) = cos(rpy(2));
// 		E_rpy(1,2) = 0.0;

// 		E_rpy(2,0) = -cos(rpy(2))*sin(rpy(1))/cos(rpy(1));
// 		E_rpy(2,1) = sin(rpy(2))*sin(rpy(1))/cos(rpy(1));
// 		E_rpy(2,2) = 1.0;
// 		J.bottomLeftCorner(3, dofSize) = E_rpy*J_temp.topLeftCorner(3, dofSize);
		

// 		// Algorithm 4 - line 5
// 		q_error = J.transpose() * (J * J.transpose()).inverse() * dx * 0.3;

// 		// Algorithm 4 -  line 6
// 		for (int i = 0; i < dofSize; i++)
// 			qs[i] -= q_error(i);

// 		// Algorithm 4 - line 7 : stuck here
// 		if (!OutsideJointLimit(qs))
// 		{
// 			//cout << "joint limit" << "\t" << qs[0]/M_PI*180.0<< "\t" << qs[1]/M_PI*180.0<< "\t" << qs[2]/M_PI*180.0<< "\t" << qs[3]/M_PI*180.0<< "\t" << qs[4]/M_PI*180.0<< "\t" << qs[5]/M_PI*180.0<< "\t" << qs[6]/M_PI*180.0 << endl;
// 			qs.clear();
// 			return qs;
// 			// flag = false;
// 			// break;
// 		}
// 	}
// }


double RRT::getDistance(std::vector<double> &a, std::vector<double> &b) {

	//Gets squared Euclidean Distance on C-Space between node and a given
	double accumulated = 0.0f;
	std::vector<double> diff = b;

	for (int i = 0; i< diff.size(); i++) {
		diff[i] -= a[i];
		//double dif = _configuration[i] - (config)[i];
		accumulated += (diff[i] * diff[i] * DOF_weights[i] * DOF_weights[i]);
	}
	return sqrt(accumulated);
}
bool RRT::smoothPath(std::ostream& sout, std::istream& sinput)
{
	std::string fullinput;
	//Parse input
	while (sinput) {
		std::string input;
		sinput >> input;
		fullinput += input;
	}
	int nSmooth = 5; // atoi(fullinput.c_str());

	while (nSmooth) { // while(x) -> while( x != 0 )
		shortcutSmoothing();
		nSmooth--;
	}
	
	for (int i = 0; i< path.size(); i++) {
		std::vector<double> node = path[i];
		for (int j = 0; j<node.size() - 1; j++) {
			sout << node[j] << ",";
		}
		sout << node[node.size() - 1] << "\n";
	}

	return true;
}
bool RRT::shortcutSmoothing() {
	if (path.size() <= 2) 
		return false;
	
	int i = ((double)rand() / RAND_MAX)*path.size();
	int j = ((double)rand() / RAND_MAX)*path.size();
	
	while (abs(j - i) < 2) 
	{
		i = ((double)rand() / RAND_MAX)*path.size();
		j = ((double)rand() / RAND_MAX)*path.size();
	}

	if (j < i) { //make sure i is the smaller number
		int a = j;
		j = i;
		i = a;
	}

	if (checkTraj(path[i], path[j])) {
		path.erase(path.begin() + i + 1, path.begin() + j);
		return true;
	}
	return false;

}

bool RRT::checkTraj(std::vector<double> &a, std::vector<double> &b) {
	std::vector<double> u = getUnitVector(a, b); // a->b
	//cout << a[0] << "\t" << a[1] << "\t" << a[2] << "\t" << a[3] << endl;
	//cout << b[0] << "\t" << b[1] << "\t" << b[2] << "\t" << b[3] << endl;
	std::vector<double> p = a;
	bool check_traj = true;
	//cout << "loop" << endl;
	while (p != b) {
		if (checkExternalCollision(_model, p) || checkSelfCollision(_model, p)) {
			//cout <<"collision \t" << p[0] << "\t" << p[1] << "\t"<< p[2] << "\t"<< p[3] << "\t"<< p[4] << "\t"<< p[5] << "\t"<< p[6] << "\n" << endl; 
			check_traj =  false;
		//break;
		}

		if (getDistance(p, b) < step_size / 2) {
			p = b;
		}
		else {
			for (int i = 0; i < u.size(); i++) 
			{
				p[i] += u[i] * step_size / 2;
			}
			//cout << p[0] << "\t" << p[1] << "\t" << p[2] << "\t" << p[3] << "\t" << p[4] << "\t" << p[5] << "\t" << p[6] << endl;

		}
	}
	return check_traj;
}
std::vector<double> RRT::getUnitVector(std::vector<double> &a, std::vector<double> &b) {

	//Gets squared Euclidean Distance on C-Space between node and a given
	double accumulated = 0.0f;
	std::vector<double> diff = b;

	for (int i = 0; i< diff.size(); i++) {
		diff[i] -= a[i];//double dif = _configuration[i] - (config)[i];
		accumulated += (diff[i] * diff[i] * DOF_weights[i] * DOF_weights[i]);
	}
	accumulated = sqrt(accumulated);
	for (int i = 0; i < diff.size(); i++) {
		diff[i] /= accumulated;
	}
	return diff;
}
bool RRT::checkExternalCollision(Robotmodel model, std::vector<double> &config) {
	int torso_dof = torso_config.size();
	model.q.resize(dofSize + torso_dof);

	if (torso_dof > 0)
	{
		for (int i = 0; i < torso_dof;i++)
			model.q(i) = torso_config[i];

		for (int i = torso_dof; i <  torso_dof + dofSize; i++)
			model.q(i) = config[i - torso_dof] * M_PI / 180.0;
	}
	else
	{
		for (int i = 0; i < dofSize; i++)
			model.q(i) = config[i] * M_PI / 180.0;
	}
	// Update Box modeling for link
	for (int i = 0; i < box_num_link; i++)
	{
		Matrix3d Rot_temp3 = CalcBodyWorldOrientation(model.model_, model.q, model.body_id_vec[i], true).transpose() * Box_robot[i].vRot ;
		Box_robot[i].vAxis[0] = Rot_temp3.col(0);
		Box_robot[i].vAxis[1] = Rot_temp3.col(1);
		Box_robot[i].vAxis[2] = Rot_temp3.col(2);
		Box_robot[i].vPos = CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec[i], (Box_robot[i].vCenter + model.body_com_pos[i]), true);
	}

	bool chk;
	if (box_num_obs == 0)
	{
		chk = false;
		return chk;
	}
	for (int i = 0; i < box_num_obs; i++)
	{
		for (int j = 0; j < box_num_link; j++)
		{
			chk = checkOBBCollision(&Box_env[i], &Box_robot[j]);
			if (chk)
			{
				//cout << "----------------External Collision \n"
				//	 << i << "\t" << j << "\t" << chk << endl;
				//cout << Box_obs[i].vPos.transpose() << endl;
//				cout << Box_link[j].vAxis[0].transpose() << endl;
//				cout << Box_link[j].vAxis[1].transpose() << endl;
//				cout << Box_link[j].vAxis[2].transpose() << endl;
//				cout << Box_link[j].vPos.transpose() << endl;
				// //cout << CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec[j], Vector3d::Zero(), true).transpose() << endl;
				//cout << model.q(0) * 180.0 / M_PI << "\t" << model.q(1) * 180.0 / M_PI << "\t" << model.q(2) * 180.0 / M_PI << "\t" << model.q(3) * 180.0 / M_PI <<
				 //"\t" << model.q(4) * 180.0 / M_PI << "\t" << model.q(5) * 180.0 / M_PI << "\t" << model.q(6) * 180.0 / M_PI << "\t"  << model.q(7) * 180.0 / M_PI << "\t" << endl;

				//cout << i << "\t" << j << "\t" << chk << endl;

				return chk;
			}
			//cout << "collision" << i << "\t" << j << endl;
		}
	}
	return chk;
}


bool RRT::checkSelfCollision(Robotmodel model, std::vector<double> &config) {
	int torso_dof = torso_config.size();
	model.q.resize(dofSize + torso_dof);

	if (torso_dof > 0)
	{
		for (int i = 0; i < torso_dof;i++)
			model.q(i) = torso_config[i];

		for (int i = torso_dof; i <  torso_dof + dofSize; i++)
			model.q(i) = config[i-torso_dof] * M_PI / 180.0;
	}
	else
	{
		for (int i = 0; i < dofSize; i++)
			model.q(i) = config[i] * M_PI / 180.0;
	}
	// Update Box modeling for link
	for (int i = 0; i < box_num_link; i++)
	{
		Matrix3d Rot_temp3 = CalcBodyWorldOrientation(model.model_, model.q, model.body_id_vec[i], true).transpose() * Box_robot[i].vRot ;
		Box_robot[i].vAxis[0] = Rot_temp3.col(0);
		Box_robot[i].vAxis[1] = Rot_temp3.col(1);
		Box_robot[i].vAxis[2] = Rot_temp3.col(2);
		Box_robot[i].vPos = CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec[i], (Box_robot[i].vCenter + model.body_com_pos[i]), true);
	}

	bool chk;
	if (box_num_link == 0)
	{
		chk = false;
		return chk;
	}

	// for torso and other links
	for (int i = 0; i < 2; i++)
	{
		for (int j = 2; j < box_num_link; j++)
		{
			chk = checkOBBCollision(&Box_robot[i], &Box_robot[j]);
			if (chk)
			{
				// cout <<  "----------------Collision \n" << i << "\t" << j << "\t" << chk << endl;
				// cout << Box_robot[i].vPos.transpose() << endl;
				// cout << Box_robot[j].vPos.transpose() << endl;
				// //cout << CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec[j], Vector3d::Zero(), true).transpose() << endl;
				// cout << model.q(0) * 180.0 / M_PI << "\t" << model.q(1) * 180.0 / M_PI << "\t" << model.q(2) * 180.0 / M_PI << "\t" 
				// << model.q(3) * 180.0 / M_PI << "\t" << model.q(4) * 180.0 / M_PI << "\t" << model.q(5) * 180.0 / M_PI << "\t" << model.q(6) * 180.0 / M_PI<< endl;

				return chk;
			}
		}
	}

	// for between other links
	for (int i = 2; i < box_num_link; i++)
	{
		for (int j = i + 2; j < box_num_link; j++)
		{
			chk = checkOBBCollision(&Box_robot[i], &Box_robot[j]);
			if (chk)
			{
				//cout <<  "----------------Collision \n" << i << "\t" << j << "\t" << chk << endl;
//				cout << Box_link[i].vPos.transpose() << endl;
//				cout << Box_link[j].vPos.transpose() << endl;
				//cout << CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec[j], Vector3d::Zero(), true).transpose() << endl;
				//cout << model.q(0) * 180.0 / M_PI << "\t" << model.q(1) * 180.0 / M_PI << "\t" << model.q(2) * 180.0 / M_PI << "\t" 
				//<< model.q(3) * 180.0 / M_PI << "\t" << model.q(4) * 180.0 / M_PI << "\t" << model.q(5) * 180.0 / M_PI << "\t" << model.q(6) * 180.0 / M_PI << "\t" << model.q(7) * 180.0 / M_PI<< endl;
				return chk;
			}
			// cout << "No collision \n" << i << "\t" << j << "\t" << chk << endl;
			// cout << Box_link[i].vPos.transpose() << endl;
			// cout << Box_link[j].vPos.transpose() << endl;
			// 	cout << CalcBodyToBaseCoordinates(model.model_, model.q, model.body_id_vec[j], Vector3d::Zero(), true).transpose() << endl;
			//cout << model.q(0) * 180.0 / M_PI << "\t" << model.q(1) * 180.0 / M_PI << "\t" << model.q(2) * 180.0 / M_PI << "\t" << model.q(3) * 180.0 / M_PI << "\t" << model.q(4) * 180.0 / M_PI << "\t" << model.q(5) * 180.0 / M_PI << "\t" << model.q(6) * 180.0 / M_PI << "\t" << endl;
		}
	}
	return chk;
}


bool RRT::checkJointLimit(std::vector<double> q) { // radian
	for (int i = 0;i < q.size();i++) {
		if (q[i] > upper_limit(i)/180.0*M_PI) {
			//cout << q[i] << "\t" << upper_limit(i)/180.0*M_PI << "\t" << i << endl;
			return false;
		}
		else if (q[i] < lower_limit(i)/180.0*M_PI) {
			//cout << q[i] << "\t" << lower_limit(i)/180.0*M_PI  << "\t" << i << endl;

			return false;
		}
	}
	return true;

}

bool RRT::checkOBBCollision(ST_OBB* box0, ST_OBB* box1) // Collision-free : False
{
	// compute difference of box centers,D=C1-C0
	Vector3d D = Vector3d(box1->vPos(0) - box0->vPos(0), box1->vPos(1) - box0->vPos(1), box1->vPos(2) - box0->vPos(2));

	float C[3][3];    //matrix C=A^T B,c_{ij}=Dot(A_i,B_j)
	float absC[3][3]; //|c_{ij}|
	float AD[3];      //Dot(A_i,D)
	float R0, R1, R;    //interval radii and distance between centers
	float R01;        //=R0+R1

					  //A0
	C[0][0] = FDotProduct(box0->vAxis[0], box1->vAxis[0]);// vAxis : direction // 3D Dot product
	C[0][1] = FDotProduct(box0->vAxis[0], box1->vAxis[1]);
	C[0][2] = FDotProduct(box0->vAxis[0], box1->vAxis[2]);
	AD[0] = FDotProduct(box0->vAxis[0], D);
	absC[0][0] = (float)fabsf(C[0][0]);
	absC[0][1] = (float)fabsf(C[0][1]);
	absC[0][2] = (float)fabsf(C[0][2]);
	R = (float)fabsf(AD[0]);
	R1 = box1->fAxis(0) * absC[0][0] + box1->fAxis(1) * absC[0][1] + box1->fAxis(2) * absC[0][2];
	R01 = box0->fAxis(0) + R1;
	if (R > R01)
		return 0;

	//A1
	C[1][0] = FDotProduct(box0->vAxis[1], box1->vAxis[0]);
	C[1][1] = FDotProduct(box0->vAxis[1], box1->vAxis[1]);
	C[1][2] = FDotProduct(box0->vAxis[1], box1->vAxis[2]);
	AD[1] = FDotProduct(box0->vAxis[1], D);
	absC[1][0] = (float)fabsf(C[1][0]);
	absC[1][1] = (float)fabsf(C[1][1]);
	absC[1][2] = (float)fabsf(C[1][2]);
	R = (float)fabsf(AD[1]);
	R1 = box1->fAxis(0) * absC[1][0] + box1->fAxis(1) * absC[1][1] + box1->fAxis(2) * absC[1][2];
	R01 = box0->fAxis(1) + R1;
	if (R > R01)
		return 0;

	//A2
	C[2][0] = FDotProduct(box0->vAxis[2], box1->vAxis[0]);
	C[2][1] = FDotProduct(box0->vAxis[2], box1->vAxis[1]);
	C[2][2] = FDotProduct(box0->vAxis[2], box1->vAxis[2]);
	AD[2] = FDotProduct(box0->vAxis[2], D);
	absC[2][0] = (float)fabsf(C[2][0]);
	absC[2][1] = (float)fabsf(C[2][1]);
	absC[2][2] = (float)fabsf(C[2][2]);
	R = (float)fabsf(AD[2]);
	R1 = box1->fAxis(0) * absC[2][0] + box1->fAxis(1) * absC[2][1] + box1->fAxis(2) * absC[2][2];
	R01 = box0->fAxis(2) + R1;
	if (R > R01)
		return 0;

	//B0
	R = (float)fabsf(FDotProduct(box1->vAxis[0], D));
	R0 = box0->fAxis(0) * absC[0][0] + box0->fAxis(1) * absC[1][0] + box0->fAxis(2) * absC[2][0];
	R01 = R0 + box1->fAxis(0);
	if (R > R01)
		return 0;

	//B1
	R = (float)fabsf(FDotProduct(box1->vAxis[1], D));
	R0 = box0->fAxis(0) * absC[0][1] + box0->fAxis(1) * absC[1][1] + box0->fAxis(2) * absC[2][1];
	R01 = R0 + box1->fAxis(1);
	if (R > R01)
		return 0;

	//B2
	R = (float)fabsf(FDotProduct(box1->vAxis[2], D));
	R0 = box0->fAxis(0) * absC[0][2] + box0->fAxis(1) * absC[1][2] + box0->fAxis(2) * absC[2][2];
	R01 = R0 + box1->fAxis(2);
	if (R > R01)
		return 0;

	//A0xB0
	R = (float)fabsf(AD[2] * C[1][0] - AD[1] * C[2][0]);
	R0 = box0->fAxis(1) * absC[2][0] + box0->fAxis(2) * absC[1][0];
	R1 = box1->fAxis(1) * absC[0][2] + box1->fAxis(2) * absC[0][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A0xB1
	R = (float)fabsf(AD[2] * C[1][1] - AD[1] * C[2][1]);
	R0 = box0->fAxis(1) * absC[2][1] + box0->fAxis(2) * absC[1][1];
	R1 = box1->fAxis(0) * absC[0][2] + box1->fAxis(2) * absC[0][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A0xB2
	R = (float)fabsf(AD[2] * C[1][2] - AD[1] * C[2][2]);
	R0 = box0->fAxis(1) * absC[2][2] + box0->fAxis(2) * absC[1][2];
	R1 = box1->fAxis(0) * absC[0][1] + box1->fAxis(1) * absC[0][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB0
	R = (float)fabsf(AD[0] * C[2][0] - AD[2] * C[0][0]);
	R0 = box0->fAxis(0) * absC[2][0] + box0->fAxis(2) * absC[0][0];
	R1 = box1->fAxis(1) * absC[1][2] + box1->fAxis(2) * absC[1][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB1
	R = (float)fabsf(AD[0] * C[2][1] - AD[2] * C[0][1]);
	R0 = box0->fAxis(0) * absC[2][1] + box0->fAxis(2) * absC[0][1];
	R1 = box1->fAxis(0) * absC[1][2] + box1->fAxis(2) * absC[1][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB2
	R = (float)fabsf(AD[0] * C[2][2] - AD[2] * C[0][2]);
	R0 = box0->fAxis(0) * absC[2][2] + box0->fAxis(2) * absC[0][2];
	R1 = box1->fAxis(0) * absC[1][1] + box1->fAxis(1) * absC[1][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB0
	R = (float)fabsf(AD[1] * C[0][0] - AD[0] * C[1][0]);
	R0 = box0->fAxis(0) * absC[1][0] + box0->fAxis(1) * absC[0][0];
	R1 = box1->fAxis(1) * absC[2][2] + box1->fAxis(2) * absC[2][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB1
	R = (float)fabsf(AD[1] * C[0][1] - AD[0] * C[1][1]);
	R0 = box0->fAxis(0) * absC[1][1] + box0->fAxis(1) * absC[0][1];
	R1 = box1->fAxis(0) * absC[2][2] + box1->fAxis(2) * absC[2][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB2
	R = (float)fabsf(AD[1] * C[0][2] - AD[0] * C[1][2]);
	R0 = box0->fAxis(0) * absC[1][2] + box0->fAxis(1) * absC[0][2];
	R1 = box1->fAxis(0) * absC[2][1] + box1->fAxis(1) * absC[2][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	return 1;
}

MatrixXd RRT::MergeRRTResults(MatrixXd joint_target1, MatrixXd joint_target2, int case_){
	// case 0 : RRT / RRT
	// case 1 : RRT / CBiRRT
	// case 2 : CBiRRT / RRT
	// case 3 : CBiRRT / CBiRRT 
	MatrixXd merged_joint_target;
	int row[2];
	int reminder[2];
	switch(case_ )
	{
		case 0 :
			row[0] = joint_target1.rows();
			row[1] = joint_target2.rows();
			merged_joint_target.resize(row[0] + row[1] -1, dofSize);
			merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
			merged_joint_target.bottomRows(row[1]-1) = joint_target2.bottomRows(row[1]-1);
		break;
		case 1 :
			row[0] = joint_target1.rows();
			row[1] = joint_target2.rows() / 20;
			reminder[1] = joint_target2.rows() % 20;
			if (reminder[1] == 0)
			{
				merged_joint_target.resize(row[0] + row[1], dofSize);
				merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
				for (int i = 0; i < row[1]; i++)
					merged_joint_target.row(row[0] + i) = joint_target2.row(20 * (i + 1) - 1);
			}
			else
			{
				merged_joint_target.resize(row[0] + row[1] + 1,dofSize);
				merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
				for (int i = 0; i < row[1]; i++)
					merged_joint_target.row(row[0] + i) = joint_target2.row(20 * (i + 1)- 1);

				merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
			}
			break; 
		case 2 :
			row[0] = joint_target1.rows() /20;
			row[1] = joint_target2.rows() ;
			reminder[0] = joint_target1.rows() % 20;
			if (reminder[0] == 0)
			{
				merged_joint_target.resize(row[0] + row[1],dofSize);
				for (int i = 0; i < row[0]; i++)
					merged_joint_target.row(i) = joint_target1.row(20 * (i));

				merged_joint_target.bottomRows(row[1]) = joint_target2.bottomRows(row[1]);
			}
			else{
				merged_joint_target.resize(row[0] + row[1] + 1,dofSize);
				for (int i = 0; i < row[0] + 1; i++)
					merged_joint_target.row(i) = joint_target1.row(20 * (i));

				merged_joint_target.bottomRows(row[1]) = joint_target2.bottomRows(row[1]);
			}
		break;
		case 3 :
			row[0] = joint_target1.rows() /20;
			row[1] = joint_target2.rows() /20;
			reminder[0] = joint_target1.rows() % 20;
			reminder[1] = joint_target2.rows() % 20;
			if (reminder[0] == 0){
				if (reminder[1] == 0){
					merged_joint_target.resize(row[0] + row[1] + 1,dofSize);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 1 + i) = joint_target2.row(20 * (i+1));
				}
				else{
					merged_joint_target.resize(row[0] + row[1] + 2,dofSize);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 1 + i) = joint_target2.row(20 * (i+1));

					merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
				}
			}
			else{
				if (reminder[1] == 0){
					merged_joint_target.resize(row[0] + row[1] + 2,dofSize);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					merged_joint_target.row(row[0]+1) = joint_target1.bottomRows(1);

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 2 + i) = joint_target2.row(20 * (i+1));
				}
				else{
					merged_joint_target.resize(row[0] + row[1] + 3,dofSize);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					merged_joint_target.row(row[0]+1) = joint_target1.bottomRows(1);

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 2 + i) = joint_target2.row(20 * (i+1));

					merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
				}
			}
		break;
	}


	return merged_joint_target;
}
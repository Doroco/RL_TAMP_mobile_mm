#ifndef __NODETREE__
#define __NODETREE__

#include "RRTNode.h"

class NodeTree
{
public:
	std::vector<RRTNodePtr> _nodes;
	
	RRTNodePtr getNode(int index) { return _nodes[index]; }
	void addNode(RRTNodePtr node) { _nodes.push_back(node); }
	void removeNode(int index) { _nodes.erase(_nodes.begin() + index); }
	std::vector<std::vector<double> > getPath() 
	{
		std::vector<std::vector<double> > path;
		RRTNodePtr node = _nodes.back(); //start at goal
		std::cout << "node size " <<_nodes.size() << std::endl;
		//std::vector<double> config;
		while (node != nullptr) {
			//config = node->getConfiguration();
			//std::cout << "config" <<config[0] <<"\t"  <<config[1] <<"\t" <<config[2] <<"\t" <<config[3] <<"\t" <<config[4] <<"\t" <<config[5] <<"\t" <<config[6] <<std::endl;
			path.push_back(node->getConfiguration());
			node = node->getParent();
		}

		std::reverse(path.begin(), path.end());
		std::cout << "path size" <<path.size() << std::endl;
		return path;
	}

	std::vector<std::vector<double> > getPathWithIndex(int index) 
	{
		std::vector<std::vector<double> > path;
		RRTNodePtr node = _nodes[index]; //start at goal
		std::cout << "node size " <<_nodes.size() << std::endl;
		//std::vector<double> config;
		while (node != nullptr) {
			//config = node->getConfiguration();
			//std::cout << "config" <<config[0] <<"\t"  <<config[1] <<"\t" <<config[2] <<"\t" <<config[3] <<"\t" <<config[4] <<"\t" <<config[5] <<"\t" <<config[6] <<std::endl;
			path.push_back(node->getConfiguration());
			node = node->getParent();
		}

		std::reverse(path.begin(), path.end());
		std::cout << "path size" <<path.size() << std::endl;
		return path;
	}

	const std::vector<RRTNodePtr> & getTree() const {
		return _nodes;
	}

	RRTNodePtr getNearest(std::vector<double> &config, std::vector<double> &w) {
		int min = 0;
		double mindist = _nodes[0]->getDistance(config, w);
		//std::cout << "_nodes.size()" << _nodes.size() << std::endl;
		for (int i = 1; i< _nodes.size(); i++) { // Ignoring index 0 as it is preselected
			double current = _nodes[i]->getDistance(config, w);
			if (current < mindist) {
				min = i;
				mindist = current;
			}
		}
		return getNode(min);
	}

	int getNearestIndex(std::vector<double> &config, std::vector<double> &w) {
		int min = 0;
		double mindist = _nodes[0]->getDistance(config, w);
		//std::cout << "_nodes.size()" << _nodes.size() << std::endl;
		for (int i = 1; i< _nodes.size(); i++) { // Ignoring index 0 as it is preselected
			double current = _nodes[i]->getDistance(config, w);
			if (current < mindist) {
				min = i;
				mindist = current;
			}
		}
		return min;
	}


};
#endif
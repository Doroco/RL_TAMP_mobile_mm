#pragma once 
#ifndef __RRTNODE__
#define __RRTNODE__

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace ArmPlanner{

    using namespace Eigen;

    class RRTNode;
    typedef std::shared_ptr<RRTNode> RRTNodePtr;

    class RRTNode {

    public:
        std::vector<double> _configuration; //Vector for the joint values in the configuration
        RRTNodePtr _parent;//Pointer to Parent node on RRT Tree

        RRTNode() {}
        RRTNode(std::vector<double> &configuration) : _configuration(configuration) {}
        RRTNode(std::vector<double> &configuration, RRTNodePtr parent) : _configuration(configuration), _parent(parent) {}
        std::vector<double> getConfiguration() {
            return _configuration;
        };
        void setConfiguration(std::vector<double> &configuration) {
            _configuration = configuration;
        };
        double getDistance(std::vector<double> &config, std::vector<double> &w) {
            double accumulated = 0.0f;


            for (int i = 0; i < _configuration.size(); i++) {
                accumulated += (_configuration[i] - config[i]) * (_configuration[i] - config[i]) *w[i] * w[i];

                //std::cout << i << "\t" << _configuration[i] << std::endl;
            }
            return accumulated;
        }
        RRTNodePtr getParent() { return _parent; }
        
    };
}
#endif

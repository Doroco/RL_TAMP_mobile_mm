#include <iostream>
#include <string>
#include "RosVrepBridge.h"
#include "Controller.h"
#include "task_assembly/sim_request.h"

#include <mutex>
#include <thread>

using namespace std;
class RosVrepBridge;
typedef std::shared_ptr<RosVrepBridge> RosVrepBridgePtr;



class RosPlanningInterface
{
public:

	enum ManipulabilityIndexType
	{
		RUN_SIMULATION,    
		STOP_SIMULATION,
		QUIT_SIMULATION,
		RESTART_SIMULATION    
	};

    RosPlanningInterface(ros::NodeHandle & nh, const double hz)
    {
		thread_condition  = false;
		isSimulationRun = true;
		exitFlag = false;	
		RosVrepBridgePtr cpoy(new RosVrepBridge(nh,hz));
		rb = cpoy;
		
		// init Sim state/////////////////
		sleep(1);
		rb->vrepStart();
		sleep(1);
		rb->vrepEnableSyncMode();
		sleep(1);

		// msgs init
		threads.push_back(std::thread(&RosPlanningInterface::sim_loop, this));
        planning_server_ =  nh.advertiseService("/TAMP_control_sim", &RosPlanningInterface::excute, this);
    }

	~RosPlanningInterface()
	{
		thread_condition = true;
		for(auto & thread : threads)
		{   
			thread.join();
		}
	}
	
private:
    bool excute(task_assembly::sim_request::Request &req, task_assembly::sim_request::Response &res)
    {
		
		switch (req.mode.data)
		{
		case RUN_SIMULATION:
			cout << "Simulation Run" << endl;
			mutex_.lock();
			isSimulationRun = true;
			mutex_.unlock();
			break;
		case STOP_SIMULATION:
			cout << "DO Arm Motion Planning" <<endl;
			mutex_.lock();
			isSimulationRun = false;
			mutex_.unlock();
			break;
		case RESTART_SIMULATION:
			cout << "Plane Mobile Motion Planning" << endl;
			sleep(1);
			rb->vrepStart();
			sleep(1);
			rb->vrepEnableSyncMode();
			sleep(1);
			break;
		case QUIT_SIMULATION:
			mutex_.lock();
			rb->vrepStop();
			isSimulationRun = false;
			exitFlag = true;
			mutex_.unlock();
			break;
		default :
			ROS_ERROR("receive undefined mode!... ");
			res.excution_state.data = false;
			return false;
			break;
		}
		res.excution_state.data = true;
		return true;
	}

	void sim_loop()
	{
		while (!thread_condition && !exitFlag)
		{
			rb->vrepRead();
			ac.readData(rb->current_ql_, rb->current_qr_, rb->current_ql_dot_, rb->current_qr_dot_, rb->current_base_pose_, rb->current_base_twist_);
			
			if (isSimulationRun) {
				// if(rb.left_traj_cb_ || rb.right_traj_cb_)
				// 	ac.getArmTrajectory(rb.desired_ql_traj, rb.desired_qr_traj);
				ac.checkTrajectory(rb->get_left_arm_trajectory, rb->get_right_arm_trajectory, rb->get_mobile_trajectory);
				ac.getTrajectory(rb->desired_ql_traj_, rb->desired_qr_traj_, rb->desired_mob_traj_);
				ac.compute();

				ac.writeData(rb->desired_ql_, rb->desired_qr_, rb->desired_base_vel_);
				rb->vrepWrite();
				
				rb->wait();	
			}
		}
	}

	std::vector<std::thread> threads;
	std::mutex mutex_;

	bool isSimulationRun;
	bool exitFlag; 
	bool thread_condition;
	RosVrepBridgePtr rb;
	Controller ac;
	ros::ServiceServer planning_server_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mobile_manipulator_controller");
	ros::NodeHandle nh("~");
	
	const double hz = 100;
	RosPlanningInterface base_pose_sampler_interface(nh,hz);
	while (ros::ok());
	return 0;
}
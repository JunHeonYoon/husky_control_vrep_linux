#include "vrep_bridge.h"

VRepBridge::VRepBridge()
{
	simInit();
	getHandle();
}
VRepBridge::~VRepBridge()
{
	simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
	simxFinish(clientID_);
}

bool VRepBridge::simConnectionCheck()
{
	return (simxGetConnectionId(clientID_) != -1);
}
void VRepBridge::simLoop()
{
	tick_++;
	simxSynchronousTrigger(clientID_);
}
void VRepBridge::simxErrorCheck(simxInt error)
{
	string errorMsg;
	switch (error)
	{
	case simx_error_noerror:
		return;	// no error
		break;
	case simx_error_timeout_flag:
		errorMsg = "The function timed out (probably the network is down or too slow)";
		break;
	case simx_error_illegal_opmode_flag:
		errorMsg = "The specified operation mode is not supported for the given function";
		break;
	case simx_error_remote_error_flag:
		errorMsg = "The function caused an error on the server side (e.g. an invalid handle was specified)";
		break;
	case simx_error_split_progress_flag:
		errorMsg = "The communication thread is still processing previous split command of the same type";
		break;
	case simx_error_local_error_flag:
		errorMsg = "The function caused an error on the client side";
		break;
	case simx_error_initialize_error_flag:
		errorMsg = "simxStart was not yet called";
		break;
	default:
		errorMsg = "Unknown error.";
		break;
	}

	cout << "[ERROR] An error is occured. code = " << error << endl;
	cout << " - Description" << endl;
	cout << " | " << errorMsg << endl;

	throw std::string(errorMsg);
}

void VRepBridge::simInit()
{
	simxFinish(-1);
	clientID_ = simxStart("127.0.0.1", -3, true, true, 2000, 5);
	if (clientID_ < 0)
	{
		throw std::string("Failed connecting to remote API server. Exiting.");
	}

	simxErrorCheck(simxStartSimulation(clientID_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxSynchronous(clientID_, true));

	cout << "[INFO] V-Rep connection is established." << endl;

}

void VRepBridge::write()
{
	for (size_t i = 0; i < DOF; i++)
	{
		simxSetJointTargetVelocity(clientID_, motorHandle_[i], desired_wheel_vel_(i), simx_opmode_streaming);
	}
}

void VRepBridge::read()
{
	for (size_t i = 0; i < DOF; i++)
	{
		simxFloat data;
		simxGetObjectFloatParameter(clientID_, motorHandle_[i], 2012, &data, simx_opmode_streaming);
		current_wheel_vel_(i) = data;
	}
	simxFloat posi[3];
	simxFloat ori[3]; // roll, pitch, yaw


	simxGetObjectPosition(clientID_, baseHandle_, -1, posi, simx_opmode_streaming);
	simxGetObjectOrientation(clientID_, baseHandle_, -1, ori, simx_opmode_streaming);
	current_pose_ << posi[0], posi[1], ori[2];

	simxFloat lin_vel[3];
	simxFloat ang_vel[3];
	simxGetObjectVelocity(clientID_, baseHandle_, lin_vel, ang_vel, simx_opmode_streaming);
	current_vel_ << lin_vel[0]*cos(ori[2]) + lin_vel[1]*sin(ori[2]), ang_vel[2];
}

Eigen::Matrix<double, DOF, 1> VRepBridge::IK(const Eigen::Matrix<double, 2, 1> &desired_vel)
{
	double wheel_separation = 0.5708;
	double wheel_radius = 0.1651;
	double wheel_separation_multiplier = 1.875;
	double wheel_radius_multiplier = 1.0;

	double max_lin_vel = 1.0;
	double max_ang_vel = 2.0;

	double target_lin_vel = std::max( std::min(max_lin_vel, desired_vel(0)), -max_lin_vel);
	double target_ang_vel = std::max( std::min(max_ang_vel, desired_vel(1)), -max_ang_vel);

	double left_wheel_vel  = (target_lin_vel - target_ang_vel * (wheel_separation*wheel_separation_multiplier)/2 ) / (wheel_radius*wheel_radius_multiplier);
	double right_wheel_vel = (target_lin_vel + target_ang_vel * (wheel_separation*wheel_separation_multiplier)/2 ) / (wheel_radius*wheel_radius_multiplier);

	Eigen::Matrix<double, DOF, 1> desired_wheel_vel;
	desired_wheel_vel << left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel;
	
	return desired_wheel_vel;
}


void VRepBridge::setDesiredVelocity(const Eigen::Matrix<double, 2, 1>& desired_vel)
{
	desired_vel_ = desired_vel;
	desired_wheel_vel_ = IK(desired_vel_);
}



const Eigen::Matrix<double, 3, 1>& VRepBridge::getPosition()
{
	return current_pose_;
}

const Eigen::Matrix<double, 2, 1>& VRepBridge::getVelocity()
{
	return current_vel_;
}



void VRepBridge::getHandle()
{
	const string base_name = "base_link_respondable";
	const string joint_name1 = "front_left_wheel";
	const string joint_name2 = "front_right_wheel";
	const string joint_name3 = "rear_left_wheel";
	const string joint_name4 = "rear_right_wheel";

	cout << "[INFO] Getting a handle named " << base_name << endl;
	simxErrorCheck(simxGetObjectHandle(clientID_, base_name.c_str(), &baseHandle_, simx_opmode_oneshot_wait));
	cout << "[INFO] Getting a handle named " << joint_name1 << endl;
	simxErrorCheck(simxGetObjectHandle(clientID_, joint_name1.c_str(), &motorHandle_[0], simx_opmode_oneshot_wait));
	cout << "[INFO] Getting a handle named " << joint_name2 << endl;
	simxErrorCheck(simxGetObjectHandle(clientID_, joint_name2.c_str(), &motorHandle_[1], simx_opmode_oneshot_wait));
	cout << "[INFO] Getting a handle named " << joint_name3 << endl;
	simxErrorCheck(simxGetObjectHandle(clientID_, joint_name3.c_str(), &motorHandle_[2], simx_opmode_oneshot_wait));
	cout << "[INFO] Getting a handle named " << joint_name4 << endl;
	simxErrorCheck(simxGetObjectHandle(clientID_, joint_name4.c_str(), &motorHandle_[3], simx_opmode_oneshot_wait));

	cout << "[INFO] The handle has been imported." << endl;
}

#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>
#include "math.h"

using namespace std;

extern "C" {
#include "extApi.h"
}

class VRepBridge
{
public:

	VRepBridge();
	~VRepBridge();
	
	bool simConnectionCheck();
	void simLoop();

	void write();
	void read();

	Eigen::Matrix<double, DOF, 1> IK(const Eigen::Matrix<double, 2, 1> &desired_vel);
	void setDesiredVelocity(const Eigen::Matrix<double, 2, 1> & desired_vel);
	const Eigen::Matrix<double, 3, 1> & getPosition();  // x, y, yaw
	const Eigen::Matrix<double, 2, 1> & getVelocity();  // vx, w

	const size_t getTick() { return tick_; }

private:
	Eigen::Matrix<double, 3, 1> current_pose_;
	Eigen::Matrix<double, 2, 1> desired_vel_;
	Eigen::Matrix<double, 2, 1> current_vel_;
	Eigen::Matrix<double, DOF, 1> desired_wheel_vel_;
	Eigen::Matrix<double, DOF, 1> current_wheel_vel_;
	

	simxInt clientID_;
	simxInt motorHandle_[DOF];	/// < Depends on simulation envrionment
	simxInt baseHandle_;

	size_t tick_{ 0 };

	void simxErrorCheck(simxInt error);
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};

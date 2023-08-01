#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class MobileController
{
	size_t dof_;

	// Current state
	Vector3d pose_; // x, y, theta
	Vector2d vel_; // vx, w

	// Control value (position controlled)
	Vector2d vel_desired_; // Control value


	unsigned long tick_;
	double play_time_;
	double hz_;
	double control_start_time_;
	double control_start_tick_;

	std::string control_mode_;
	bool is_mode_changed_;

	std::vector<MatrixXd> traj_; // [x, y, theta, vx, vy, w]; x,y,theta in global frame; vx,vy,w in robot frame

private:
	void printState();
	void setDesiredVelocity(const Vector2d & desired_vel, double duration);
	void KanayamaController(const int traj_type);

public:
	void readData(const Vector3d &pose, const Vector2d &velocity);
	const Vector2d & getDesiredVelocity();

public:
		MobileController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
			initDimension(); initFile(); readTrajFile();
	}


    void setMode(const std::string & mode);
    void initDimension();
	void initFile();
	void readTrajFile();
    void initPosition();
    void compute();
private:
	ofstream debug_file_;
	constexpr static int NUM_PLOT{20};
	ofstream plot_files_[NUM_PLOT];
	const string plot_file_names_[NUM_PLOT]
	{"circle, square, eight"};
	constexpr static int NUM_TRAJ_FILE{3};
	const string traj_file_names_[NUM_TRAJ_FILE]
	{"circle", "square", "eight"}; 

	void record(int file_number, double duration);
};

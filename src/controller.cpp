#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>



void MobileController::compute()
{

	if (is_mode_changed_)
	{
		is_mode_changed_ = false;
		control_start_time_ = play_time_;
	}

	if (control_mode_ == "velocity_command")
	{
		Vector2d target_vel;
		target_vel << 0.25, 0.125;
		setDesiredVelocity(target_vel, 2*2*M_PI/0.25);
	}
	
	else
	{
		vel_desired_.setZero();
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}

void MobileController::record(int file_number, double duration)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		plot_files_[file_number] << play_time_ - control_start_time_ << " " <<
									pose_.transpose() << " " <<
									vel_.transpose() <<
									endl;
	}
}




void MobileController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 50.)
	{
		DBG_CNT = 0;

		cout << "-------------------------------------------------------" << endl;
		cout << "time        : " << std::fixed << std::setprecision(3) << play_time_ << endl;
		cout << "pose now    :\t";
		cout << std::fixed << std::setprecision(3) << pose_.transpose() << endl;
		cout << "vel desired :\t";
		cout << std::fixed << std::setprecision(3) << vel_desired_.transpose() << endl;
		cout << "vel         :\t";
		cout << vel_.transpose() << endl;
		cout << "-------------------------------------------------------" << endl;
		
	}
}

void MobileController::setDesiredVelocity(const Vector2d & desired_vel, double duration)
{
	if(play_time_ < control_start_time_ + duration)
	{
		vel_desired_ = desired_vel;
	}
	else
	{
		vel_desired_.setZero();
	}
	
	record(0, duration);
}
// Controller Core Methods ----------------------------

void MobileController::setMode(const std::string & mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}
void MobileController::initDimension()
{
	dof_ = DOF;
	vel_desired_.setZero();
	pose_.setZero();
	vel_.setZero();
}

void MobileController::initFile()
{
	debug_file_.open("debug.txt");
	for (int i = 0; i < NUM_PLOT; i++)
	{
		plot_files_[i].open(plot_file_names_[i] + ".txt");
	}
}

void MobileController::readData(const Vector3d &pose, const Vector2d &velocity)
{
	for (size_t i = 0; i < 3; i++)
	{
		pose_(i) = pose(i);
	}
	for (size_t i = 0; i < 2; i++)
	{
		vel_(i) = velocity(i);
	}
}

const Vector2d & MobileController::getDesiredVelocity()
{
	return vel_desired_;
}

void MobileController::initPosition()
{
    vel_desired_ = vel_;
}

// ----------------------------------------------------


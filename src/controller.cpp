#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>

enum Traj_Type
{
	CIRCLE,
	SQUARE,
	EIGHT
};

void MobileController::compute()
{

	if (is_mode_changed_)
	{
		is_mode_changed_ = false;
		control_start_time_ = play_time_;
		control_start_tick_ = tick_;
	}

	if (control_mode_ == "velocity_command")
	{
		Vector2d target_vel;
		target_vel << 0.25, 0.125;
		setDesiredVelocity(target_vel, 2*2*M_PI/0.25);
	}
	else if(control_mode_ == "Kanayama_circle")
	{
		KanayamaController(CIRCLE);
	}
	else if(control_mode_ == "Kanayama_square")
	{
		KanayamaController(SQUARE);
	}
	else if(control_mode_ == "Kanayama_eight")
	{
		KanayamaController(EIGHT);
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

void MobileController::KanayamaController(const int traj_type)
{
	unsigned long time_index = tick_ - control_start_tick_;
	if(time_index >= traj_[traj_type].rows()) time_index = traj_[traj_type].rows() - 1;
	
	double Kx = 9;
	double Ky = 9;
	double Kth = 4.5;

	Vector3d ref_pose, error_pose; // x, y, theta wrt global frame
	Vector3d ref_vel; // vx, vy, w wrt robot frame
	Matrix3d Rz; // rotation matrix along z-axis

	ref_pose = traj_[traj_type].block(time_index, 0, 1, 3).transpose();
	ref_vel  = traj_[traj_type].block(time_index, 3, 1, 3).transpose();
	Rz << cos(pose_(2)), sin(pose_(2)), 0, 
		 -sin(pose_(2)), cos(pose_(2)), 0,
		  0,             0,             1;

	// Error pose wrt Robot coordinate
	error_pose = Rz * (ref_pose - pose_);
	error_pose(2) = atan2(sin(error_pose(2)), cos(error_pose(2))); // angle error correction

	// Velocity getting by Kanayama Eqn
	Vector2d opt_velocity;
	opt_velocity(0) = ( ref_vel(0) * cos(error_pose(2)) ) + ( Kx * error_pose(0) );
	opt_velocity(1) = ref_vel(2) + ref_vel(0) * ( ( Ky * error_pose(1) ) + ( Kth * sin(error_pose(2)) ) );
	// opt_velocity(0) = ref_vel(0);
	// opt_velocity(1) = ref_vel(2);

	vel_desired_ = opt_velocity;

	record(traj_type, double(traj_[traj_type].rows() / hz_));
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

void MobileController::readTrajFile()
{
	traj_.resize(NUM_TRAJ_FILE);
	std::string file_path = "../traj/";
	for(int file_idx = 0; file_idx < NUM_TRAJ_FILE; file_idx++)
	{
		FILE *traj_file = NULL;
		traj_file = fopen( (file_path + traj_file_names_[file_idx] + ".txt").c_str(), "r" );
		int traj_length = 0;
		char tmp;

		if (traj_file == NULL)
		{
			std::cout<<"There is no txt file named: "<< traj_file_names_[file_idx] <<". Please edit code."<<std::endl;
			break;
		}

		while (fscanf(traj_file, "%c", &tmp) != EOF)
		{
			if (tmp == '\n')
				traj_length++;
		}

		fseek(traj_file, 0L, SEEK_SET);

		traj_[file_idx].setZero(traj_length, 6);

		for(int i = 0; i < traj_length; i++)
		{
			fscanf(traj_file, "%lf %lf %lf %lf %lf %lf \n",
					&traj_[file_idx](i,0),
					&traj_[file_idx](i,1),
					&traj_[file_idx](i,2),
					&traj_[file_idx](i,3),
					&traj_[file_idx](i,4),
					&traj_[file_idx](i,5));
		}
		fclose(traj_file);
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


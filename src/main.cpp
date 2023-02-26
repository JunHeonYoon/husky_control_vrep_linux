#include <iostream>
#include <string>
#include "vrep_bridge.h"

#include "controller.h"

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;
 
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
	ungetc(ch, stdin);
	return 1;
	}

	return 0;
}


int main()
{
	VRepBridge vb; 
	const double hz = 50;
	MobileController mc(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		vb.read();
		mc.readData(vb.getPosition(), vb.getVelocity());
		if (is_first)
		{
			vb.simLoop();
			vb.read();
			mc.readData(vb.getPosition(), vb.getVelocity());
			cout << "Initial pose: " << vb.getPosition().transpose() << endl;
			is_first = false;
			mc.initPosition();
		}

		if (kbhit())
		{
			int key = getchar();
			switch (key)
			{
				// Implement with user input
			case '1':
				mc.setMode("velocity_command");
				break;
			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			mc.compute();
			vb.setDesiredVelocity(mc.getDesiredVelocity());
		
			vb.write();
			vb.simLoop();
		}
	}
		
	return 0;
}

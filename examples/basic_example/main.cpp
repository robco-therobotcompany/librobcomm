#include <robcomm/robcomm.hpp>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <vector>

bool keepRunning = true;
robcomm::Robot robot("sn23-1000360", 25002, 25000);


void print_robot_status(robcomm::RobotStatus status) {
	printf("State: %#04x\n", status.robot_state);
}

void rx_task() {
    robcomm::RobotStatus status = {};
    while(keepRunning) {
        robot.receive();

        robcomm::RobotStatus new_status = robot.get_status();
        if (new_status.robot_state != status.robot_state) {
            status = new_status;
            print_robot_status(status);
        }

        usleep(1e3);
    }
}

void jog_trapezoid(int ms, bool dir, double maxvel) {
    std::vector<double> dq_cmd = { -.0001, .0, .0, .0, .0 };
    int cycles = ms/10;
    double step = -(maxvel - .0001)/(cycles/2);

    if (dir) {
       dq_cmd[0] = -dq_cmd[0];
       step = -step;
    }

    for( int i = 0; i < cycles; i ++) {
        robot.jog_joints(dq_cmd);

        if (i < cycles/2) dq_cmd[0] += step;
        else dq_cmd[0] -= step;

        usleep(1e4);
    }
}

int main(int argc, char** argv) {
    std::cout << "librobcomm basic_example" << std::endl;

    std::cout << "Connecting to robot..." << std::flush;
    robot.connect();

    std::cout << " Done." << std::endl;

    std::thread rx_thread(rx_task);

    usleep(1e6);

    robot.set_state(robcomm::RobotStateCommand::ROBOT_STATE_CMD_OPERATIONAL);

    usleep(3e6);

    for (int i = 0; i < 10; i ++) {
	    jog_trapezoid(2000, false, .005);
	    jog_trapezoid(2000, true, .005);
    }

    usleep(1e6);

    robot.set_state(robcomm::RobotStateCommand::ROBOT_STATE_CMD_SWITCHED_ON);

    usleep(1e6);

    keepRunning = false;
    rx_thread.join();
}

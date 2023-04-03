#include <robcomm/robcomm.hpp>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <thread>

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

int main(int argc, char** argv) {
    std::cout << "librobcomm basic_example" << std::endl;

    std::cout << "Connecting to robot..." << std::flush;
    robot.connect();

    std::cout << " Done." << std::endl;

    std::thread rx_thread(rx_task);

    usleep(1e6);

    robot.set_state(robcomm::RobotStateCommand::ROBOT_STATE_CMD_OPERATIONAL);

    usleep(3e6);

    robot.set_state(robcomm::RobotStateCommand::ROBOT_STATE_CMD_SWITCHED_ON);

    usleep(1e6);

    keepRunning = false;
    rx_thread.join();
}

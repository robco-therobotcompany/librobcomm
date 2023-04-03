#include <robcomm/robcomm.hpp>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

void print_robot_status(robcomm::RobotStatus status) {
    std::cout << "State: " << (uint8_t)status.robot_state << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "librobcomm basic_example" << std::endl;

    robcomm::Robot robot("sn23-1000360", 25001, 25000);

    std::cout << "Connecting to robot..." << std::flush;
    robot.connect();

    std::cout << " Done." << std::endl;

    robcomm::RobotStatus status = {};
    while(1) {
        robot.receive();

        robcomm::RobotStatus new_status = robot.get_status();
        if (new_status.robot_state != status.robot_state) {
            status = new_status;
            print_robot_status(status);
        }

        usleep(1e3);
    }
}
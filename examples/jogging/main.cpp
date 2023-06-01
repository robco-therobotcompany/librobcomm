#include <robcomm/robcomm.hpp>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <iomanip>
#include <vector>
#include <unistd.h>

robcomm::Robot robot;

const int NUM_JOINTS = 5;
const double MAX_VELOCITY = 0.5; // rad/s

// Connection parameters
const std::string ROBOT_IP = "192.168.3.1";
const int ROBOT_RX_PORT = 25001; // Local listening port
const int ROBOT_TX_PORT = 25000; // Target remote port

bool keepRunning = true; // Flag to stop rx thread

std::vector<double> dq_cmd; // Joint velocity command vector

void rx_task() {
    robcomm::RobotStatus status = {};

    while (keepRunning) {
        robot.receive();

        robcomm::RobotStatus newStatus = robot.get_status();
        if (newStatus.robot_state != status.robot_state) {
            status = newStatus;
            std::cout << "Robot state: 0x" << std::setfill('0') << std::hex <<
                status.robot_state << std::endl;
        }
    }
}

// Jog the first joint for ms milliseconds with maximum velocity maxvel, using a
// triangular velocity profile. reverse reverses the direction of travel.
void jog_triangular(int ms, bool reverse, double maxvel) {
    int cycles = ms/10;
    double step = maxvel/(cycles/2.0);

    if (reverse) {
       dq_cmd[0] = -dq_cmd[0];
       step = -step;
    }

    for(int i = 0; i < cycles; i ++) {
        // Send velocity command
        robot.jog_joints(dq_cmd);

        dq_cmd[0] += (i < cycles/2) ? step : -step;

        usleep(1e4);
    }
}

int main(int argc, char** argv) {
    dq_cmd.resize(NUM_JOINTS);

    std::cout << "librobcomm jogging example" << std::endl;

    std::cout << "Connecting to robot..." << std::flush;
    robot.connect(ROBOT_IP, ROBOT_RX_PORT, ROBOT_TX_PORT);

    // Start receiving status messages in rx thread
    std::thread rx_thread(rx_task);

    std::cout << " Done." << std::endl;

    std::cout << "Waiting for robot to initialize..." << std::endl;
    while (!robot.is_initialized()) {
        robot.receive();
    }

    std::cout << "Switching to operational state" << std::endl;
    robot.set_state(robcomm::RobotStateCommand::ROBOT_STATE_CMD_OPERATIONAL);

    usleep(3e6);

    for (int i = 0; i < 10; i ++) {
        jog_triangular(2000, false, MAX_VELOCITY);
        jog_triangular(2000, true, MAX_VELOCITY);
    }

    std::cout << "Exiting operational state" << std::endl;
    robot.set_state(robcomm::RobotStateCommand::ROBOT_STATE_CMD_SWITCHED_ON);

    keepRunning = false;
    rx_thread.join();

    return 0;
}

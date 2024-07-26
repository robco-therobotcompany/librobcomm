/** \file main.cpp
 * \brief Read joint data example.
 *
 *
 * Created on: 26.07.2024
 * Author: Bernhard Vorhofer
 * Contributor: -
 *
 * Copyright (C) 2024 RobCo GmbH - All Rights Reserved
 *
 */

#include "robcomm/module.hpp"
#include "robcomm/types.hpp"
#include <robcomm/robcomm.hpp>
#include <iostream>
#include <chrono>
#include <thread>

robcomm::Robot robot;

std::vector<double> dqs; // Jogging command vector

int main(int argc, char** argv) {
    std::cout << "librobcomm read_joint_data" << std::endl;

    // TODO: Set robot IP
    std::cout << "Connecting to robot..." << std::flush;
    robot.connect("10.11.12.14", 25001, 26000);

    std::cout << " Done." << std::endl;

    std::cout << "Waiting for robot to initialize..." << std::endl;
    while (!robot.is_initialized()) {
        robot.receive();
    }

    // Print module count
    std::cout << robot.get_module_count() << " modules found:" << std::endl;
    for (int i = 0; i < robot.get_module_count(); i ++) {
        std::cout << "  Module " << i << ": " << robot.get_module_type_id(i) << std::endl;
    }

    // Get number of joints and resize jogging command vector accordingly
    dqs.resize(robot.joints().size());

    // Set all but the first joint command to 0 rad/s
    for (int i = 0; i < dqs.size(); i ++) {
        dqs[i] = 0.0;
    }
    dqs[0] = 0.1; // Jog joint 1 with 0.1 rad/s

    // Set robot to operational state (required for moving)
    robot.set_state(robcomm::RobotStateCommand::ROBOT_STATE_CMD_OPERATIONAL);

    auto last_wake = std::chrono::steady_clock::now();
    int read_counter = 0;
    int jog_counter = 0;

    while (true) {
        // Run loop with 1 kHz
        std::this_thread::sleep_until(last_wake + std::chrono::milliseconds(1));

        // Receive data from robot to update internal model
        robot.receive();

        // Read joint values at 1 Hz
        if (read_counter++ >= 1000) {
            read_counter = 0;

            // Print joing angle and torque values for each joint
            const std::vector<robcomm::Joint>& joints = robot.joints();
            std::cout << "Joints (angle, torque)" << std::endl;
            for (int i = 0; i < joints.size(); i ++) {
                std::cout << "  " << i+1 << " ( " << joints[i].q() << ", " << joints[i].torque() << " )" << std::endl;
            }
        }

        // Send jogging commands at 100 Hz
        if (jog_counter++ >= 10) {
            jog_counter = 0;

            // Jog first joint at 0.1 rad/s
            robot.jog_joints(dqs);
        }

        last_wake = std::chrono::steady_clock::now();
    }

    return 0;
}

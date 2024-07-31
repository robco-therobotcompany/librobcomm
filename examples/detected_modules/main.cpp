/** \file main.cpp
 * \brief Detected modules example.
 *
 *
 * Created on: 26.07.2024
 * Author: Bernhard Vorhofer
 * Contributor: -
 *
 * Copyright (C) 2024 RobCo GmbH - All Rights Reserved
 *
 * This example demonstrates simple read-only access to the robot. It waits for the robot model to
 * be initialized from the UDP stream, then prints the module IDs of all detected modules.
 *
 */

#include <iostream>
#include <robcomm/robcomm.hpp>

robcomm::Robot robot;

int main(int argc, char** argv) {
	std::cout << "librobcomm basic_example" << std::endl;

	std::cout << "Connecting to robot..." << std::flush;
	robot.connect("192.168.3.1", 25001);

	std::cout << " Done." << std::endl;

	std::cout << "Waiting for robot to initialize..." << std::endl;
	while(!robot.is_initialized()) {
		robot.receive();
	}

	std::cout << robot.modules().size() << " modules found:" << std::endl;
	for(int i = 0; i < robot.modules().size(); i++) {
		std::cout << "  Module " << i << ": " << robot.modules()[i].module_id() << std::endl;
	}

	return 0;
}

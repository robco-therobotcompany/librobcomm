#include <robcomm/robcomm.hpp>
#include <iostream>

robcomm::Robot robot;

int main(int argc, char** argv) {
    std::cout << "librobcomm basic_example" << std::endl;

    std::cout << "Connecting to robot..." << std::flush;
    robot.connect("192.168.3.1", 25001, 25000);

    std::cout << " Done." << std::endl;

    std::cout << "Waiting for robot to initialize..." << std::endl;
    while (!robot.is_initialized()) {
        robot.receive();
    }

    std::cout << robot.get_module_count() << " modules found:" << std::endl;
    for (int i = 0; i < robot.get_module_count(); i ++) {
        std::cout << "  Module " << i << ": " << robot.get_module_type_id(i) << std::endl;
    }

    return 0;
}

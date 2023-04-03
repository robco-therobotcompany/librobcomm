/** \file robot.hpp
 * \brief Robot class, handles communication with Robco robot.
 *
 * Created on: 24.03.2023
 * Author: Bernhard Vorhofer
 * Contributor: -
 * 
 * Copyright (C) 2023 RobCo GmbH - All Rights Reserved
 * 
 */

#ifndef ROBCOMM_ROBOT_H
#define ROBCOMM_ROBOT_H

#include <robcomm/types.hpp>
#include <robcomm/robot_messages.hpp>
#include <string>
#include <stdint.h>
#include <netinet/in.h>
#include <vector>

// Major protocol version this implementation is compatible with
#define COMPATIBLE_MAJOR_VERSION 4

namespace robcomm {
    class Robot {
        public:
            Robot(std::string host, uint16_t rx_port_local, uint16_t tx_port_remote);
            ~Robot();

            void connect();
            void receive();

            void set_state(RobotStateCommand cmd);
            void jog_joints(std::vector<double> &dqs);

            RobotStatus get_status();
            int get_module_count();
            ModuleState get_module_state(int i);
            int get_error_count();
            uint16_t get_error_code(int i);

            int get_joint_count();
            const std::vector<double>& getJointAngles() const;

        private:
            std::string host;
            uint16_t rx_port_local;
            uint16_t tx_port_remote;
            int sockfd_rx;
            int sockfd_tx;
            sockaddr_in robot_addr;
            sockaddr_in local_addr;
            char* recv_buffer;

            uint8_t robot_protocol_version_major;
            uint8_t robot_protocol_version_minor;
            uint8_t seq_counter;

            RobotStatus robot_status = {};
            std::vector<ModuleState> module_states;
            std::vector<uint16_t> error_codes;

            std::vector<double> q;

            SET_MSG* new_message(uint8_t msg_type, size_t payload_size);
            void send_message(SET_MSG* msg);

            void handle_get_message(GET_MSG* msg);
            void handle_get_udp_protocol_version(MSG_GET_UDP_PROTOCOL_VERSION* msg);
            void handle_get_status(MSG_GET_STATUS* msg);
            void handle_get_joint_abs(MSG_GET_JOINT_ABS* msg);
    };
};

#endif // ROBCOMM_ROBOT_H
/** \file robot.hpp
 * \brief Robot class, handles communication with RobCo robot.
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

#include <netinet/in.h>
#include <robcomm/joint.hpp>
#include <robcomm/robot_messages.hpp>
#include <robcomm/types.hpp>
#include <stdint.h>
#include <string>
#include <vector>

// Major protocol version this implementation is compatible with
#define COMPATIBLE_MAJOR_VERSION 4

namespace robcomm {
	class Robot {
	public:
		Robot();
		~Robot();

		/**
		 * @brief Initializes network sockets for robot connection.
		 */
		void connect(std::string host, uint16_t rx_port_local, uint16_t tx_port_remote);

		/**
		 * @brief Processes incoming messages from robot, to be called cyclically.
		 */
		void receive();

		/**
		 * @brief Request the robot to change into given state.
		 *
		 * @param cmd State command
		 */
		void set_state(RobotStateCommand cmd);

		/**
		 * @brief Returns a vector with the robot's joints.
		 *
		 * @return A vector containing joint objects representing the joints of the robot
		 */
		const std::vector<Joint>& joints() const;

		/**
		 * @brief Returns a vector with the robot's modules.
		 *
		 * @return A vector containing joint objects representing the modules of the robot
		 */
		const std::vector<Module>& modules() const;

		/**
		 * @brief Send jog command with given joint angles.
		 *
		 * @param dqs Joint angle vector (values in rad/s), must have
		 *        correct size (number of joints).
		 */
		void jog_joints(std::vector<double>& dqs);

		void set_output(uint8_t bank, uint32_t address, uint32_t value);

		bool is_initialized();
		RobotStatus get_status();
		int get_module_count();
		uint32_t get_module_type_id(int i);
		ModuleState get_module_state(int i);
		int get_active_error_count();
		uint16_t get_active_error_code(int i);

		int get_joint_count();
		const std::vector<double> getJointAngles() const;

	private:
		bool _joints_valid;  // Set to true after first GET_JOINT_ABS message received
		bool _status_valid;  // Set to true after first GET_STATUS message received
		bool _modules_valid; // Set to true after first GET_DETECTED_MODULES message received

		std::string _host;
		uint16_t _rx_port_local;
		uint16_t _tx_port_remote;
		int _sockfd_rx;
		int _sockfd_tx;
		sockaddr_in _robot_addr;
		sockaddr_in _local_addr;
		char* _recv_buffer;

		uint8_t _robot_protocol_version_major;
		uint8_t _robot_protocol_version_minor;
		uint8_t _seq_counter;

		RobotStatus _robot_status;
		std::vector<uint16_t> _active_error_codes;

		uint8_t _actual_global_velocity_percent;
		uint8_t _desired_global_velocity_percent;
		double _payload_mass_kg;
		double _payload_cog_x_meters;
		double _payload_cog_y_meters;
		double _payload_cog_z_meters;

		std::vector<Module> _modules;
		std::vector<Joint> _joints;

		void check_payload_length_equal(uint16_t payload_len, uint16_t expected_len,
		        std::string message_name);
		void check_payload_length_greater_than(uint16_t payload_len, uint16_t min_len,
		        std::string message_name);
		void check_n_modules(uint8_t n_modules, std::string message_name);
		void check_n_joints(uint8_t n_joints, std::string message_name);

		SET_MSG* new_message(uint8_t msg_type, size_t payload_size);
		void send_message(SET_MSG* msg);

		void handle_get_message(GET_MSG* msg);
		void handle_get_udp_protocol_version(MSG_GET_UDP_PROTOCOL_VERSION* msg);
		void handle_get_status(MSG_GET_STATUS* msg);
		void handle_get_joint_abs(MSG_GET_JOINT_ABS* msg);
		void handle_get_detected_modules(MSG_GET_DETECTED_MODULES* msg);
		void handle_get_global_velocity(MSG_GET_GLOBAL_VELOCITY* msg);
		void handle_get_payload_mass(MSG_GET_PAYLOAD_MASS* msg);
		void handle_get_joint_temperature_pairs(MSG_GET_JOINT_TEMPERATURE_PAIRS* msg);
		void handle_get_joint_torsions(MSG_GET_JOINT_TORSIONS* msg);
		void handle_get_joint_overloads_percent(MSG_GET_JOINT_OVERLOADS_PERCENT* msg);
		void handle_get_joint_torques(MSG_GET_JOINT_TORQUES* msg);
	};
}; // namespace robcomm

#endif // ROBCOMM_ROBOT_H

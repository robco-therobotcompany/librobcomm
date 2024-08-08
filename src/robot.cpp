#include <arpa/inet.h>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <robcomm/robcomm.hpp>
#include <robcomm/robot_messages.hpp>
#include <sstream>
#include <stdexcept>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define ROBCOMM_RECV_BUFFER_SIZE 1024 // receive buffer size in bytes

namespace robcomm {

	Robot::Robot() {
		this->_recv_buffer = new char[ROBCOMM_RECV_BUFFER_SIZE];

		this->_robot_protocol_version_major = 0;
		this->_robot_protocol_version_minor = 0;
	}

	void Robot::connect(std::string host, uint16_t rx_port_local) {
		this->connect(host, rx_port_local, 0);
	}

	void Robot::connect(std::string host, uint16_t rx_port_local, uint16_t tx_port_remote) {
		this->_host = host;
		this->_rx_port_local = rx_port_local;
		this->_tx_port_remote = tx_port_remote;
		this->_seq_counter = 0;

		if((_sockfd_rx = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
			throw std::runtime_error("RX Socket creation failed.");

		fcntl(_sockfd_rx, F_SETFL, O_NONBLOCK); // set socket nonblocking

		_local_addr.sin_family = AF_INET;
		_local_addr.sin_addr.s_addr = INADDR_ANY;
		_local_addr.sin_port = htons(rx_port_local);

		if(bind(_sockfd_rx, (const sockaddr*)&_local_addr, sizeof(_local_addr)) < 0)
			throw std::runtime_error("RX Socket bind() failed.");

		memset(&_robot_addr, 0, sizeof(_robot_addr));

		_robot_addr.sin_family = AF_INET;
		_robot_addr.sin_port = htons(tx_port_remote);

		inet_aton(host.c_str(), &_robot_addr.sin_addr);

		if (tx_port_remote != 0) {
			if((_sockfd_tx = socket(AF_INET, SOCK_STREAM, 0)) < 0)
				throw std::runtime_error("TX socket creation failed");

			if(::connect(_sockfd_tx, (struct sockaddr*)&_robot_addr, sizeof(_robot_addr)) < 0)
				throw std::runtime_error("TX socket connection failed");
		}
	}

	bool Robot::is_initialized() {
		return _modules_valid && _status_valid && _modules_valid;
	}

	void Robot::receive() {
		sockaddr_in from_addr;

		socklen_t len = (socklen_t)sizeof(from_addr);
		int n;
		while((n = recvfrom(_sockfd_rx, _recv_buffer, ROBCOMM_RECV_BUFFER_SIZE, 0,
		               (sockaddr*)&from_addr, &len)) > 0) {
			if(n < sizeof(GET_MSG))
				throw std::runtime_error("Received packet is too short");

			GET_MSG* msg = (GET_MSG*)_recv_buffer;

			if (msg->type == MSG_TYPE_GET_GLOBAL_VELOCITY) {
				__asm__("nop");
			}

			handle_get_message(msg);
		}
	}

	const std::vector<Joint>& Robot::joints() const {
		return this->_joints;
	}

	const std::vector<Module>& Robot::modules() const {
		return this->_modules;
	}

	void Robot::check_payload_length_equal(uint16_t payload_len, uint16_t expected_len,
	        std::string message_name) {
		std::stringstream exception_ss;
		if(payload_len != expected_len) {
			exception_ss << "Invalid payload length " << payload_len << " for " << message_name
			             << " messge (expected " << expected_len << ")";
			if(payload_len > expected_len)
				std::cout << exception_ss.str() << std::endl;
			else
				throw std::runtime_error(exception_ss.str());
		}
	}

	void Robot::check_payload_length_greater_than(uint16_t payload_len, uint16_t min_len,
	        std::string message_name) {
		std::stringstream exception_ss;
		if(payload_len < min_len) {
			exception_ss << "Invalid payload length " << payload_len << " for " << message_name
			             << " messge (expected at least " << min_len << ")";
			throw std::runtime_error(exception_ss.str());
		}
	}

	void Robot::check_n_modules(uint8_t n_modules, std::string message_name) {
		std::stringstream exception_ss;
		if(n_modules != this->_modules.size()) {
			exception_ss << message_name
			             << " message has an invalid number of modules "
			                "(expected "
			             << this->_modules.size() << ", got " << n_modules;
			throw std::runtime_error(exception_ss.str());
		}
	}

	void Robot::check_n_joints(uint8_t n_joints, std::string message_name) {
		std::stringstream exception_ss;
		if(n_joints != this->_joints.size()) {
			exception_ss << message_name
			             << " message has an invalid number of joints "
			                "(expected "
			             << this->_joints.size() << ", got " << n_joints;
			throw std::runtime_error(exception_ss.str());
		}
	}

	void Robot::handle_get_message(GET_MSG* msg) {
		std::stringstream exception_ss;

		uint16_t payload_len = ntohs(msg->payload_len);

		switch(msg->type) {
		case MSG_TYPE_GET_UDP_PROTOCOL_VERSION:
			check_payload_length_equal(payload_len, sizeof(MSG_GET_UDP_PROTOCOL_VERSION),
			        "GET_UDP_PROTOCOL_VERSION");
			handle_get_udp_protocol_version((MSG_GET_UDP_PROTOCOL_VERSION*)msg->payload);
			break;
		case MSG_TYPE_GET_STATUS:
			check_payload_length_greater_than(payload_len,
			        sizeof(MSG_GET_STATUS) + sizeof(MSG_GET_STATUS_MODULES) +
			                sizeof(MSG_GET_STATUS_ERRORS),
			        "GET_STATUS");
			handle_get_status((MSG_GET_STATUS*)msg->payload);
			break;
		case MSG_TYPE_GET_JOINT_ABS:
			check_payload_length_greater_than(payload_len, sizeof(MSG_GET_JOINT_ABS),
			        "GET_JOINT_ABS");
			handle_get_joint_abs((MSG_GET_JOINT_ABS*)msg->payload);
			break;
		case MSG_TYPE_GET_DETECTED_MODULES:
			check_payload_length_greater_than(payload_len, sizeof(MSG_GET_DETECTED_MODULES),
			        "GET_DETECTED_MODULES");
			handle_get_detected_modules((MSG_GET_DETECTED_MODULES*)msg->payload);
			break;
		case MSG_TYPE_GET_GLOBAL_VELOCITY:
			check_payload_length_equal(payload_len, sizeof(MSG_GET_GLOBAL_VELOCITY),
			        "GET_GLOBAL_VELOCITY");
			handle_get_global_velocity((MSG_GET_GLOBAL_VELOCITY*)msg->payload);
			break;
		case MSG_TYPE_GET_PAYLOAD_MASS:
			check_payload_length_equal(payload_len, sizeof(MSG_GET_PAYLOAD_MASS),
			        "GET_PAYLOAD_MASS");
			handle_get_payload_mass((MSG_GET_PAYLOAD_MASS*)msg->payload);
			break;
		case MSG_TYPE_GET_JOINT_TEMPERATURE_PAIRS:
			check_payload_length_greater_than(payload_len, sizeof(MSG_GET_JOINT_TEMPERATURE_PAIRS),
			        "GET_JOINT_TEMPERATURE_PAIRS");
			handle_get_joint_temperature_pairs((MSG_GET_JOINT_TEMPERATURE_PAIRS*)msg->payload);
			break;
		case MSG_TYPE_GET_JOINT_TORSIONS:
			check_payload_length_greater_than(payload_len, sizeof(MSG_GET_JOINT_TORSIONS),
			        "GET_JOINT_TORSIONS");
			handle_get_joint_torsions((MSG_GET_JOINT_TORSIONS*)msg->payload);
			break;
		case MSG_TYPE_GET_JOINT_OVERLOADS_PERCENT:
			check_payload_length_greater_than(payload_len, sizeof(MSG_GET_JOINT_OVERLOADS_PERCENT),
			        "GET_JOINT_OVERLOADS_PERCENT");
			handle_get_joint_overloads_percent((MSG_GET_JOINT_OVERLOADS_PERCENT*)msg->payload);
		case MSG_TYPE_GET_JOINT_TORQUES:
			check_payload_length_greater_than(payload_len, sizeof(MSG_GET_JOINT_TORQUES),
			        "GET_JOINT_TORQUES");
			handle_get_joint_torques((MSG_GET_JOINT_TORQUES*)msg->payload);
		case MSG_TYPE_GET_TCP_SHIFT:
		case MSG_TYPE_GET_OUTPUT:
		case MSG_TYPE_GET_INPUT:
		case MSG_TYPE_GET_LAST_OCCURRED_ERRORS:
		case MSG_TYPE_GET_LAST_REACHED_WAYPOINT:
		case MSG_TYPE_GET_POSE_ABS:
		case MSG_TYPE_SET_JOINT_ABS:
		case MSG_TYPE_SET_JOINT_OFFS:
		case MSG_TYPE_SET_POSE_ABS:
		case MSG_TYPE_SET_POSE_OFFS:
		case MSG_TYPE_SET_CONSTANT_VELOCITY_TRAJECTORY:
		case MSG_TYPE_SET_GLOBAL_VELOCITY:
		case MSG_TYPE_SET_OUTPUT:
		case MSG_TYPE_SET_JOINT_LEDS:
		case MSG_TYPE_SET_G_COMPENSATION:
		case MSG_TYPE_SET_PAYLOAD_MASS:
		case MSG_TYPE_SET_TCP_SHIFT:
		case MSG_TYPE_SET_ROBOT_STATE:
		case MSG_TYPE_SET_STOP:
			break;
		default:
			exception_ss << "Invalid message type '" << unsigned(msg->type) << "'";
			throw std::runtime_error(exception_ss.str());
			break;
		}
	}

	void Robot::handle_get_udp_protocol_version(MSG_GET_UDP_PROTOCOL_VERSION* msg) {
		std::stringstream exception_ss;

		if(_robot_protocol_version_major == 0 && _robot_protocol_version_minor == 0) {
			if(msg->major != COMPATIBLE_MAJOR_VERSION) {
				exception_ss << "Controller reports incompatible protocol version " << msg->major
				             << "." << msg->minor;
				throw std::runtime_error(exception_ss.str());
			}

			_robot_protocol_version_major = msg->major;
			_robot_protocol_version_minor = msg->minor;
		}
		else if(_robot_protocol_version_major != msg->major ||
		        _robot_protocol_version_minor != msg->minor) {
			exception_ss << "Controller changed protocol version from "
			             << _robot_protocol_version_major << "." << _robot_protocol_version_minor
			             << " to " << msg->major << "." << msg->minor << "during operation!";
			throw std::runtime_error(exception_ss.str());
		}
	}

	void Robot::handle_get_status(MSG_GET_STATUS* msg) {
		std::stringstream exception_ss;

		_robot_status = msg_get_robot_status(msg);

		MSG_GET_STATUS_MODULES* modules = (MSG_GET_STATUS_MODULES*)msg->data;

		if(_modules_valid) {
			if(this->_modules.size() + 2 != modules->n_modules) {
				exception_ss << "Controller changed number of modules  from "
				             << this->_modules.size() << " to " << unsigned(modules->n_modules)
				             << " during operation!";
				throw std::runtime_error(exception_ss.str());
			}

			for(int i = 0; i < this->_modules.size(); i++) {
				ModuleState ms = msg_get_module_state(modules, i);
				this->_modules[i]._state = ms;
			}
		}

		MSG_GET_STATUS_ERRORS* errors =
		        (MSG_GET_STATUS_ERRORS*)(msg->data + len_MSG_GET_STATUS_MODULES(modules));

		// Resize internal error code vector, if necessary
		_active_error_codes.resize(errors->n_errors);

		for(int i = 0; i < errors->n_errors; i++) {
			_active_error_codes[i] = errors->errors[i];
		}

		_status_valid = true;
	}

	void Robot::handle_get_joint_abs(MSG_GET_JOINT_ABS* msg) {
		std::stringstream exception_ss;

		// Resize joints vector if necessary
		_joints.resize(msg->n_joints);

		for(int i = 0; i < msg->n_joints; i++) {
			_joints[i]._q = ntoh_angle(msg->joint_values[i]);
		}

		this->_joints_valid = true;
	}

	void Robot::handle_get_detected_modules(MSG_GET_DETECTED_MODULES* msg) {
		this->_modules.resize(msg->n_modules);

		int actual_n_modules = 0;
		for(int i = 0; i < msg->n_modules; i++) {
			MSG_GET_DETECTED_MODULES_MODULE* module =
			        (MSG_GET_DETECTED_MODULES_MODULE*)(&msg->data[i *
			                sizeof(MSG_GET_DETECTED_MODULES_MODULE)]);

			uint32_t module_id = ntohl(module->id);

			// Ignore clamps
			if(module_id >= 8000 && module_id <= 8999)
				continue;

			_modules[actual_n_modules]._module_id = module_id;
			actual_n_modules++;
		}

		// Resize again, as we ignored some modules.
		_modules.resize(actual_n_modules);
		_modules_valid = true;
	}

	void Robot::handle_get_global_velocity(MSG_GET_GLOBAL_VELOCITY* msg) {
		_actual_global_velocity_percent = msg->actual_velocity_percent;
		_desired_global_velocity_percent = msg->desired_velocity_percent;
	}

	void Robot::handle_get_payload_mass(MSG_GET_PAYLOAD_MASS* msg) {
		_payload_mass_kg = ntoh_mass(msg->mass);
		_payload_cog_x_meters = ntoh_linear(msg->center_of_mass_x);
		_payload_cog_y_meters = ntoh_linear(msg->center_of_mass_y);
		_payload_cog_z_meters = ntoh_linear(msg->center_of_mass_z);
	}

	void Robot::handle_get_joint_temperature_pairs(MSG_GET_JOINT_TEMPERATURE_PAIRS* msg) {
		if(!_joints_valid) {
			return;
		}

		check_n_joints(msg->n_joints, "GET_JOINT_TEMPERATURE_PAIRS");

		for(int i = 0; i < msg->n_joints; i++) {
			MSG_GET_JOINT_TEMPERATURE_PAIRS_TEMPS* temps =
			        (MSG_GET_JOINT_TEMPERATURE_PAIRS_TEMPS*)(&msg->data[i *
			                sizeof(MSG_GET_JOINT_TEMPERATURE_PAIRS_TEMPS)]);

			_joints[i]._motor_temperature_deg_c = ntoh_temperature(temps->motor_temperature);
			_joints[i]._controller_temperature_deg_c = ntoh_temperature(temps->controller_temperature);
		}
	}

	void Robot::handle_get_joint_torsions(MSG_GET_JOINT_TORSIONS* msg) {
		if(!_joints_valid) {
			return;
		}

		check_n_joints(msg->n_joints, "GET_JOINT_TORSIONS");

		for(int i = 0; i < msg->n_joints; i++) {
			_joints[i]._torsion_rad = ntoh_angle(msg->joint_torsions[i]);
		}
	}

	void Robot::handle_get_joint_overloads_percent(MSG_GET_JOINT_OVERLOADS_PERCENT* msg) {
		if(!_joints_valid) {
			return;
		}

		check_n_joints(msg->n_joints, "GET_JOINT_OVERLOADS_PERCENT");

		for(int i = 0; i < msg->n_joints; i++) {
			_joints[i]._overload_percent = msg->overload_values[i];
		}
	}

	void Robot::handle_get_joint_torques(MSG_GET_JOINT_TORQUES* msg) {
		if(!_joints_valid) {
			return;
		}

		check_n_joints(msg->n_joints, "GET_JOINT_TORQUES");

		for(int i = 0; i < msg->n_joints; i++) {
			_joints[i]._torque_n = ntoh_torque(msg->torque_values[i]);
		}
	}

	SET_MSG* Robot::new_message(uint8_t msg_type, size_t payload_size) {
		return new_UDP_MSG(msg_type, _seq_counter++, payload_size);
	}

	void Robot::send_message(SET_MSG* msg) {
		if (this->_sockfd_tx <= 0) {
			throw std::runtime_error("TX socket was not set up, unable to send commands");
		}

		ssize_t n = send(_sockfd_tx, msg, len_SET_MSG(msg), 0);

		if(n < 0) {
			throw std::runtime_error("Error while sending message");
		}
	}

	void Robot::set_state(RobotStateCommand cmd) {
		SET_MSG* msg = new_message(MSG_TYPE_SET_ROBOT_STATE, sizeof(MSG_SET_ROBOT_STATE));
		MSG_SET_ROBOT_STATE* payload = (MSG_SET_ROBOT_STATE*)msg->payload;

		payload->robot_state = (uint8_t)cmd;

		send_message(msg);

		free(msg);
	}

	void Robot::jog_joints(std::vector<double>& dqs) {
		if(dqs.size() != _joints.size()) {
			throw std::runtime_error("jog_joints command has size different from internal joint vector");
		}

		// TODO: centralize handling of sequence number
		SET_MSG* msg = new_MSG_SET_JOINT_OFFS(_seq_counter++, dqs.size());
		MSG_SET_JOINT_OFFS* payload = (MSG_SET_JOINT_OFFS*)msg->payload;

		for(int i = 0; i < dqs.size(); i++) {
			// / 100 because the controller will execute this dq within 10ms,
			// so we need to command 1/100th of the velocity value (rad/s) at
			// a time.
			payload->joint_angles[i] = hton_angle(dqs[i] / 100.0);
		}

		send_message(msg);
		free(msg);
	}

	void Robot::set_output(uint8_t bank, uint32_t address, uint32_t value) {
		SET_MSG* msg = new_message(MSG_TYPE_SET_OUTPUT, sizeof(MSG_SET_OUTPUT));
		MSG_SET_OUTPUT* payload = (MSG_SET_OUTPUT*)msg->payload;

		payload->bank = bank;
		payload->address = address;
		payload->value = value;

		send_message(msg);

		free(msg);
	}

	RobotStatus Robot::get_status() {
		return _robot_status;
	}

	int Robot::get_active_error_count() {
		return _active_error_codes.size();
	}

	uint16_t Robot::get_active_error_code(int i) {
		return _active_error_codes[i];
	}

	const std::vector<double> Robot::getJointAngles() const {
		std::vector<double> result(this->_joints.size());

		for(int i = 0; i < this->_joints.size(); i++) {
			result[i] = this->_joints[i]._q;
		}

		return result;
	}

	Robot::~Robot() {
		close(_sockfd_rx);
		close(_sockfd_tx);
		delete[] _recv_buffer;
	}
} // namespace robcomm

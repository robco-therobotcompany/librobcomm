/**
 * @file robot_messages.hpp
 * @brief UDP messages for robot communication.
 *        Robco UDP protocol v4.5
 *
 * Created on: 24.03.2023
 * Author: Bernhard Vorhofer
 * Contributor: -
 */

#ifndef ROBCOMM_ROBOT_MESSAGES_H
#define ROBCOMM_ROBOT_MESSAGES_H

#include <robcomm/types.hpp>
#include <stdint.h>
#include <sys/types.h>

namespace robcomm {
	struct __attribute__((packed)) CartesianVector {
		int32_t x; /*<! X-component of Cartesian position, in micrometers */
		int32_t y; /*<! Y-component of Cartesian position, in micrometers */
		int32_t z; /*<! Z-component of Cartesian position, in micrometers */
	};

	struct __attribute__((packed)) EulerAngles {
		int32_t euler_z; /*<! Z-component of Cartesian orientation, in ZYX Euler angles, encoded as
		                    nanorad/pi */
		int32_t euler_y; /*<! Y-component of Cartesian orientation, in ZYX Euler angles, encoded as
		                    nanorad/pi */
		int32_t euler_x; /*<! X-component of Cartesian orientation, in ZYX Euler angles, encoded as
		                    nanorad/pi */
	};

	struct __attribute__((packed)) RobcommCartesianPose {
		CartesianVector position;
		EulerAngles orientation;
	};

	enum MSG_TYPE {
		MSG_TYPE_GET_UDP_PROTOCOL_VERSION = 0x01,
		MSG_TYPE_GET_STATUS = 0x02,
		MSG_TYPE_GET_LAST_OCCURRED_ERRORS = 0x03,
		MSG_TYPE_GET_DETECTED_MODULES = 0x05,
		MSG_TYPE_GET_LAST_REACHED_WAYPOINT = 0x0F,
		MSG_TYPE_SET_JOINT_ABS = 0x10,
		MSG_TYPE_GET_JOINT_ABS = 0x11,
		MSG_TYPE_SET_JOINT_OFFS = 0x12,
		MSG_TYPE_SET_POSE_ABS = 0x14,
		MSG_TYPE_GET_POSE_ABS = 0x15,
		MSG_TYPE_SET_POSE_OFFS = 0x16,
		MSG_TYPE_SET_CONSTANT_VELOCITY_TRAJECTORY = 0x1A,
		MSG_TYPE_SET_CONSTANT_VELOCITY_TRAJECTORY_WAYPOINT = 0x1B,
		MSG_TYPE_SET_GLOBAL_VELOCITY = 0x20,
		MSG_TYPE_GET_GLOBAL_VELOCITY = 0x21,
		MSG_TYPE_SET_OUTPUT = 0x30,
		MSG_TYPE_GET_OUTPUT = 0x31,
		MSG_TYPE_GET_INPUT = 0x33,
		MSG_TYPE_SET_JOINT_LEDS = 0x34,
		MSG_TYPE_SET_G_COMPENSATION = 0x41,
		MSG_TYPE_SET_PAYLOAD_MASS = 0x70,
		MSG_TYPE_GET_PAYLOAD_MASS = 0x71,
		MSG_TYPE_SET_TCP_SHIFT = 0x72,
		MSG_TYPE_GET_TCP_SHIFT = 0x73,
		MSG_TYPE_GET_JOINT_TEMPERATURE_PAIRS = 0xA0,
		MSG_TYPE_GET_JOINT_TORSIONS = 0xA1,
		MSG_TYPE_GET_JOINT_OVERLOADS_PERCENT = 0xA2,
		MSG_TYPE_GET_JOINT_TORQUES = 0xA3,
		MSG_TYPE_SET_ROBOT_STATE = 0xE0,
		MSG_TYPE_SET_STOP = 0xEE,
		MSG_TYPE_SET_PAUSE = 0xEF
	};

	/* SET MESSAGES - Commands to robot */

	struct __attribute__((packed)) SET_MSG {
		uint8_t type;
		uint8_t seq;
		uint16_t payload_len;
		char payload[];
	};

	/// Command robot to move to an absolute position in joint space.
	struct __attribute__((packed)) MSG_SET_JOINT_ABS {
		uint8_t point_seq_num; /*<! Arbitrary sequence number to identify the waypoint in this
		                          message */
		uint8_t approach_mode; /*<! Approach mode for waypoint in this message, use values from @ref
		                          ApproachMode */
		uint8_t approach_velocity; /*<! Permitted approach velocity for waypoint in this message,
		                              where 0 equals 0% of the maximum velocity and 255 equals 100%
		                            */
		uint8_t approach_acceleration; /*<! Permitted approach acceleration for waypoint in this
		                                  message, where 0 equals 0% of the maximum acceleration and
		                                  255 equals 100% */
		uint32_t blending_radius; /*<! Radius of a sphere around the waypoint within which the robot
		                             can smoothly transition to the next trajectory */
		uint8_t n_joints;         /*<! Number of joint values in this message */
		int32_t joint_angles[]; /*<! Variable-length data: n_joints x int32_t, encoding joint values
		                           as nanoradians/pi */
	};

	/// Command robot to move by a given offset in joint space, relative to the current position
	struct __attribute__((packed)) MSG_SET_JOINT_OFFS {
		uint8_t num_joints;     /*<! Number of joint offsets in this message */
		int32_t joint_angles[]; /*<! Variable-length data: n_joints x int32_t, encoding joint
		                           offsets as nanoradians/pi */
	};

	/// Command robot to move its end effector to an absolute pose in Cartesian space
	struct __attribute__((packed)) MSG_SET_POSE_ABS {
		uint8_t point_seq_num; /*<! Arbitrary sequence number to identify the waypoint in this
		                          message */
		uint8_t approach_mode; /*<! Approach mode for waypoint in this message, use values from @ref
		                          MSG_SET_JOINT_ABS_APPROACH_MODE */
		uint8_t approach_velocity; /*<! Permitted approach velocity for waypoint in this message,
		                              where 0 equals 0% of the maximum velocity and 255 equals 100%
		                            */
		uint8_t approach_acceleration; /*<! Permitted approach acceleration for waypoint in this
		                                  message, where 0 equals 0% of the maximum acceleration and
		                                  255 equals 100% */
		uint32_t blending_radius; /*<! Radius of a sphere around the waypoint within which the robot
		                             can smoothly transition to the next trajectory */
		RobcommCartesianPose desired_pose; /*<! Desired absoluted cartesian pose */
	};

	/// Command robot to move by the given offset in Cartesian space, relative to the current
	/// position. Use for jogging only.
	struct __attribute__((packed)) MSG_SET_POSE_OFFS {
		uint8_t coordinate_frame; /*<! Coordinate frame to use for relative movement, use values
		                             from @ref CoordinateFrame */
		RobcommCartesianPose
		        offset; /*<! Cartesian offset from current position to new desired position */
	};

	struct __attribute__((packed)) MSG_SET_CONSTANT_VELOCITY_TRAJECTORY_WAYPOINT {
		uint8_t point_sequence_number; /*<! Arbitrary sequence number to identify the waypoint in
		                                  this message */
		uint8_t segment_type;          /*<! Type of segment, use values from @ref SegmentType */
		uint32_t blending_radius; /*<! Radius of a sphere around the waypoint within which the robot
		                             can smoothly transition to the next trajectory */
		RobcommCartesianPose target_pose; /*<! Cartesian pose for final configuration at the end of
		                                     the segment */
		CartesianVector intermediate_point; /*<! Cartesian vector representing intermediate
		                               point of the segment, ignored when segment type is not arc */
	};

	struct __attribute__((packed)) MSG_SET_CONSTANT_VELOCITY_TRAJECTORY {
		uint8_t reserved;     /*<! Reserved for future use */
		uint8_t n_waypoints;  /*<! Number of waypoints in this trajectory, must be >= 1, controller
		                         expects n_waypoints-1 messages after this to send the remaining
		                         waypoints */
		uint8_t velocity;     /*<! Permitted approach velocity for waypoint in this message, where 0
		                         equals 0% of the maximum velocity and 255 equals 100% */
		uint8_t acceleration; /*<! Permitted approach acceleration for waypoint in this message,
		                         where 0 equals 0% of the maximum acceleration and 255 equals 100%
		                       */
		struct MSG_SET_CONSTANT_VELOCITY_TRAJECTORY_WAYPOINT
		        first_waypoint; /*<! First waypoint in the trajectory. Remaining will follow as
		                           separate SET_CONSTANT_VELOCITY_TRAJECTORY_WAYPOINT messages. */
	};

	/// Command robot to execute all movements at the given percentage of the full velocity.
	struct __attribute__((packed)) MSG_SET_GLOBAL_VELOCITY {
		uint8_t velocity_percent; /*<! Percentage of the full velocity to use, 0 equals 0%, 200
		                             equals 100% */
	};

	/// Command robot to set an analog or digital output to a given value
	struct __attribute__((packed)) MSG_SET_OUTPUT {
		uint8_t bank;     /*<! I/O bank number on which to set an output */
		uint32_t address; /*<! Address of output to be set within I/O bank */
		uint32_t value;   /*<! Value to set output to */
	};

	/// Command robot to temporarily enter gravity compensation mode. This message must be sent
	/// cyclically to keep the robot in this mode (recommended rate is 100 Hz). To exit gravity
	/// compensation mode, stop sending this message.
	struct __attribute__((packed)) MSG_SET_G_COMPENSATION {
		uint8_t mode; /*<! Gravity compensation mode. Currently, only 0 is supported. */
	};

	struct __attribute__((packed)) MSG_SET_PAYLOAD_MASS {
		uint32_t mass;                  /*<! New payload mass in milligrams */
		CartesianVector center_of_mass; /*<! Center of mass, relative to the distal point of
		                                          the end effector (ignoring any TCP shift) */
	};

	struct __attribute__((packed)) MSG_SET_TCP_SHIFT {
		RobcommCartesianPose offset; /*<! Offset between distal frame of end effector and desired
		                                tool-center-point (TCP) frame */
	};

	/// Set desired state of the robot
	struct __attribute__((packed)) MSG_SET_ROBOT_STATE {
		uint8_t robot_state; /*<! Desired robot state, use values from @ref RobotStateCommand */
	};

	/// Command robot to stop all current and pending actions
	struct __attribute__((packed)) MSG_SET_STOP {
		uint8_t reserved; /*<! Reserved for future use, set to 0 */
	};

	/// Command robot to pause or resume a trajectory
	struct __attribute__((packed)) MSG_SET_Pause {
		uint8_t command; /*<! Pause command, use values from @ref PauseCommand */
	};

	/* GET MESSAGES - Messages from robot */

	struct __attribute__((packed)) GET_MSG {
		uint8_t type;
		uint8_t seq;
		uint8_t last_rx_seq;
		int8_t last_rx_result;
		uint16_t payload_len;
		char payload[];
	};

	struct __attribute__((packed)) MSG_GET_UDP_PROTOCOL_VERSION {
		uint8_t major;
		uint8_t minor;
	};

	struct __attribute__((packed)) MSG_GET_STATUS {
		uint8_t error_flags;
		uint8_t robot_state;
		uint8_t safety_state;
		char data[];
		// Variable-length data:
		//  - 1x MSG_GET_STATUS_MODULES struct
		//  - 1x MSG_GET_STATUS_ERRORS struct
	};

	// Bitmask for robot error flag in GET_STATUS message robot_state field
	const uint8_t ROBOT_ERROR_MASK = 0x80;

	// Bitmask for robot state in GET_STATUS message robot_state field
	const uint8_t ROBOT_STATE_DETAIL_MASK = 0x7F;

	// Bitmask for safety error flag in GET_STATUS message safety_state field
	const uint8_t SAFETY_ERROR_MASK = 0x80;

	// Bitmask for safe stop state in GET_STATUS message safety_state field
	const uint8_t SAFE_STOP_STATE_MASK = 0x60;

	// Bitmask for safety mod GET_STATUS message safety_state field
	const uint8_t SAFETY_MODE_MASK = 0x1F;

	struct __attribute__((packed)) MSG_GET_STATUS_MODULES {
		uint8_t n_modules;
		uint8_t module_states[];
	};

	// Bitmask for module type in GET_STATUS message module_state field
	const uint8_t MODULE_TYPE_MASK = 0xE0;

	// Bistmask for module error in GET_STATUS message module_state field
	const uint8_t MODULE_ERROR_MASK = 0x10;

	// Bistmask for ethercat not op in GET_STATUS message module_state field
	const uint8_t ETHERCAT_NOT_OP_MASK = 0x08;

	// Bistmask for drive not op in GET_STATUS message module_state field
	const uint8_t DRIVE_NOT_OP_MASK = 0x04;

	struct __attribute__((packed)) MSG_GET_STATUS_ERRORS {
		uint8_t n_errors;
		uint16_t errors[];
	};

	struct __attribute__((packed)) MSG_GET_LAST_OCCURRED_ERRORS {
		uint8_t n_errors; /*!< Number of error structs in data area of this message */
		char data[];      /*!< Variable-length data:
		            - n_errors x @ref MSG_GET_LAST_OCCURRED_ERRORS_ERROR  */
	};

	struct __attribute__((packed)) MSG_GET_LAST_OCCURRED_ERRORS_ERROR {
		uint32_t timestamp;  /*!< UNIX timestamp indicating when the error occurred */
		uint16_t error_code; /*!< Error code indicating which error occurred */
	};

	struct __attribute__((packed)) MSG_GET_DETECTED_MODULES {
		uint8_t n_modules; /*!< Number of detected modules in data area of this message */
		char data[];       /*!< Variable-length data:
		            - n_modules x @ref MSG_GET_DETECTED_MODULES_MODULE */
	};

	struct __attribute__((packed)) MSG_GET_DETECTED_MODULES_MODULE {
		uint32_t id;     /*!< Type ID of the module */
		uint32_t serial; /*!< Serial number of the module - readable serial number stripped of "SN"
		                    and "-" (i.e. SN12-3456789 = 123456789) */
	};

	struct __attribute__((packed)) MSG_GET_JOINT_ABS {
		uint8_t n_joints;       /*!< Number of joint values transmitted in this message */
		int32_t joint_values[]; /*!< Variable-length data: n_joints x int32_t, encoding actual
		                           absolute joint value of robot joint x. Counting starts at the
		                           joint mounted closest to the robot base. Encoding of joint values
		                           in nanoradians / pi. */
	};

	struct __attribute__((packed)) MSG_GET_POSE_ABS {
		int32_t pos_x; /*<! X-component of curent actual absolute cartesian position in micrometers
		                */
		int32_t pos_y; /*<! Y-component of curent actual absolute cartesian position in micrometers
		                */
		int32_t pos_z; /*<! Z-component of curent actual absolute cartesian position in micrometers
		                */
		int32_t euler_z; /*<! Z-component of current actual absolute orientation given in ZYX Euler
		                    angles, encoded in nanorads / pi */
		int32_t euler_y; /*<! Y-component of current actual absolute orientation given in ZYX Euler
		                    angles, encoded in nanorads / pi */
		int32_t euler_x; /*<! X-component of current actual absolute orientation given in ZYX Euler
		                    angles, encoded in nanorads / pi */
	};

	struct __attribute__((packed)) MSG_GET_GLOBAL_VELOCITY {
		uint8_t actual_velocity_percent; /*<! Number between 0 and 200 representing currently active
		                                    velocity, where 0 represents 0% and 200 represents 100%
		                                  */
		uint8_t desired_velocity_percent; /*<! Number between 0 and 200 representing user-specified
		                                     velocity, where 0 represents 0% and 200 represents 100%
		                                   */
	};

	struct __attribute__((packed)) MSG_GET_OUTPUT {
		uint8_t n_banks; /*<! Total number of available I/O banks */
		char data[];     /*<! Variable-length data: n_banks * MSG_GET_OUTPUT_BANK */
	};

	struct __attribute__((packed)) MSG_GET_OUTPUT_BANK {
		uint8_t bank_id;  /*<! ID of the I/O bank */
		uint8_t n_values; /*<! Number of output values in this bank */
		uint8_t values[]; /*<! Variable-length data: n_values * uint8_t, encoding the current state
		                     of the corresponding output */
	};

	struct __attribute__((packed)) MSG_GET_INPUT {
		uint8_t n_banks; /*<! Total number of available I/O banks */
		char data[];     /*<! Variable-length data: n_banks * MSG_GET_INPUT_BANK */
	};

	struct __attribute__((packed)) MSG_GET_INPUT_BANK {
		uint8_t bank_id;  /*<! ID of the I/O bank */
		uint8_t n_values; /*<! Number of input values in this bank */
		uint8_t values[]; /*<! Variable-length data: n_values * uint8_t, encoding the current state
		                     of the corresponding input */
	};

	struct __attribute__((packed)) MSG_GET_PAYLOAD_MASS {
		uint32_t mass;            /*<! Configured payload mass at end effector, in milligrams */
		int32_t center_of_mass_x; /*<! Configured X-distance from distal point of the end effector
		                             (ignoring any additional TCP shift) to the payload's center of
		                             mass */
		int32_t center_of_mass_y; /*<! Configured Y-distance from distal point of the end effector
		                             (ignoring any additional TCP shift) to the payload's center of
		                             mass */
		int32_t center_of_mass_z; /*<! Configured Z-distance from distal point of the end effector
		                             (ignoring any additional TCP shift) to the payload's center of
		                             mass */
	};

	struct __attribute__((packed)) MSG_GET_TCP_SHIFT {
		int32_t pos_x;   /*<! X-offset from the distal frame of the end effector to the configured
		                    tool centerpoint (TCP) frame, in micrometers */
		int32_t pos_y;   /*<! Y-offset from the distal frame of the end effector to the configured
		                    tool centerpoint (TCP) frame, in micrometers */
		int32_t pos_z;   /*<! Z-offset from the distal frame of the end effector to the configured
		                    tool centerpoint (TCP) frame, in micrometers */
		int32_t euler_z; /*<! Z-angle between distal frame of the end effector and the configured
		                    tool centerpoint (TCP) frame, in nanorad/pi */
		int32_t euler_y; /*<! Y-angle between distal frame of the end effector and the configured
		                    tool centerpoint (TCP) frame, in nanorad/pi */
		int32_t euler_x; /*<! X-angle between distal frame of the end effector and the configured
		                    tool centerpoint (TCP) frame, in nanorad/pi */
	};

	struct __attribute__((packed)) MSG_GET_JOINT_TEMPERATURE_PAIRS {
		uint8_t n_joints; /*<! Number of joint temperature structs in this message */
		char data[]; /*<! Variable-length data: n_joints * MSG_GET_JOINT_TEMPERATURE_PAIRS_TEMPS */
	};

	struct __attribute__((packed)) MSG_GET_JOINT_TEMPERATURE_PAIRS_TEMPS {
		int32_t motor_temperature;      /*<! Joint motor temperature, in millidegrees Celsius */
		int32_t controller_temperature; /*<! Joint controller temperature, in millidegrees Celsius
		                                 */
	};

	struct __attribute__((packed)) MSG_GET_JOINT_TORSIONS {
		uint8_t n_joints;         /*<! Number of joint torsion values in this message */
		int32_t joint_torsions[]; /*<! Vairable-length data: n_joints * int32_t, encoding joint
		                             torsion, in nanorad/pi */
	};

	struct __attribute__((packed)) MSG_GET_JOINT_OVERLOADS_PERCENT {
		uint8_t n_joints;          /*<! Number of joint overload values in this message */
		uint8_t overload_values[]; /*<! Variable-length data: n_joints * uint8_t, encoding joint
		                              overload percentage, where 100 indicates a fully overloaded
		                              joint that will be shut down to prevent hardware damage, and 0
		                              indicates no overload condition is present */
	};

	struct __attribute__((packed)) MSG_GET_JOINT_TORQUES {
		uint8_t n_joints;        /*<! Number of joint torque values in this message */
		int32_t torque_values[]; /*<! Variable-length data: n_joints * int32_t, encoding actual
		                            output torque of the robot joints, in millinewton meters */
	};

	/**
	 * @brief Converts the given angle from radians to nanorad/pi for transport.
	 *
	 * @param rad Angle in radians
	 * @return Angle in nanorad/pi
	 */
	int32_t hton_angle(double rad);

	/**
	 * @brief Converts the given angle from nanorad/pi to radians.
	 *
	 * @param nrad_div_pi Angle in nanorad/pi
	 * @return double Angle in radians
	 */
	double ntoh_angle(int32_t nrad_div_pi);

	/**
	 * @brief Converts the given value from meters to micrometers.
	 *
	 * @param meters Value in meters
	 * @return int32_t Value in nanometers
	 */
	int32_t hton_linear(double meters);

	/**
	 * @brief Converts the given value from micrometers to meters.
	 *
	 * @param micrometers Value in micrometers
	 * @return double Value in meters
	 */
	double ntoh_linear(int32_t micrometers);

	/**
	 * @brief Converts the given value from kilograms to milligrams.
	 *
	 * @param kilograms Value in kilograms
	 * @return uint32_t Value in milligrams
	 */
	uint32_t hton_mass(double kilograms);

	/**
	 * @brief Converts the given value from milligrams to kilograms.
	 *
	 * @param milligrams Value in milligrams
	 * @return double Value in meters
	 */
	double ntoh_mass(uint32_t milligrams);

	/**
	 * @brief Converts the given value from newton meters to millinewton meters.
	 *
	 * @param newton_meters Value in newton meters
	 * @return int32_t Value in millinewton meters
	 */
	int32_t hton_torque(double newton_meters);

	/**
	 * @brief Converts the given value from millinewton meters to newton meters.
	 *
	 * @param millinewton_meters Value in millinewton meters
	 * @return double Value in newton meters
	 */
	double ntoh_torque(int32_t millinewton_meters);

	/**
	 * @brief Converts the given value from degrees Celsius to millidegrees Celsius.
	 *
	 * @param degrees_c Value in degrees Celsius
	 * @return int32_t Value in millidegrees Celsius
	 */
	int32_t hton_temperature(double degrees_c);

	/**
	 * @brief Converts the given value from millidegrees Celsius to degrees Celsius.
	 *
	 * @param millidegrees_c Value in millidegrees Celsius
	 * @return double Value in degrees Celsius
	 */
	double ntoh_temperature(int32_t millidegrees_c);

	/**
	 * @brief Returns the size of the given SET message, including its payload.
	 *
	 * @param m Pointer to message struct
	 * @return Size of message in bytes, including payload
	 */
	int len_SET_MSG(SET_MSG* m);

	/**
	 * @brief Allocates a new SET_MSG struct.
	 *
	 * Note: This function allocates memory. It is the caller's responsibility to
	 *       free the returned SET_MSG pointer.
	 *
	 * @param msg_type Message type value
	 * @param seq Sequence number
	 * @param payload_size Size of the payload part of the message
	 * @return SET_MSG*
	 */
	SET_MSG* new_UDP_MSG(uint8_t msg_type, uint8_t seq, size_t payload_size);

	/**
	 * @brief Returns the size of the given SET_JOINT_OFFS message payload in bytes.
	 *
	 * @param m Pointer to payload struct
	 * @return Size of payload in bytes
	 */
	int len_MSG_SET_JOINT_OFFS(MSG_SET_JOINT_OFFS* m);

	/**
	 * @brief Allocastes a new SET_MSG struct with a SET_JOINT_OFFS payload and
	 *	  the given parameters.
	 *
	 * Note: This function allocates memory. It is the caller's responsibility to free
	 *	 the returned SET_MSG pointer.
	 *
	 * @param seq Message sequence number
	 * @param num_joints Number of joints in payload
	 */
	SET_MSG* new_MSG_SET_JOINT_OFFS(uint8_t seq, int num_joints);

	/**
	 * @brief Returns the size of the given GET_STATUS_MODULES message payload in bytes.
	 *
	 * @param m Pointer to payload struct
	 * @return Size of payload in bytes
	 */
	int len_MSG_GET_STATUS_MODULES(MSG_GET_STATUS_MODULES* m);

	/**
	 * @brief Returns the size of the given GET_STATUS_ERRORS message payload in bytes.
	 *
	 * @param m Pointer to payload struct
	 * @return Size of payload in bytes
	 */
	int len_MSG_GET_STATUS_ERRORS(MSG_GET_STATUS_ERRORS* m);

	/**
	 * @brief Get RobotState from the robot_state field of a GET_STATUS message.
	 *
	 * @param robot_state content of robot_state field of GET_STATUS message.
	 * @return RobotState
	 */
	RobotState msg_get_robot_state(uint8_t robot_state);

	/**
	 * @brief Get SafeStopState from safety_state field of GET_STATUS message.
	 *
	 * @param safety_state safety_state field of GET_STATUS message.
	 * @return SafeStopState
	 */
	SafeStopState msg_get_safe_stop_state(uint8_t safety_state);

	/**
	 * @brief Get SafetyMode from safety_state field of GET_STATUS message.
	 *
	 * @param safety_mode safety_mode field of GET_STATUS message.
	 * @return SafetyMode
	 */
	SafetyMode msg_get_safety_mode(uint8_t safety_state);
	
	/**
	 * @brief Get RobotStatus struct from GET_STATUS message.
	 *
	 * @param msg pointer to GET_STATUS message struct.
	 * @return RobotStatus
	 */
	RobotStatus msg_get_robot_status(MSG_GET_STATUS* msg);

	/**
	 * @brief Get ModuleType from module_state field of GET_STATUS message.
	 *
	 * @param module_state module_state value from GET_STATUS message.
	 * @return ModuleType
	 */
	ModuleType msg_get_module_type(uint8_t module_state);

	/**
	 * @brief Get ModuleState for module inside GET_STATUS messaage.
	 *
	 * @param modules pointer to modules part of GET_STATUS message
	 * @param i index of the module to get state for
	 * @return ModuleState of module i
	 */
	ModuleState msg_get_module_state(MSG_GET_STATUS_MODULES* modules, int i);

	void print_UDP_MSG(SET_MSG* m);
} // namespace robcomm

#endif // ROBCOMM_ROBOT_MESSAGES_H

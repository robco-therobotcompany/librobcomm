/** \file types.hpp
 * \brief Type definitions for librobcomm.
 *
 * Created on: 28.03.2023
 * Author: Bernhard Vorhofer
 * Contributor: -
 */

#ifndef ROBCOMM_TYPE_H_
#define ROBCOMM_TYPE_H_

#include <stdint.h>

namespace robcomm {
    /// Safety stop state as reported in GET_STATUS message.
    enum SafeStopState {
        SAFE_STOP_NONE = 0x00,
        SAFE_STOP_0 = 0x03,
        SAFE_STOP_1 = 0x02,
        SAFE_STOP_2 = 0x01
    };

    /// Safety mode as reported in GET_STATUS message.
    enum SafetyMode {
        SAFETY_MODE_MANUAL_REDUCED_VELOCITY = 0x01,
        SAFETY_MODE_AUTOMATIC = 0x02
    };

    /// Detailed robot state values as reported in GET_STATUS messages.
    enum RobotState {
        ROBOT_STATE_ERROR = 0x00,
        ROBOT_STATE_IDLE = 0x01,
        ROBOT_STATE_DECELERATE_INTERNAL = 0x02,
        ROBOT_STATE_DECELERATE_USER = 0x03,
        ROBOT_STATE_GRAVITY_COMPENSATED = 0x04,
        ROBOT_STATE_JOGGING = 0x05,
        ROBOT_STATE_RUN_PTP = 0x06,
        ROBOT_STATE_RUN_LINEAR = 0x07,
        ROBOT_STATE_DISABLED = 0x70,
        ROBOT_STATE_SWITCHED_ON = 0x71,
        ROBOT_STATE_TRANSITION = 0x7F
    };

    /// Robot status as reported in GET_STATUS message.
    struct RobotStatus {
        uint8_t error_flags;
        bool robot_error;
        RobotState robot_state;
        bool safety_error;
        SafeStopState safe_stop_state;
        SafetyMode safety_mode;
    };

    /// Module types as reported in GET_STATUS message.
    enum ModuleType {
        MODULE_TYPE_GENERIC = 0x00,
        MODULE_TYPE_DRIVE = 0x01,
        MODULE_TYPE_LINK = 0x02,
        MODULE_TYPE_IO = 0x04
    };

    /// Commanded robot state for SET_ROBOT_STATE messages.
    enum RobotStateCommand {
        ROBOT_STATE_CMD_DISABLED    = 0x00, // not used
        ROBOT_STATE_CMD_SWITCHED_ON = 0x01,
        ROBOT_STATE_CMD_OPERATIONAL = 0x02
    };

    /// Command values for SET_PAUSE messages.
    enum PauseCommand {
        PAUSE_COMMAND_PAUSE = 0, ///< Pause current trajectory
        PAUSE_COMMAND_RESUME = 1 ///< Resume current trajectory
    };

    /// Coordinate frame to use for a relative movement
    enum CoordinateFrame {
            COORD_FRAME_BASE = 0,    ///< Robot base coordinate frame
            COORD_FRAME_EFFECTOR = 1 ///< End effector coordinate frame
    };

    /// Type of a segment within a constant velocity trajectory
    enum SegmentType {
            SEGMENT_TYPE_LINEAR = 1,     ///< Linear segment without intermedate point
            SEGMENT_TYPE_ARC = 2,        ///< Circular arc passing through intermediate point
            SEGMENT_TYPE_TANGENT_ARC = 3 ///< Tangent arc
    };

    /// Approach mode for a waypoint
    enum ApproachMode {
            APPROACH_MODE_PTP = 1,   ///< Point-to-point approach mode
            APPROACH_MODE_LINEAR = 2 ///< Linear approach mode
    };


    /// Module status as reported in GET_STATUS message.
    struct ModuleState {
        ModuleType type;
        bool module_error;
        bool ethercat_not_op;
        bool drive_not_op;
    };
};

#endif // ROBCOMM_TYPE_H_

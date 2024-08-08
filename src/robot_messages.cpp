#include <robcomm/robot_messages.hpp>
#include <robcomm/types.hpp>
#include <stdint.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdexcept>
#include <sstream>
#include <math.h>

namespace robcomm {

    int32_t hton_angle(double rad) {
        return htonl((int32_t)(rad * 1e9 / M_PI));
    }

    double ntoh_angle(int32_t nrad_div_pi) {
        return (int32_t)ntohl(nrad_div_pi) * 1.0e-9 * M_PI;
    }

    int32_t hton_linear(double meters) {
        return htonl((int32_t)(meters * 1e6));
    }

    double ntoh_linear(int32_t micrometers) {
        return (int32_t)ntohl(micrometers) * 1.0e-6;
    }

    uint32_t hton_mass(double kilograms) {
        return htonl((uint32_t)(kilograms * 1e6));
    }

    double ntoh_mass(uint32_t milligrams) {
        return ntohl(milligrams) * 1.0e-6;
    }

    int32_t hton_torque(double newton_meters) {
        return htonl((int32_t)(newton_meters * 1e3));
    }

    double ntoh_torque(int32_t millinewton_meters) {
        return (int32_t)ntohl(millinewton_meters) * 1.0e-3;
    }

    int32_t hton_temperature(double degrees_c) {
        return htonl((int32_t)(degrees_c * 1e3));
    }

    double ntoh_temperature(int32_t millidegrees_c) {
        return (int32_t)ntohl(millidegrees_c) * 1.0e-3;
    }

    int len_MSG_SET_JOINT_OFFS(MSG_SET_JOINT_OFFS* m) {
        return sizeof(MSG_SET_JOINT_OFFS) + m->num_joints * sizeof(*m->joint_angles);
    }

    int len_SET_MSG(SET_MSG* m) {
        return sizeof(SET_MSG) + ntohs(m->payload_len);
    }

    int len_MSG_GET_STATUS_MODULES(MSG_GET_STATUS_MODULES* m) {
        return sizeof(MSG_GET_STATUS_MODULES) + m->n_modules * sizeof(*m->module_states);
    }

    int len_MSG_GET_STATUS_ERRORS(MSG_GET_STATUS_ERRORS* m) {
        return sizeof(MSG_GET_STATUS_ERRORS) + m->n_errors * sizeof(*m->errors);
    }

    SET_MSG* new_UDP_MSG(uint8_t msg_type, uint8_t seq, size_t payload_size) {
        SET_MSG* msg = (SET_MSG*)malloc(sizeof(SET_MSG) + payload_size);

        msg->type = msg_type;
        msg->seq = seq;
        msg->payload_len = htons((uint16_t)payload_size);

        return msg;
    }

    SET_MSG* new_MSG_SET_JOINT_OFFS(uint8_t seq, int num_joints) {
        size_t payload_size = sizeof(MSG_SET_JOINT_OFFS) + num_joints * sizeof(int32_t);
        SET_MSG* msg = new_UDP_MSG(MSG_TYPE_SET_JOINT_OFFS, seq, payload_size);

        MSG_SET_JOINT_OFFS* payload = (MSG_SET_JOINT_OFFS*)msg->payload;
        payload->num_joints = (uint8_t)num_joints;

        return msg;
    }

    RobotState msg_get_robot_state(uint8_t robot_state) {
        uint8_t state = robot_state & ROBOT_STATE_DETAIL_MASK;

        std::stringstream except_ss;

        switch(state) {
            case ROBOT_STATE_ERROR:
            case ROBOT_STATE_IDLE:
            case ROBOT_STATE_DECELERATE_INTERNAL:
            case ROBOT_STATE_DECELERATE_USER:
            case ROBOT_STATE_GRAVITY_COMPENSATED:
            case ROBOT_STATE_JOGGING:
            case ROBOT_STATE_RUN_PTP:
            case ROBOT_STATE_RUN_LINEAR:
            case ROBOT_STATE_DISABLED:
            case ROBOT_STATE_SWITCHED_ON:
            case ROBOT_STATE_TRANSITION:
                return (RobotState)state;
        }

        except_ss << "Invalid robot state " << state;
        throw std::runtime_error(except_ss.str());
    }

    SafeStopState msg_get_safe_stop_state(uint8_t safety_state) {
        uint8_t state = safety_state & SAFE_STOP_STATE_MASK;
        state = state >> 5;

        std::stringstream except_ss;

        switch(state) {
	    case SAFE_STOP_NONE:
            case SAFE_STOP_0:
            case SAFE_STOP_1:
            case SAFE_STOP_2:
                return (SafeStopState)state;
        }

        except_ss << "Invalid safe stop state " << state;
        throw std::runtime_error(except_ss.str());
    }

    SafetyMode msg_get_safety_mode(uint8_t safety_state) {
        uint8_t mode = safety_state & SAFETY_MODE_MASK;

        std::stringstream except_ss;

        switch(mode) {
            case SAFETY_MODE_MANUAL_REDUCED_VELOCITY:
            case SAFETY_MODE_AUTOMATIC:
                return (SafetyMode)mode;
        }

        except_ss << "Invalid safety mode " << mode;
        throw std::runtime_error(except_ss.str());
    }

    RobotStatus msg_get_robot_status(MSG_GET_STATUS* msg) {
        RobotStatus rs;
        rs.error_flags = msg->error_flags;
        rs.robot_error = (bool)(msg->robot_state & ROBOT_ERROR_MASK);
        rs.robot_state = msg_get_robot_state(msg->robot_state);
        rs.safety_error = (bool)(msg->safety_state & SAFETY_ERROR_MASK);
        rs.safe_stop_state = msg_get_safe_stop_state(msg->safety_state);
        rs.safety_mode = msg_get_safety_mode(msg->safety_state);

        return rs;
    }

    ModuleType msg_get_module_type(uint8_t module_state) {
        std::stringstream except_ss;
        uint8_t mtype = module_state & MODULE_TYPE_MASK;
        mtype = mtype >> 5;

        switch (mtype) {
            case MODULE_TYPE_GENERIC:
            case MODULE_TYPE_DRIVE:
            case MODULE_TYPE_LINK:
            case MODULE_TYPE_IO:
                return (ModuleType)mtype;
        }

        except_ss << "Invalid module type " << mtype;
        throw std::runtime_error(except_ss.str());
    }

    ModuleState msg_get_module_state(MSG_GET_STATUS_MODULES* modules, int i) {
        ModuleState ms;

        if (i < 0 || i > modules->n_modules - 1)
            throw std::out_of_range("Module index out of range");

        ms.type = msg_get_module_type(modules->module_states[i]);
        ms.module_error = modules->module_states[i] & MODULE_ERROR_MASK;
        ms.ethercat_not_op = modules->module_states[i] & ETHERCAT_NOT_OP_MASK;
        ms.drive_not_op = modules->module_states[i] & DRIVE_NOT_OP_MASK;

        return ms;
    }
}

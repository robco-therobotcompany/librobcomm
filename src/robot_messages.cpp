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
        return (double)ntohl(nrad_div_pi) * 1e-9 * M_PI;
    }

    int32_t hton_linear(double meters) {
        return htonl((int32_t)(meters * 1e6));
    }

    double ntoh_linear(int32_t micrometers) {
        return (double)ntohl(micrometers) * 1e-6;
    }

    int len_MSG_SET_JOINT_OFFS(MSG_SET_JOINT_OFFS* m) {
        return sizeof(MSG_SET_JOINT_OFFS) + m->num_joints * sizeof(*m->joint_angles);
    }

    int len_UDP_MSG(SET_MSG* m) {
        return sizeof(SET_MSG) + ntohs(m->payload_len);
    }

    int len_MSG_GET_STATUS_MODULES(MSG_GET_STATUS_MODULES* m) {
        return sizeof(MSG_GET_STATUS_MODULES) + m->n_modules * sizeof(*m->module_states);
    }

    int len_MSG_GET_STATUS_ERRORS(MSG_GET_STATUS_ERRORS* m) {
        return sizeof(MSG_GET_STATUS_ERRORS) + m->n_errors * sizeof(*m->errors);
    }

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

    /**
     * @brief Get RobotState from the robot_state field of a GET_STATUS message.
     * 
     * @param robot_state content of robot_state field of GET_STATUS message.
     * @return RobotState 
     */
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

    /**
     * @brief Get SafeStopState from safety_state field of GET_STATUS message.
     * 
     * @param safety_state safety_state field of GET_STATUS message.
     * @return SafeStopState 
     */
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

    /**
     * @brief Get SafetyMode from safety_state field of GET_STATUS message.
     * 
     * @param safety_mode safety_mode field of GET_STATUS message.
     * @return SafetyMode 
     */
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

    /**
     * @brief Get RobotStatus struct from GET_STATUS message.
     * 
     * @param msg pointer to GET_STATUS message struct.
     * @return RobotStatus 
     */
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

    /**
     * @brief Get ModuleType from module_state field of GET_STATUS message.
     * 
     * @param module_state module_state value from GET_STATUS message.
     * @return ModuleType 
     */
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

    /**
     * @brief Get ModuleState for module inside GET_STATUS messaage.
     * 
     * @param modules pointer to modules part of GET_STATUS message
     * @param i index of the module to get state for 
     * @return ModuleState of module i
     */
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

    void print_UDP_MSG(SET_MSG* m) {
        const char* msgtypestr;

        if (m->type == MSG_TYPE_SET_JOINT_OFFS)
            msgtypestr = "SET_JOINT_OFFS";
        else if (m->type == MSG_TYPE_SET_ROBOT_STATE)
            msgtypestr = "SET_ROBOT_STATE";
        else
            msgtypestr = "UNKNOWN";

        printf("type: %#04x (%s)\n", m->type, msgtypestr);
        printf("seq: %d\n", m->seq);
        printf("payload_len: %d\n", ntohs(m->payload_len));

        if (m->type == MSG_TYPE_SET_JOINT_OFFS) {
            MSG_SET_JOINT_OFFS* payload = (MSG_SET_JOINT_OFFS*)m->payload;
            printf("num_joints: %d\n", payload->num_joints);
            printf("joint_angles:\n");
            for (int i = 0; i < payload->num_joints; i ++) {
                printf("  %02d: %10d (%02.5f)\n", i, ntohl(payload->joint_angles[i]), ntoh_angle(payload->joint_angles[i]));
            }
        } else if (m->type == MSG_TYPE_SET_ROBOT_STATE) {
            MSG_SET_ROBOT_STATE* payload = (MSG_SET_ROBOT_STATE*)m->payload;
            printf("robot_state: %d\n", payload->robot_state);
        }
    }
}

/** \file module.hpp
 * \brief Module class, represents a generic module on the RobCo robot.
 *
 * Created on: 26.07.2024
 * Author: Bernhard Vorhofer
 * Contributor: -
 *
 * Copyright (C) 2024 RobCo GmbH - All Rights Reserved
 *
 */

#ifndef ROBCOMM_MODULE_H
#define ROBCOMM_MODULE_H

#include <cstdint>
#include "robcomm/types.hpp"

namespace robcomm {
    class Module {
        public:
            Module();
            Module(uint32_t id);
            ~Module();

            const uint32_t module_id() const;
            const ModuleState module_state() const;
        private:
            uint32_t _module_id;
            ModuleState _state;

        friend class Robot;
    };
}

#endif // ROBCOMM_MODULE_H

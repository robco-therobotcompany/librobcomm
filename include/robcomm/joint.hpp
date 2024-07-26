/** \file joint.hpp
 * \brief Joint class, represents a drive module on the RobCo robot.
 *
 * Created on: 26.07.2024
 * Author: Bernhard Vorhofer
 * Contributor: -
 *
 * Copyright (C) 2024 RobCo GmbH - All Rights Reserved
 *
 */

#ifndef ROBCOMM_JOINT_H
#define ROBCOMM_JOINT_H

#include <robcomm/module.hpp>

namespace robcomm {
    class Joint {
        public:
            Joint();
            ~Joint();

            /**
             * @brief Returns the module representing this joint.
             *
             * @return Module object this joint is associated with
             */
            const Module& module() const;

            /**
             * @brief Returns the current joint angle in radians.
             *
             * @return Joint angle in rad
             */
            const double q() const ;

            /**
             * @brief Returns the current joint torque in Newton meters.
             *
             * @return Joint torque in Nm
             */
            const double torque() const;

            /**
             * @brief Returns the current joint torsion in radians.
             *
             * @return Joint torsion in rad
             */
            const double torsion() const ;


            /**
             * @brief Returns the current joint overload value in percent.
             *
             * @return Joint overload in percent
             */
            const int overload() const ;

            /**
             * @brief Returns the current joint motor temperature in degrees Celsius.
             *
             * @return Motor temperature in degrees C
             */
            const double motor_temperature() const;

            /**
             * @brief Returns the current joint motor controller temperature in degrees Celsius.
             *
             * @return Motor controller temperature in degrees C
             */
            const double controller_temperature() const;
        private:
            Module _module;
            double _q;
            double _motor_temperature_deg_c;
            double _controller_temperature_deg_c;
            double _torsion_rad;
            uint8_t _overload_percent;
            double _torque_n;

        friend class Robot;
    };
}

#endif // ROBCOMM_JOINT_H

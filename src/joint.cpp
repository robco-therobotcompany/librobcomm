#include <robcomm/joint.hpp>

namespace robcomm {
	Joint::Joint() {}
	Joint::~Joint() {}

	const Module& Joint::module() const {
		return _module;
	}

	const double Joint::q() const {
		return _q;
	}

	const double Joint::torque() const {
		return _torque_n;
	}

	const double Joint::torsion() const {
		return _torsion_rad;
	}

	const int Joint::overload() const {
		return _overload_percent;
	}

	const double Joint::motor_temperature() const {
		return _motor_temperature_deg_c;
	}

	const double Joint::controller_temperature() const {
		return _controller_temperature_deg_c;
	}
} // namespace robcomm

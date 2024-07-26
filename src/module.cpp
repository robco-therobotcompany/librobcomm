#include <robcomm/module.hpp>

namespace robcomm {
	Module::Module() {}

	Module::~Module() {}

	const uint32_t Module::module_id() const {
		return _module_id;
	}

	const ModuleState Module::module_state() const {
		return _state;
	}
} // namespace robcomm

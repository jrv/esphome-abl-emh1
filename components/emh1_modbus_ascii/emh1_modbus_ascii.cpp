#include "emh1_modbus_ascii.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace emh1_modbus_ascii {

void Emh1Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
	    this->flow_control_pin_->setup();
	}
}

}  // namespace emh1_modbus_ascii
}  // namespace esphome

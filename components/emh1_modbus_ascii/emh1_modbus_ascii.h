#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace emh1_modbus_ascii {

class Emh1Modbus : public uart::UARTDevice, public Component {
  public:
	  Emh1Modbus() = default;
		
		void setup() override;
		void loop() override;
    void set_enable_pin(GPIOPin *enable_pin) { this->enable_pin_ = enable_pin; }

	protected:
	  GPIOPin *enable_pin_{nullptr};
}

}  // namespace emh1_modbus_ascii
}  // namespace esphome

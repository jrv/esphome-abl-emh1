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

}  // namespace emh1_modbus_ascii
}  // namespace esphome

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/emh1_modbus/emh1_modbus.h"

namespace esphome {
namespace abl_emh1 {

static const uint8_t REDISCOVERY_THRESHOLD = 5;
static const uint16_t CONFIG_AGE_THRESHOLD = 10;

// class ABLeMH1;
// ABLeMH1 *my_abl_emh1;

class ABLeMH1: public PollingComponent, public emh1_modbus::eMH1ModbusDevice {
 public:
  void set_mode_sensor(sensor::Sensor *mode_sensor) { mode_sensor_ = mode_sensor; }
  void set_l1_current_sensor(sensor::Sensor *l1_current_sensor) { l1_current_sensor_ = l1_current_sensor; }
  void set_l2_current_sensor(sensor::Sensor *l2_current_sensor) { l2_current_sensor_ = l2_current_sensor; }
  void set_l3_current_sensor(sensor::Sensor *l3_current_sensor) { l3_current_sensor_ = l3_current_sensor; }
	void set_max_current_sensor(sensor::Sensor *max_current_sensor) { max_current_sensor_ = max_current_sensor; }
	void set_en1_status_sensor(sensor::Sensor *en1_status_sensor) { en1_status_sensor_ = en1_status_sensor; }
	void set_en2_status_sensor(sensor::Sensor *en2_status_sensor) { en2_status_sensor_ = en2_status_sensor; }
	void set_duty_cycle_reduced_sensor(sensor::Sensor *duty_cycle_reduced_sensor) { duty_cycle_reduced_ = duty_cycle_reduced_sensor; }
	void set_ucp_status_sensor(sensor::Sensor *ucp_status_sensor) { ucp_status_sensor_ = ucp_status_sensor; }
	void set_outlet_state_sensor(sensor::Sensor *outlet_state_sensor) { outlet_state_sensor_ = outlet_state_sensor; }
	void set_mode_name_text_sensor(text_sensor::TextSensor *mode_name_text_sensor) { mode_name_text_sensor_ = mode_name_text_sensor; }
  void set_serial_number_text_sensor(text_sensor::TextSensor *serial_number_text_sensor) { serial_number_text_sensor_ = serial_number_text_sensor; }

  void update() override;
  void on_emh1_modbus_data(uint16_t function, uint16_t datalength, const uint8_t* data) override;
  void dump_config() override;

 protected:
  sensor::Sensor *mode_sensor_;
  sensor::Sensor *l1_current_sensor_;
  sensor::Sensor *l2_current_sensor_;
  sensor::Sensor *l3_current_sensor_;
  sensor::Sensor *max_current_sensor_;
  sensor::Sensor *en1_status_sensor_;
  sensor::Sensor *en2_status_sensor_;
  sensor::Sensor *duty_cycle_reduced_;
  sensor::Sensor *ucp_status_sensor_;
  sensor::Sensor *outlet_state_sensor_;

  text_sensor::TextSensor *mode_name_text_sensor_;
  text_sensor::TextSensor *serial_number_text_sensor_;
  uint8_t no_response_count_ = REDISCOVERY_THRESHOLD;
	uint16_t config_age_ = CONFIG_AGE_THRESHOLD;

  void decode_serial_number_(const uint8_t* data, uint16_t datalength);
  void decode_device_info_(const uint8_t* data, uint16_t datalength);
  void decode_status_report_(const uint8_t* data, uint16_t datalength);
  void decode_config_settings_(const uint8_t* data, uint16_t datalength);
  void publish_state_(sensor::Sensor *sensor, float value);
  void publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state);
  void publish_device_offline_();
};


}  // namespace abl_emh1
}  // namespace esphome

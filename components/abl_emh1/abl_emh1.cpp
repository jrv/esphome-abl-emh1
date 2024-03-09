#include "abl_emh1.h"
#include "esphome/core/log.h"

namespace esphome {
namespace abl_emh1 {

static const char *const TAG = "abl_emh1";

static const uint8_t FUNCTION_STATUS_REPORT = 0x002E;
static const uint8_t FUNCTION_DEVICE_INFO = 0x002C;
static const uint8_t FUNCTION_CONFIG_SETTINGS = 0x0001;
static const uint8_t FUNCTION_GET_SERIAL = 0x0050;

static const uint8_t MODES_SIZE = 7;
static const std::string MODES[MODES_SIZE] = {
    "Wait",             // 0
    "Check",            // 1
    "Normal",           // 2
    "Fault",            // 3
    "Permanent Fault",  // 4
    "Update",           // 5
    "Self Test",        // 6
};

static const uint8_t STATE_SIZE = 13;
static const char *const STATE[STATE_SIZE] = {
	"Waiting for EV",													// A1
	"EV is asking for charging", 							// B1
	"EV has the permission to charge",				// B2
	"EV is charging",													// C2
	"C2, reduced current (error F16, F17)",		// C3
	"C2, reduced current (imbalance F15)",		// C4
	"Outlet disabled",												// E0
	"Production test",												// E1
	"EVCC setup mode",												// E2
	"Bus idle",																// E3
	"Unintended closed contact (Welding)",		// F1
	"Internal error",													// F2
  "Unknown State code"											// default
};
static const char STATECODE[STATE_SIZE] = {
  0xA1, 0xB1, 0xB2, 0xC2, 0xC3, 0xc4, 
	0xE0, 0xE1, 0xE2, 0xE3, 0xF1, 0xF2, 0x00
};

void ABLeMH1::on_emh1_modbus_data(uint16_t function, uint16_t datalength, const uint8_t* data) {
  switch (function) {
    case FUNCTION_DEVICE_INFO:
      this->decode_device_info_(data, datalength);
      break;
    case FUNCTION_STATUS_REPORT:
      this->decode_status_report_(data, datalength);
      break;
    case FUNCTION_CONFIG_SETTINGS:
      this->decode_config_settings_(data, datalength);
      break;
    case FUNCTION_GET_SERIAL:
      this->decode_serial_number_(data, datalength);
      break;
    default:
      // ESP_LOGW(TAG, "Unhandled ABL frame: %s", format_hex_pretty(&data.front(), data.size()).c_str());
      ESP_LOGW(TAG, "Unhandled ABL frame");
  }
}

void ABLeMH1::decode_serial_number_(const uint8_t* data, uint16_t datalength) {
  if (datalength != 8) {
	  ESP_LOGW(TAG, "Serial number length problem, detected %n", datalength);
	}
  char buffer[15];
  for (int x=2; x<16; x++) {
	  if (data[x] == 0x20) 
		  buffer[x] = '\0';
	  else
		  buffer[x] = data[x];
	}
	buffer[14] = '\0';
	ESP_LOGI(TAG, "Serial: %s", buffer);
	
/*
	uint8_t dmax = d
	if (dmax > 14) dmax = 14;
	for (int x=2; x<16; x++) {
	  buffer[x] = chr(data[x+2]);
	}
	char buffer[15] = '\0';
	for (int x=0; x < dmax; x++) {
	  buffer[x] += data[x+2];
	}
	// this->publish_state_(this->serial_number_text_sensor_, "2W22xy01234567");
  // this->publish_state_(this->serial_number_text_sensor_, buffer);
	ESP_LOGD(TAG, "Serial number: %s", data);
	*/
  this->no_response_count_ = 0;
}

void ABLeMH1::decode_device_info_(const uint8_t* data, uint16_t datalength) {
  ESP_LOGI(TAG, "Device info frame received");
  //ESP_LOGI(TAG, "  Device type: %d", data[0]);
  //ESP_LOGI(TAG, "  Rated power: %s", std::string(data.begin() + 1, data.begin() + 1 + 6).c_str());
  //ESP_LOGI(TAG, "  Firmware version: %s", std::string(data.begin() + 7, data.begin() + 7 + 5).c_str());
  //ESP_LOGI(TAG, "  Module name: %s", std::string(data.begin() + 12, data.begin() + 12 + 14).c_str());
  //ESP_LOGI(TAG, "  Manufacturer: %s", std::string(data.begin() + 26, data.begin() + 26 + 14).c_str());
  //ESP_LOGI(TAG, "  Serial number: %s", std::string(data.begin() + 40, data.begin() + 40 + 14).c_str());
  //ESP_LOGI(TAG, "  Rated bus voltage: %s", std::string(data.begin() + 54, data.begin() + 54 + 4).c_str());
  this->no_response_count_ = 0;
}

void ABLeMH1::decode_config_settings_(const uint8_t* data, uint16_t datalength) {
  //if (data.size() != 68) {
  //  ESP_LOGW(TAG, "Invalid response size: %zu", data.size());
  //  return;
  //}

//  auto emh1_get_16bit = [&](size_t i) -> uint16_t {
//    return (uint16_t(data[i + 0]) << 8) | (uint16_t(data[i + 1]) << 0);
//  };

  ESP_LOGI(TAG, "Config settings frame received");
  //ESP_LOGI(TAG, "  wVpvStart [9.10]: %f V", emh1_get_16bit(0) * 0.1f);
  //ESP_LOGI(TAG, "  wTimeStart [11.12]: %d S", emh1_get_16bit(2));
  //ESP_LOGI(TAG, "  wVacMinProtect [13.14]: %f V", emh1_get_16bit(4) * 0.1f);

  this->no_response_count_ = 0;
}

void ABLeMH1::decode_status_report_(const uint8_t* data, uint16_t datalength) {
  ESP_LOGI(TAG, "Status frame received");
	if (data[0] != 0x2E) {
	  ESP_LOGD(TAG, "Expected data[0] to be 0x2E");
		return;
	}
	uint8_t x;
	for (x=0; x < STATE_SIZE; x++) {
	  if (data[1] == STATECODE[x]) break;
	}
  this->publish_state_(this->outlet_state_sensor_, STATECODE[x]);
  this->publish_state_(this->mode_sensor_, STATECODE[x]);
  this->publish_state_(this->mode_name_text_sensor_, STATE[x]);
  this->publish_state_(this->en1_status_sensor_, (data[2] & 0x10) >> 4);
  this->publish_state_(this->en2_status_sensor_, (data[2] & 0x20) >> 5);
  this->publish_state_(this->duty_cycle_reduced_, (data[2] & 0x40) >> 6);
  this->publish_state_(this->ucp_status_sensor_, (data[2] & 0x80) >> 7);
	if (STATECODE[x] == 0xA1) {
    this->publish_state_(this->l1_current_sensor_, 0.0);
    this->publish_state_(this->l2_current_sensor_, 0.0);
    this->publish_state_(this->l3_current_sensor_, 0.0);
  } else {
    this->publish_state_(this->l1_current_sensor_, 
  	  ((data[4] << 4) + data[5]) / 10.0);
    this->publish_state_(this->l2_current_sensor_,
      ((data[6] << 4) + data[7]) / 10.0);
    this->publish_state_(this->l3_current_sensor_, 
  	  ((data[8] << 4) + data[9]) / 10.0);
	}
	uint8_t v1 = data[2] & 0x03;
	uint8_t v2 = data[3];
	float v = (v1 * 256 + v2) * 1000.0 / 16625.0;
	ESP_LOGD(TAG, "Read max current value 0x%02X 0x%02X", v1, v2);
  this->publish_state_(this->max_current_sensor_, v);
  this->no_response_count_ = 0;
}

void ABLeMH1::publish_device_offline_() {
  this->publish_state_(this->mode_sensor_, -1);
  this->publish_state_(this->l1_current_sensor_, NAN);
  this->publish_state_(this->l2_current_sensor_, NAN);
  this->publish_state_(this->l3_current_sensor_, NAN);
  this->publish_state_(this->max_current_sensor_, NAN);
  this->publish_state_(this->en1_status_sensor_, NAN);
  this->publish_state_(this->en2_status_sensor_, NAN);
  this->publish_state_(this->duty_cycle_reduced_, NAN);
  this->publish_state_(this->ucp_status_sensor_, NAN);
  this->publish_state_(this->outlet_state_sensor_, NAN);
  this->publish_state_(this->mode_name_text_sensor_, "Offline");
  this->publish_state_(this->serial_number_text_sensor_, "");
}

void ABLeMH1::update() {
  if (this->config_age_ >= CONFIG_AGE_THRESHOLD) {
    ESP_LOGD(TAG, "Get device serial numer");
	  this->get_serial();
		this->config_age_ = 0;
	  return;
	}
	if (this->no_response_count_ >= REDISCOVERY_THRESHOLD) {
    this->publish_device_offline_();
    ESP_LOGD(TAG, "The device is or was offline. Broadcasting discovery for address configuration...");
    this->get_serial();
    // this->query_device_info(this->address_);
    // Try to query live data on next update again. The device doesn't
    // respond to the discovery broadcast if it's already configured.
    this->no_response_count_ = 0;
  } else {
    this->no_response_count_++;
		this->get_serial();
  }
}

void ABLeMH1::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

void ABLeMH1::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;

  text_sensor->publish_state(state);
}

void ABLeMH1::dump_config() {
  ESP_LOGCONFIG(TAG, "ABLeMH1:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
//  LOG_SENSOR("", "Temperature", this->temperature_sensor_);
//  LOG_SENSOR("", "Energy today", this->energy_today_sensor_);
//  LOG_SENSOR("", "DC1 voltage", this->dc1_voltage_sensor_);
//  LOG_SENSOR("", "DC2 voltage", this->dc2_voltage_sensor_);
//  LOG_SENSOR("", "DC1 current", this->dc1_current_sensor_);
//  LOG_SENSOR("", "DC2 current", this->dc2_current_sensor_);
//  LOG_SENSOR("", "AC current", this->ac_current_sensor_);
//  LOG_SENSOR("", "AC voltage", this->ac_voltage_sensor_);
//  LOG_SENSOR("", "AC frequency", this->ac_frequency_sensor_);
//  LOG_SENSOR("", "AC power", this->ac_power_sensor_);
//  LOG_SENSOR("", "Energy total", this->energy_total_sensor_);
//  LOG_SENSOR("", "Runtime total", this->runtime_total_sensor_);
//  LOG_SENSOR("", "Mode", this->mode_sensor_);
//  LOG_SENSOR("", "Error bits", this->error_bits_sensor_);
//  LOG_SENSOR("", "Grid voltage fault", this->grid_voltage_fault_sensor_);
//  LOG_SENSOR("", "Grid frequency fault", this->grid_frequency_fault_sensor_);
//  LOG_SENSOR("", "DC injection fault", this->dc_injection_fault_sensor_);
//  LOG_SENSOR("", "Temperature fault", this->temperature_fault_sensor_);
//  LOG_SENSOR("", "PV1 voltage fault", this->pv1_voltage_fault_sensor_);
//  LOG_SENSOR("", "PV2 voltage fault", this->pv2_voltage_fault_sensor_);
//  LOG_SENSOR("", "GFC fault", this->gfc_fault_sensor_);
//  LOG_TEXT_SENSOR("  ", "Mode name", this->mode_name_text_sensor_);
//  LOG_TEXT_SENSOR("  ", "Errors", this->errors_text_sensor_);
}

}  // namespace abl_emh1
}  // namespace esphome

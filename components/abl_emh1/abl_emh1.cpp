#include "abl_emh1.h"
#include "esphome/core/log.h"

namespace esphome {
namespace abl_emh1 {

static const char *const TAG = "abl_emh1";

static const uint8_t FUNCTION_STATUS_REPORT = 0x002E;
static const uint8_t FUNCTION_GET_SERIAL = 0x0050;

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
    case FUNCTION_STATUS_REPORT:
      this->decode_status_report_(data, datalength);
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
	int x=0;
	while ((x<14) && (data[x+2] != 0x20)) {
		buffer[x] = data[x+2];
		x++;
	}
	buffer[x] = '\0';
	ESP_LOGD(TAG, "Serial number: %s", buffer);
  this->publish_state_(this->serial_number_text_sensor_, buffer);
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
  this->publish_state_(this->mode_text_sensor_, STATE[x]);
  this->publish_state_(this->en1_status_sensor_, (data[2] & 0x10) >> 4);
  this->publish_state_(this->en2_status_sensor_, (data[2] & 0x20) >> 5);
  this->publish_state_(this->duty_cycle_reduced_, (data[2] & 0x40) >> 6);
  this->publish_state_(this->ucp_status_sensor_, (data[2] & 0x80) >> 7);
	if ((STATECODE[x] == 0xC2) || (STATECODE[x] == 0xC3) || (STATECODE[x] == 0xC4)) {
     this->publish_state_(this->l1_current_sensor_, 
  	  ((data[4] << 8) + data[5]) / 10.0);
    this->publish_state_(this->l2_current_sensor_,
      ((data[6] << 8) + data[7]) / 10.0);
    this->publish_state_(this->l3_current_sensor_, 
  	  ((data[8] << 8) + data[9]) / 10.0);
  } else {
   	this->publish_state_(this->l1_current_sensor_, 0.0);
    this->publish_state_(this->l2_current_sensor_, 0.0);
    this->publish_state_(this->l3_current_sensor_, 0.0);
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
  this->publish_state_(this->outlet_state_sensor_, 0x00);
  this->publish_state_(this->mode_text_sensor_, "Offline");
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
		this->query_status_report();
    this->no_response_count_++;
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
  LOG_SENSOR("", "Outlet State", this->outlet_state_sensor_);
  LOG_SENSOR("", "Mode", this->mode_sensor_);
  LOG_SENSOR("", "EN1 Status", this->en1_status_sensor_);
  LOG_SENSOR("", "EN2 Status", this->en2_status_sensor_);
  LOG_SENSOR("", "Duty cycle reduced", this->duty_cycle_reduced_);
  LOG_SENSOR("", "UCP Status", this->ucp_status_sensor_);
  LOG_SENSOR("", "L1 Current", this->l1_current_sensor_);
  LOG_SENSOR("", "L2 Current", this->l2_current_sensor_);
  LOG_SENSOR("", "L3 Current", this->l3_current_sensor_);
  LOG_SENSOR("", "Max current", this->max_current_sensor_);
  LOG_TEXT_SENSOR("  ", "Mode name", this->mode_text_sensor_);
}

}  // namespace abl_emh1
}  // namespace esphome

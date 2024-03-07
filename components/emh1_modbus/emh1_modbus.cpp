#include "emh1_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome.h"
#include <cmath>

namespace esphome {
namespace emh1_modbus {

static const char *const TAG = "emh1_modbus";

void eMH1Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
     this->flow_control_pin_->setup();
  }
	eMH1MessageT *tx_message = &this->emh1_tx_message;
	tx_message->DeviceId = 0x01;
	tx_message->FunctionCode = 0x03;
	tx_message->Destination = 0x002E;
	tx_message->DataLength = 0x0005;
	tx_message->LRC = 0x00;
	tx_message->WriteBytes = 0x00;
}

void eMH1Modbus::loop() {
  const uint32_t now = millis();
  if (now - this->last_emh1_modbus_byte_ > 50) {
    this->rx_buffer_.clear();
    this->last_emh1_modbus_byte_ = now;
  }
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_emh1_modbus_byte_(byte)) {
      this->last_emh1_modbus_byte_ = now;
    } else {
      this->rx_buffer_.clear();
    }
  }
}

uint8_t ascii2uint8(const char* value) {
  char c1 = value[0];
  uint8_t highBits = (c1 > '9')?(c1-55):(c1-48);
  char c2 = value[1];
  uint8_t lowBits = (c2 > '9')?(c2-55):(c2-48);
  return (highBits << 4 | lowBits);
}

uint16_t ascii2uint16(const char* value) {
  uint16_t res = 0;
  char c;
  uint16_t bits;
  for (uint8_t x=0; x<4; x++) {
    c = value[x];
    bits = (c > '9')?(c-55):(c-48);
    res = (res << 4 | bits);
  }
  return res;
}

uint8_t lrc(const char* value, uint8_t l) {
  uint8_t lrc_ = 0;
  for (int i = 0; i < l-1; i = i + 2) {
    lrc_ -= ascii2uint8(&value[i]);
  }
  return lrc_;
}

bool eMH1Modbus::parse_emh1_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  if (byte != 0x0A) // 0x0A == LF == End of transmission
    return true;
  this->rx_buffer_.push_back('\0');
  char *frame = &this->rx_buffer_[0];
  eMH1MessageT *rx_message;

  // check LRC
  uint8_t lrc1 = ascii2uint8(&frame[at-3]);
  uint8_t lrc2 = lrc(&frame[1], at-3);
  if (lrc1 != lrc2) {
    ESP_LOGW(TAG, "LRC check failed, discarding transmission %02X != %02X", lrc1, lrc2);
  	return false;
  }

  // check contents of first byte
  switch (frame[0]) {
    case ':':
      // ESP_LOGD(TAG, "Ignore Master transmission: %s", frame);
	    return false;
		case '>':
    	ESP_LOGD(TAG, "Received client transmission: %s", frame);
			break;
		default:
      ESP_LOGW(TAG, "Unknown broadcast data: %s", frame);
		  return false;
	}

  // Check Device ID
	uint8_t r = ascii2uint8(&frame[1]);
	if (r != 0x01) {
	  ESP_LOGW(TAG, "ERROR: Received from device ID: 0x%02X", r);
		return false;
  }

	// Check Function Code
  r = ascii2uint8(&frame[3]);
	uint16_t v;
	switch(r) {
	  case 0x03:
      // ESP_LOGD(TAG, "Response to read operation");
			r = ascii2uint8(&frame[5]);
	    ESP_LOGD(TAG, "Receiving %u bytes", r);
			if (r == rx_message->DataLength * 2) {
				// ESP_LOGD(TAG, "Send data upwards");
				for (uint8_t x = 0; x<r; x++) {
				  rx_message->Data[x] = ascii2uint8(&frame[7+x*2]);
				}
  			bool found = false;
  			for (auto *device : this->devices_) {
    		  if (device->address_ == rx_message->DeviceId) {
            device->on_emh1_modbus_data(rx_message->Destination, rx_message->DataLength, rx_message->Data);
						found = true;
      		}
    		}
  			if (!found) {
    		  ESP_LOGW(TAG, "Got eMH1 frame from unknown device address");
  			}
			} else {
				ESP_LOGW(TAG, "Response data size mismatch, expected %u got %u bytes", this->emh1_rx_message.DataLength * 2, r);
			}
			break;
		case 0x10:
      ESP_LOGD(TAG, "Response to write operation");
      // Read eMH1 starting address
	    v = ascii2uint16(&frame[5]);
	    ESP_LOGD(TAG, "Starting address: 0x%04X", v);
			break;
	  case 0x90:
      ESP_LOGW(TAG, "Error response");
			break;
		default:
      ESP_LOGW(TAG, "Unknown response type");
  }
	this->rx_buffer_.clear();
  ESP_LOGD(TAG, "Cleared buffer");

	return true;

}

void eMH1Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "eMH1Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  this->check_uart_settings(38400);
}

float eMH1Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void eMH1Modbus::query_status_report() {
  ESP_LOGW(TAG, "Query Status Report");
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = 0x01;
	tx_message->FunctionCode = 0x03;
	tx_message->Destination = 0x002E;
	tx_message->DataLength = 0x0005;
  this->send();
}

void eMH1Modbus::query_device_info() {
  ESP_LOGW(TAG, "Query Device Info");
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = 0x01;
	tx_message->FunctionCode = 0x03;
	tx_message->Destination = 0x002C;
	tx_message->DataLength = 0x0001;
  this->send();
}

void eMH1Modbus::query_config_settings() {
  ESP_LOGW(TAG, "Query Config Settings");
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = 0x01;
	tx_message->FunctionCode = 0x03;
	tx_message->Destination = 0x0001;
	tx_message->DataLength = 0x0002;
  this->send();
}

void eMH1Modbus::get_serial() {
  ESP_LOGW(TAG, "Get Serial Number");
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = 0x01;
	tx_message->FunctionCode = 0x03;
	tx_message->Destination = 0x0050;
	tx_message->DataLength = 0x0008;
  this->send();
}

// TODO: kijk of het zonder de bovenste twee hexencode_ascii definities kan?!

uint8_t eMH1Modbus::hexencode_ascii(uint8_t val, char* outStr, uint8_t offset) {
  ESP_LOGW("Using hexencode_ascii 1");
  uint8_t highBits = (val & 0xF0) >> 4;
  uint8_t lowBits = (val & 0x0F);
  outStr[offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
  outStr[offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
	return offset+2;
}

uint8_t eMH1Modbus::hexencode_ascii(uint16_t val, char* outStr, uint8_t offset) {
  ESP_LOGW("Using hexencode_ascii 2");
  uint8_t highBits = (val & 0xF000) >> 12;
  uint8_t lowBits = (val & 0x0F00) >> 8;
  outStr[offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
  outStr[offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
  highBits = (val & 0x00F0) >> 4;
  lowBits = (val & 0x000F);
  outStr[offset+2] = (highBits > 0x09)?(highBits+55):(highBits+48);
  outStr[offset+3] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
	return offset+4;
}

uint8_t eMH1Modbus::hexencode_ascii(uint8_t* val, char* outStr, uint8_t offset, uint8_t cnt) {
  ESP_LOGW("Using hexencode_ascii 3");
  for (uint8_t x=0; x<cnt; x++) { 
    uint8_t highBits = (val[x] & 0xF0) >> 4;
    uint8_t lowBits = (val[x] & 0x0F);
    outStr[2*x+offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
    outStr[2*x+offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
  }
	return offset+cnt*2;
}

void eMH1Modbus::send_current(uint8_t x) {
  // example: ":0110002D00010200A632";
	// 0x01 = address
	// 0x10 = write operation
	// 0x0014 = Set Ic max
	// 0x0001 = 1 16-bit register
	// 0x02 = quantity of value bytes
	// 0x00A6 = actual value (166 = 16.6%) = 10A
	// 0x32 = LRC
  ESP_LOGW(TAG, "Set Max Current to %d Amps", x);
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = 0x01;				// default address
	tx_message->FunctionCode = 0x10;		// write operation
	tx_message->Destination = 0x0014;		// 
	tx_message->DataLength = 0x0001;
	tx_message->WriteBytes = 0x02;
	uint16_t v = std::floor(16.67*x);
	uint8_t v1 = 0 + (v >> 8);
	uint8_t v2 = 0 + (v & 0x00FF);
  ESP_LOGW(TAG, "Amp setting: 0x%02X 0x%02X", v1, v2);
	tx_message->Data[0] = v1;
	tx_message->Data[1] = v2;
	this->send();
}

void eMH1Modbus::send() {
  // Send Modbus query as ASCII text (modbus-ascii !)
	eMH1MessageT *tx_message = &this->emh1_tx_message;
	char buffer[200];
	uint8_t size = 0;
	size = hexencode_ascii(tx_message->DeviceId, buffer, size);
	size = hexencode_ascii(tx_message->FunctionCode, buffer, size);
	size = hexencode_ascii(tx_message->Destination, buffer, size);
	size = hexencode_ascii(tx_message->DataLength, buffer, size);

	if (tx_message->FunctionCode == 0x03) {
		tx_message->LRC = lrc(buffer, size);
	  size = hexencode_ascii(tx_message->LRC, buffer, size);
	} else {
	  size = hexencode_ascii(tx_message->WriteBytes, buffer, size);
	  size = hexencode_ascii(tx_message->Data, buffer, size, 2);
		tx_message->LRC = lrc(buffer, size);
	  size = hexencode_ascii(tx_message->LRC, buffer, size);
  }
  ESP_LOGD(TAG, "TX -> :%s", buffer);
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);
	this->write(':');
  this->write_array((const uint8_t *)buffer, size);
	this->write(0x0D);
	this->write(0x0A);
  this->flush();
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

}  // namespace emh1_modbus
}  // namespace esphome

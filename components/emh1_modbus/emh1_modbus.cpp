#include "emh1_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome.h"

#define BROADCAST_ADDRESS 0xFF

namespace esphome {
namespace emh1_modbus {

static const char *const TAG = "emh1_modbus";

void eMH1Modbus::setup() {
  // if (this->flow_control_pin_ != nullptr) {
  //   this->flow_control_pin_->setup();
  // }
  // ESP_LOGD(TAG, "Flow control pin setup");
	pinMode(5, OUTPUT);
	digitalWrite(5, LOW);
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

std::string hexencode_plain(const uint8_t *data, uint32_t len) {
  char buf[20];
  std::string res;
  for (size_t i = 0; i < len; i++) {
    sprintf(buf, "%02X", data[i]);
    res += buf;
  }
  return res;
}

bool eMH1Modbus::parse_emh1_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *frame = &this->rx_buffer_[0];

  // Byte 0: modbus address (match all)
  if (at == 0)
    return true;

  // Byte 1: Function (msb indicates error)
  if (at == 1)
    return (byte & 0x80) != 0x80;

  if (at == 2)
    return true;

  // Byte 3: eMH1 device address
  if (at == 3)
    return true;
  uint8_t address = frame[3];

  // Byte 9: data length
  if (at < 9)
    return true;

  uint8_t data_len = frame[8];
  // Byte 9...9+data_len-1: Data
  if (at < 9 + data_len)
    return true;

  // Byte 9+data_len: CRC_LO (over all bytes)
  if (at == 9 + data_len)
    return true;

  ESP_LOGVV(TAG, "RX <- %s", format_hex_pretty(frame, at + 1).c_str());

  if (frame[0] != 0xAA || frame[1] != 0x55) {
    ESP_LOGW(TAG, "Invalid header");
    return false;
  }

  // Byte 9+data_len+1: CRC_HI (over all bytes)
  uint16_t computed_checksum = 0; // removed checksum routine!
  uint16_t remote_checksum = uint16_t(frame[9 + data_len + 1]) | (uint16_t(frame[9 + data_len]) << 8);
  if (computed_checksum != remote_checksum) {
    ESP_LOGW(TAG, "Invalid checksum! 0x%02X !=  0x%02X", computed_checksum, remote_checksum);
    return false;
  }

  // data only
  std::vector<uint8_t> data(this->rx_buffer_.begin() + 9, this->rx_buffer_.begin() + 9 + data_len);

  if (address == BROADCAST_ADDRESS) {
    // check control code && function code
    if (frame[6] == 0x10 && frame[7] == 0x80 && data.size() == 14) {
      // ESP_LOGI(TAG, "Inverter discovered. Serial number: %s", hexencode_plain(&data.front(), data.size()).c_str());
      ESP_LOGI(TAG, "Charger discovered.");
      // this->register_address(0x01);
    } else {
      ESP_LOGW(TAG, "Unknown broadcast data: %s", format_hex_pretty(&data.front(), data.size()).c_str());
    }

    // early return false to reset buffer
    return false;
  }

  bool found = false;
  for (auto *device : this->devices_) {
    if (device->address_ == address) {
      if (frame[6] == 0x11) {
        device->on_emh1_modbus_data(frame[7], data);
      } else {
        ESP_LOGW(TAG, "Unhandled control code (%d) of frame for address 0x%02X: %s", frame[6], address,
                 format_hex_pretty(frame, at + 1).c_str());
      }
      found = true;
    }
  }

  if (!found) {
    ESP_LOGW(TAG, "Got eMH1 frame from unknown device address 0x%02X!", address);
  }

  // return false to reset buffer
  return false;
}

void eMH1Modbus::dump_config() {
  // ESP_LOGCONFIG(TAG, "eMH1Modbus:");
  // LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  // this->check_uart_settings(38400);
}

float eMH1Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

uint8_t char2int8(char value[2]) {
  char c1 = value[0];
  uint8_t highBits = (c1 > '9')?(c1-55):(c1-48);
  char c2 = value[1];
  uint8_t lowBits = (c2 > '9')?(c2-55):(c2-48);
  return (highBits << 4 | lowBits);
}

uint16_t char2int16(char value[4]) {
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

uint8_t lrc(char *value, uint8_t l) {
  uint8_t lrc_ = 0;
  for (int i = 0; i < l-1; i = i + 2) {
    lrc_ -= char2int8(&value[i]);
  }
  return lrc_;
}

void eMH1Modbus::query_status_report(uint8_t address) {
  static eMH1MessageT tx_message;
	tx_message.DeviceId = 0x01;
	tx_message.FunctionCode = 0x03;
	tx_message.Destination = 0x002E;
	tx_message.DataLength = 0x0032;
  this->send(&tx_message);
}

void eMH1Modbus::query_device_info(uint8_t address) {
  static eMH1MessageT tx_message;
	tx_message.DeviceId = 0x01;
	tx_message.FunctionCode = 0x03;
	tx_message.Destination = 0x002C;
	tx_message.DataLength = 0x0001;
  this->send(&tx_message);
}

void eMH1Modbus::query_config_settings(uint8_t address) {
  static eMH1MessageT tx_message;
	tx_message.DeviceId = 0x01;
	tx_message.FunctionCode = 0x03;
	tx_message.Destination = 0x0001;
	tx_message.DataLength = 0x0002;
  this->send(&tx_message);
}

void eMH1Modbus::discover_devices() {
  // broadcast query for serial number
  static eMH1MessageT tx_message;
	tx_message.DeviceId = 0x00;
	tx_message.FunctionCode = 0x03;
	tx_message.Destination = 0x0050;
	tx_message.DataLength = 0x0008;
  this->send(&tx_message);
}

// TODO: kijk of het zonder de bovenste twee int2char definities kan?!

uint8_t int2char(uint8_t val, char* outStr, uint8_t offset) {
  uint8_t highBits = (val & 0xF0) >> 4;
  uint8_t lowBits = (val & 0x0F);
  outStr[offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
  outStr[offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
	return offset+2;
}

uint8_t int2char(uint16_t val, char* outStr, uint8_t offset) {
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

uint8_t int2char(uint8_t* val, char* outStr, uint8_t offset, uint8_t cnt) {
  for (uint8_t x=0; x<cnt; x++) { 
    uint8_t highBits = (val[x] & 0xF0) >> 4;
    uint8_t lowBits = (val[x] & 0x0F);
    outStr[2*x+offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
    outStr[2*x+offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
  }
	return offset+cnt*2;
}

void eMH1Modbus::send(eMH1MessageT *tx_message) {
  ESP_LOGW(TAG, "TX -> Basics");
  // Send Modbus query as ASCII text (modbus-ascii !)
	char buffer[200];
	uint8_t size = 0;
	buffer[size++] = ':';
	size = int2char(tx_message->DeviceId, buffer, size);
	size = int2char(tx_message->FunctionCode, buffer, size);
	size = int2char(tx_message->Destination, buffer, size);
	size = int2char(tx_message->DataLength, buffer, size);
	if (tx_message->FunctionCode == 0x03) {
		tx_message->LRC = lrc(buffer, size);
	  size = int2char(tx_message->LRC, buffer, size);
	} else {
	  // TODO: write moet nog!!!@
		tx_message->LRC = lrc(buffer, size);
	  size = int2char(tx_message->LRC, buffer, size);
  }
	buffer[size++] = 0x0D;
	buffer[size++] = 0x0A;
  ESP_LOGW(TAG, "TX -> %s", buffer);
	digitalWrite(5, HIGH);
  // if (this->flow_control_pin_ != nullptr)
  //  this->flow_control_pin_->digital_write(false);
  this->write_array((const uint8_t *)buffer, size);
  this->flush();
	digitalWrite(5, LOW);
  // if (this->flow_control_pin_ != nullptr)
  //   this->flow_control_pin_->digital_write(false);
}

}  // namespace emh1_modbus
}  // namespace esphome

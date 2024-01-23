#include "emh1_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#define BROADCAST_ADDRESS 0xFF

namespace esphome {
namespace emh1_modbus {

static const char *const TAG = "emh1_modbus";

void eMH1Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
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
      this->register_address(0x01);
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
  ESP_LOGCONFIG(TAG, "eMH1Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);

  this->check_uart_settings(9600);
}

float eMH1Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void eMH1Modbus::query_status_report(uint8_t address) {
  static const char *const query = ":0103002E0032";	
  this->send(&query);
}

void eMH1Modbus::query_device_info(uint8_t address) {
  static const char *const query = ":0103002C0001";
  this->send(&query);
}

void eMH1Modbus::query_config_settings(uint8_t address) {
  static const char *const query = ":010300010002";
  this->send(&query);
}

// void eMH1Modbus::register_address(uint8_t serial_number[14], uint8_t address) {
void eMH1Modbus::register_address(uint8_t address) {
  static eMH1MessageT tx_message;

  tx_message.Source[0] = 0x00;
  tx_message.Source[1] = 0x00;
  tx_message.Destination[0] = 0x00;
  tx_message.Destination[1] = 0x00;
  tx_message.ControlCode = 0x10;
  tx_message.FunctionCode = 0x01;
  tx_message.DataLength = 0x0F;
//  memcpy(tx_message.Data, serial_number, 14);
  tx_message.Data[14] = address;

  this->send(&tx_message);
}

void eMH1Modbus::discover_devices() {
  // broadcast query for serial number
  static const char *const query = ":000300500008";
  this->send(&query);
}

uint8_t Char2Int8(char value[2]) {
  char c1 = value[0];
  uint8_t highBits = (c1 > '9')?(c1-55):(c1-48);
  char c2 = value[1];
  uint8_t lowBits = (c2 > '9')?(c2-55):(c2-48);
  return (highBits << 4 | lowBits);
}

uint16_t Char2Int16(char value[4]) {
  uint16_t res = 0;
  char c;
  uint16_t bits;
  for (byte x=0; x<4; x++) {
    c = value[x];
    bits = (c > '9')?(c-55):(c-48);
    res = (res << 4 | bits);
  }
  return res;
}

char Int2Char(char c[2], uint8_t value) {
  uint8_t highBits = (value & 0xF0) >> 4;
  uint8_t lowBits = (value & 0x0F);
  c[0] = (highBits > 0x09)?(highBits+55):(highBits+48);
  c[1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
  return c;
}

uint8_t lrc(char *value, uint8_t l) {
  uint8_t lrc = 0;
  for (int i = 0; i < l-1; i = i + 2) {
    lrc -= Char2Int8(&value[i]);
  }
  return lrc;
}

void eMH1Modbus::send(const char* bytes) {
  // Send Modbus query as ASCII text (modbus-ascii !)
	uint8_t lrc = lrc(bytes, sizeof(bytes));
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);
  this->print(bytes);
	this->print(lrc, HEX);
	this->print("\r\n");
  this->flush();
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

void eMH1Modbus::send(eMH1MessageT *tx_message) {
  uint8_t msg_len;

  tx_message->Header[0] = 0xAA;
  tx_message->Header[1] = 0x55;

  msg_len = tx_message->DataLength + 9;
  auto checksum = 0;

  tx_message->Data[tx_message->DataLength + 0] = checksum >> 8;
  tx_message->Data[tx_message->DataLength + 1] = checksum >> 0;
  msg_len += 2;

  ESP_LOGVV(TAG, "TX -> %s", format_hex_pretty((const uint8_t *) tx_message, msg_len).c_str());

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_array((const uint8_t *) tx_message, msg_len);
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

}  // namespace emh1_modbus
}  // namespace esphome

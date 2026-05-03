#include "emh1_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome.h"

namespace esphome {
namespace emh1_modbus {

static const char *const TAG = "emh1_modbus";

static const uint8_t DEVICE_ID = 0x01; // TODO: make variable?

void eMH1Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
  eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = DEVICE_ID;
  tx_message->FunctionCode = FUNCTION_READ;
  tx_message->Destination = REG_READ_CURRENT_FULL;
  tx_message->DataLength = 5;
  tx_message->LRC = 0x00;
  tx_message->WriteBytes = 0x00;
}

// loop() receives incoming replies from ABL
void eMH1Modbus::loop() {
  const uint32_t now = millis();
  if (now - this->last_emh1_modbus_byte_ > 50) {
    if (!this->rx_buffer_.empty()) {
      ESP_LOGW(TAG, "Transmission interrupted. Discarding buffer.");
      this->rx_buffer_.clear();
    }
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

// convert char[2] hexencoded ascii to a single uint8_t value
uint8_t ascii2uint8(const char *value) {
  char c1 = value[0];
  uint8_t highBits = (c1 > '9') ? (c1 - 55) : (c1 - 48);
  char c2 = value[1];
  uint8_t lowBits = (c2 > '9') ? (c2 - 55) : (c2 - 48);
  return (highBits << 4 | lowBits);
}

// convert char[4] hexencoded ascii to a single uint16_t value
uint16_t ascii2uint16(const char *value) {
  uint16_t res = 0;
  char c;
  uint16_t bits;
  for (uint8_t x = 0; x < 4; x++) {
    c = value[x];
    bits = (c > '9') ? (c - 55) : (c - 48);
    res = (res << 4 | bits);
  }
  return res;
}

// calculate LRC checksum over char[], result is single uint8_t value
uint8_t lrc(const char *value, uint8_t l) {
  uint8_t lrc_ = 0;
  for (int i = 0; i < l - 1; i = i + 2) {
    lrc_ -= ascii2uint8(&value[i]);
  }
  return lrc_;
}

// parse_emh1_modbus_byte will read a byte from RS485
// if it's the last byte of a transmission, it will
// proceed to parse the buffer, trying to make sense
// of the received package
//
bool eMH1Modbus::parse_emh1_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  if (byte != '\n') // LF == End of transmission
    return true;
  this->rx_buffer_.push_back('\0');
  char *frame = &this->rx_buffer_[0];
  eMH1MessageT *tx_message = &this->emh1_tx_message;

  // check LRC
  uint8_t lrc1 = ascii2uint8(&frame[at - 3]);
  uint8_t lrc2 = lrc(&frame[1], at - 3);
  if (lrc1 != lrc2) {
    ESP_LOGW(TAG, "LRC check failed, discarding transmission");
    return false;
  }

  // check contents of first byte
  switch (frame[0]) {
  case ':':
    ESP_LOGD(TAG, "Ignore Master transmission: %s", frame);
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
  if (r != DEVICE_ID) {
    ESP_LOGW(TAG, "ERROR: Received from wrong device ID: 0x%02X", r);
    return false;
  }

  // Check Function Code
  r = ascii2uint8(&frame[3]);
  uint16_t v;
  switch (r) {
  case FUNCTION_READ:
    // ESP_LOGD(TAG, "Response to read operation");
    r = ascii2uint8(&frame[5]);
    ESP_LOGD(TAG, "Receiving %u bytes", r);
    if (r == tx_message->DataLength * 2) {
      // ESP_LOGD(TAG, "Send data upwards");
      for (uint8_t x = 0; x < r; x++) {
        tx_message->Data[x] = ascii2uint8(&frame[7 + x * 2]);
      }
      bool found = false;
      // TODO: if we check the deviceId here, we should also use address_ for sending
      for (auto *device : this->devices_) {
        if (device->address_ == tx_message->DeviceId) {
          device->on_emh1_modbus_data(tx_message->Destination, tx_message->DataLength, tx_message->Data);
          found = true;
        }
      }
      if (!found) {
        ESP_LOGW(TAG, "Got eMH1 frame from unknown device address");
      }
    } else {
      ESP_LOGW(TAG, "Response data size mismatch, expected %u got %u bytes", this->emh1_tx_message.DataLength * 2, r);
    }
    break;
  case FUNCTION_WRITE:
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
  tx_message->DeviceId = DEVICE_ID;
  tx_message->FunctionCode = FUNCTION_READ;
  tx_message->Destination = REG_READ_CURRENT_FULL;
  tx_message->DataLength = 5;
  this->send();
}

void eMH1Modbus::get_serial() {
  ESP_LOGW(TAG, "Query Serial Number");
  eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = DEVICE_ID;
  tx_message->FunctionCode = FUNCTION_READ  ;
  tx_message->Destination = REG_READ_SERIAL_NUMBER;
  tx_message->DataLength = 8;
  this->send();
}

void eMH1Modbus::get_charging_allowed() {
  ESP_LOGW(TAG, "Get Enable Status");
  eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = DEVICE_ID;
  tx_message->FunctionCode = FUNCTION_READ;
  tx_message->Destination = REG_READ_CHARGING_ALLOWED;
  tx_message->DataLength = 5;
  this->send();
}

void eMH1Modbus::send_charging_disable() {
  ESP_LOGW(TAG, "Get Enable Status");
  send_duty_cycle(NO_CURRENT_ALLOWED);
}

// set Max current
void eMH1Modbus::send_current(float x) {
  ESP_LOGD(TAG, "Set Max Current to %.1f Amps", x);
  send_duty_cycle(std::floor(x / 0.06));
}

void eMH1Modbus::send_duty_cycle(uint16_t duty_cycle) {
  eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = DEVICE_ID;
  tx_message->FunctionCode = FUNCTION_WRITE;
  tx_message->Destination = REG_SET_I_CMAX;
  tx_message->DataLength = 1;
  tx_message->WriteBytes = 2;
  ESP_LOGD(TAG, "Set Max Current duty cycle to %d/1000 (0x%04X)", duty_cycle, duty_cycle);
  tx_message->Data[0] = (uint8_t) (duty_cycle >> 8);
  tx_message->Data[1] = (uint8_t) (duty_cycle & 0x00FF);
  this->send();
}

// send enable/disable
void eMH1Modbus::send_enable(uint8_t x) {
  eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = DEVICE_ID;
  tx_message->FunctionCode = FUNCTION_WRITE;
  tx_message->Destination = REG_MODIFY_STATE;
  tx_message->DataLength = 1;
  tx_message->WriteBytes = 2;
  if (x == 1) {
    tx_message->Data[0] = 0xA1;
    tx_message->Data[1] = 0xA1;
    ESP_LOGD(TAG, "Enable charger");
  } else {
    tx_message->Data[0] = 0xE0;
    tx_message->Data[1] = 0xE0;
    ESP_LOGD(TAG, "Disable charger");
  }
  this->send();
}

char eMH1Modbus::hexencode_single(uint8_t val) {
  return (val < 0x0A) ? (val + '0') : (val - 0x0A + 'A');
}

// convert single uint8_t value to hex-encoded ascii, append to outStr
uint8_t eMH1Modbus::hexencode_ascii(uint8_t val, char *outStr, uint8_t offset) {
  uint8_t highBits = (val & 0xF0) >> 4;
  uint8_t lowBits = (val & 0x0F);
  outStr[offset] = hexencode_single(highBits);
  outStr[offset + 1] = hexencode_single(lowBits);
  return offset + 2;
}

// convert single uint16_t value to hex-encoded ascii, append to outStr
uint8_t eMH1Modbus::hexencode_ascii(uint16_t val, char *outStr, uint8_t offset) {
  uint8_t bytes[2];
  bytes[0] = (val >> 8) & 0xFF; // high byte
  bytes[1] = val & 0xFF;        // low byte
  return hexencode_ascii(bytes, outStr, offset, 2);
}

// convert array of uint8_t to hex-encoded ascii, append to outStr
uint8_t eMH1Modbus::hexencode_ascii(uint8_t *val, char *outStr, uint8_t offset, uint8_t cnt) {
  for (uint8_t x = 0; x < cnt; x++) {
    uint8_t highBits = (val[x] & 0xF0) >> 4;
    uint8_t lowBits = (val[x] & 0x0F);
    outStr[2 * x + offset] = (highBits > 0x09) ? (highBits + 55) : (highBits + 48);
    outStr[2 * x + offset + 1] = (lowBits > 0x09) ? (lowBits + 55) : (lowBits + 48);
  }
  return offset + cnt * 2;
}

// Send a query or command to ABL eMH1 via RS485
void eMH1Modbus::send() {
  // Send Modbus query as ASCII text (modbus-ascii !)
  eMH1MessageT *tx_message = &this->emh1_tx_message;
  char buffer[200];
  uint8_t size = 0;
  size = hexencode_ascii(tx_message->DeviceId, buffer, size);
  size = hexencode_ascii(tx_message->FunctionCode, buffer, size);
  size = hexencode_ascii(tx_message->Destination, buffer, size);
  size = hexencode_ascii(tx_message->DataLength, buffer, size);

  if (tx_message->FunctionCode == FUNCTION_READ) {
    tx_message->LRC = lrc(buffer, size);
    size = hexencode_ascii(tx_message->LRC, buffer, size);
  } else {
    size = hexencode_ascii(tx_message->WriteBytes, buffer, size);
    size = hexencode_ascii(tx_message->Data, buffer, size, tx_message->WriteBytes);
    tx_message->LRC = lrc(buffer, size);
    size = hexencode_ascii(tx_message->LRC, buffer, size);
  }

  buffer[size] = '\0';
  ESP_LOGD(TAG, "TX -> :%s", buffer);
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write(':');
  this->write_array((const uint8_t *)buffer, size);
  this->write('\r');
  this->write('\n');
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

} // namespace emh1_modbus
} // namespace esphome

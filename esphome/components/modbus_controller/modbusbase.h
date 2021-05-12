// Base class for ModbusController
// this code is an extension of the existing modbus class (../modbus/modbus.h)
// it replaces Modbus and uses the uart directly

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace modbus_controller {

class ModbusBase : public uart::UARTDevice, public PollingComponent {
 public:
  static const bool RX_ENABLE = false;
  static const bool TX_ENABLE = true;
  ModbusBase() = default;

  void setup() override {
    if (this->ctrl_pin_) {
      this->ctrl_pin_->setup();
      this->ctrl_pin_->digital_write(RX_ENABLE);
    }
  }

  void read_uart();
  void dump_config() override;

  void register_device(ModbusBase *device) { this->devices_.push_back(device); }

  float get_setup_priority() const override;

  void send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities,
            uint8_t payload_len = 0, const uint8_t *payload = nullptr);

  void send_raw(const std::vector<uint8_t> &payload);

  /** RX,TX Control pin Ref: https://github.com/greays/esphome/blob/master/esphome/components/rs485/rs485.h */
  void set_ctrl_pin(uint8_t ctrl_pin) {
    static GPIOPin PIN(ctrl_pin, OUTPUT);
    ctrl_pin_ = &PIN;
  }

 protected:
  bool parse_modbus_byte_(uint8_t byte);

  std::vector<uint8_t> rx_buffer_;
  uint32_t last_modbus_byte_{0};
  std::vector<ModbusBase *> devices_;
  GPIOPin *ctrl_pin_{nullptr};

 public:
  void set_address(uint8_t address) { this->address_ = address; }
  virtual void on_modbus_data(const std::vector<uint8_t> &data) = 0;
  // provide a default implementation to avoid breaking existing code
  virtual void on_modbus_error(uint8_t function_code, uint8_t exception_code) {}
  void send(uint8_t function_code, uint16_t start_address, uint16_t num_values, uint8_t payload_len = 0,
            const uint8_t *payload = nullptr) {
    send(this->address_, function_code, start_address, num_values, payload_len, payload);
  }

 protected:
  uint8_t address_;
};

}  // namespace modbus_controller
}  // namespace esphome

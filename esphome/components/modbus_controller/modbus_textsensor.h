#pragma once

#include <stdint.h>
#include "modbuscontroller.h"

namespace esphome {
namespace modbus_controller {

class ModbusTextSensor : public Component, public text_sensor::TextSensor, public SensorItem {
 public:
  ModbusTextSensor(const std::string &name, ModbusFunctionCode register_type, uint16_t address, uint8_t offset,
                   uint8_t register_count, uint16_t response_bytes, bool hex_encode, uint8_t skip_updates)
      : Component(), text_sensor::TextSensor(name) {
    this->register_type = register_type;
    this->start_address = start_address;
    this->offset = offset;

    this->register_type = register_type;
    this->start_address = start_address;
    this->offset = offset;
    this->response_bytes_ = response_bytes;
    this->register_count = register_count;
    this->hex_encode = hex_encode;
    this->skip_updates = skip_updates;
  };

  ModbusTextSensor(const std::string &name) : text_sensor::TextSensor(name) {}
  ModbusTextSensor(ModbusFunctionCode register_type, uint16_t address, uint8_t offset, uint8_t register_count,
                   uint16_t response_bytes, bool hex_encode, uint8_t skip_updates)
      : ModbusTextSensor("", register_type, address, offset, register_count, response_bytes, hex_encode, skip_updates) {
  }
  float parse_and_publish(const std::vector<uint8_t> &data) override;
  virtual void log() override;
  void add_to_controller(ModbusController *master, ModbusFunctionCode register_type, uint16_t start_address,
                         uint8_t offset, uint8_t register_count, uint16_t response_bytes, bool hex_encode,
                         uint8_t skip_updates);
  std::string const &get_sensorname() override { return this->get_name(); };
  void update(){};
  void set_state(bool state) { this->state = state; }
  void set_modbus_parent(ModbusController *parent) { this->parent_ = parent; }
  uint16_t response_bytes_;
  bool hex_encode;

 private:
  ModbusController *parent_{nullptr};
};

}  // namespace modbus_controller
}  // namespace esphome

#pragma once

#include "esphome/components/modbus_controller/modbus_controller.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace modbus_controller {

class ModbusSensor : public Component, public sensor::Sensor, public SensorItem {
 public:
  ModbusSensor(ModbusFunctionCode register_type, uint16_t start_address, uint8_t offset, uint32_t bitmask,
               SensorValueType value_type, uint8_t skip_updates)
      : Component(), sensor::Sensor() {
    this->register_type = register_type;
    this->start_address = start_address;
    this->offset = offset;
    this->bitmask = bitmask;
    this->sensor_value_type = value_type;
    this->skip_updates = skip_updates;
    switch (sensor_value_type) {
      case SensorValueType::BIT:
      case SensorValueType::U_WORD:
      case SensorValueType::S_WORD:
        this->register_count = 1;
        break;
      case SensorValueType::RAW:
      case SensorValueType::U_DWORD:
      case SensorValueType::S_DWORD:
      case SensorValueType::U_DWORD_R:
      case SensorValueType::S_DWORD_R:
      case SensorValueType::FP32:
      case SensorValueType::FP32_R:
        this->register_count = 2;
        break;
      case SensorValueType::U_QWORD:
      case SensorValueType::U_QWORD_R:
      case SensorValueType::S_QWORD:
      case SensorValueType::S_QWORD_R:
        this->register_count = 4;
        break;
    }
  }

  void parse_and_publish(const std::vector<uint8_t> &data) override;

  void dump_config() override;
};

}  // namespace modbus_controller
}  // namespace esphome

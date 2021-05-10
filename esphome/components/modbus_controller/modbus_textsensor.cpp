
#include <sstream>
#include <iomanip>
#include "modbuscontroller.h"
#include "modbus_textsensor.h"

namespace esphome {
namespace modbus_controller {

static const char *TAG = "modbus_textsensor";

// ModbusTextSensor
void ModbusTextSensor::log() { LOG_TEXT_SENSOR(TAG, get_name().c_str(), this); }

void ModbusTextSensor::add_to_controller(ModbusController *master, ModbusFunctionCode register_type,
                                         uint16_t start_address, uint8_t offset, uint8_t register_count,
                                         uint16_t response_bytes, bool hex_encode, uint8_t skip_updates) {
  this->register_type = register_type;
  this->start_address = start_address;
  this->offset = offset;
  this->bitmask = 0xFFFFFFFF;
  this->sensor_value_type = SensorValueType::RAW;
  this->response_bytes_ = response_bytes;
  this->last_value = INT64_MIN;
  this->register_count = register_count;
  this->hex_encode = hex_encode;
  this->skip_updates = skip_updates;
  this->parent_ = master;
  master->add_sensor_item(this);
}

float ModbusTextSensor::parse_and_publish(const std::vector<uint8_t> &data) {
  float result = this->response_bytes_;
  std::ostringstream output;
  uint8_t max_items = this->response_bytes_;
  char buffer[4];
  for (auto b : data) {
    if (this - hex_encode) {
      sprintf(buffer, "%02x", b);
      output << buffer;
    } else {
      output << (char) b;
      if (--max_items == 0) {
        break;
      }
    }
  }
  this->publish_state(output.str());
  return result;
}
// ModbusTextSensor End

}  // namespace modbus_controller
}  // namespace esphome

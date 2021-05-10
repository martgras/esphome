#include "esphome/core/application.h"
#include "modbuscontroller.h"
#include "modbus_binarysensor.h"

namespace esphome {
namespace modbus_controller {

static const char *TAG = "modbus_binarysensor";

// ModbusBinarySensor
void ModbusBinarySensor::add_to_controller(ModbusController *master, ModbusFunctionCode register_type,
                                           uint16_t start_address, uint8_t offset, uint32_t bitmask, bool create_switch,
                                           uint8_t skip_updates) {
  this->register_type = register_type;
  this->start_address = start_address;
  this->offset = offset;
  this->bitmask = bitmask;
  this->sensor_value_type = SensorValueType::BIT;
  if (register_type == ModbusFunctionCode::READ_COILS || register_type == ModbusFunctionCode::READ_DISCRETE_INPUTS)
    this->register_count = offset + 1;
  else
    this->register_count = 1;

  this->skip_updates = skip_updates;

  // If this is coil with read/write we can created a switch item on the fly
  // if create_switch is true then the binary_sensor will be changed to internal and a switch with the same name is
  // created when the binary_sensor value is updated the change will be synced to the switch item and vice versa
  if (create_switch && (register_type == ModbusFunctionCode::READ_COILS)) {
    auto new_switch = make_unique<ModbusSwitch>(ModbusFunctionCode::WRITE_SINGLE_COIL, start_address, offset, bitmask);
    this->set_internal(true);  // Make the BinarySensor internal and present a switch instead
    App.register_component(new_switch.get());
    App.register_switch(new_switch.get());
    new_switch->set_name(this->get_name());
    new_switch->start_address = this->start_address;
    new_switch->offset = this->offset;
    new_switch->bitmask = this->bitmask;
    new_switch->set_modbus_parent(master);
    new_switch->set_connected_sensor(this);
#ifdef USE_MQTT
    auto mqtt_sw = make_unique<mqtt::MQTTSwitchComponent>(new_switch.get());
    App.register_component(mqtt_sw.get());
    this->mqtt_switch = std::move(mqtt_sw);
#endif
    this->modbus_switch = std::move(new_switch);
  }
  this->parent_ = master;
  master->add_sensor_item(this);
}

void ModbusBinarySensor::log() { LOG_BINARY_SENSOR(TAG, get_name().c_str(), this); }

float ModbusBinarySensor::parse_and_publish(const std::vector<uint8_t> &data) {
  int64_t value = 0;
  float result = NAN;
  switch (this->register_type) {
    case ModbusFunctionCode::READ_DISCRETE_INPUTS:
      value = coil_from_vector(this->offset, data);
      break;
    case ModbusFunctionCode::READ_COILS:
      // offset for coil is the actual number of the coil not the byte offset
      value = coil_from_vector(this->offset, data);
      break;
    default:
      value = get_data<uint16_t>(data, this->offset) & this->bitmask;
      break;
  }

  result = float(value);
  this->publish_state(value != 0.0);
  // Update the state of the connected switch
  if (this->modbus_switch != nullptr) {
    this->modbus_switch.get()->publish_state(value);
  }
  return result;
}
// ModbusBinarySensor End

}  // namespace modbus_controller
}  // namespace esphome

/*


*/

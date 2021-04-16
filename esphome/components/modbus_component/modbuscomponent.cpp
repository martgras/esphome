
#include <stdint.h>
#include "esphome/core/log.h"
#include <sstream>
#include <iomanip>

#include "modbuscomponent.h"
#define TAG MODBUS_TAG

namespace esphome {
namespace modbus_component {

std::string get_hex_string(const std::vector<uint8_t> &data) {
  std::ostringstream output;
  char buffer[3];
  for (uint8_t b : data) {
    //    sprintf(buffer, "%02x", b);
    uint8_t val = (b & 0xF0) >> 4;
    buffer[0] = val > 9 ? 'a' + val : '0' + val;
    val = (b & 0xF);
    buffer[1] = val > 9 ? 'a' + val : '0' + val;
    buffer[2] = '\0';
    output << buffer;
  }
  return output.str();
}

void ModbusComponent::setup() { this->create_register_ranges(); }

/*
 To work with the existing modbus class and avoid polling for responses a command queue is used.
 send_next_command will submit the command at the top of the queue and set the corresponding callback
 to handle the response from the device.
 Once the response has been processed it is removed from the queue and the next command is sent
*/
bool ModbusComponent::send_next_command_() {
  uint32_t command_delay = millis() - this->last_command_timestamp_;

  if (!sending_ && (command_delay > this->command_throttle_) && (!command_queue_.empty())) {
    this->sending_ = true;
    auto &command = command_queue_.front();
    ESP_LOGD(MODBUS_TAG, "Sending next modbus command %u %u", command_delay, this->command_throttle_);
    command->send();
    this->last_command_timestamp_ = millis();
    if (!command->on_data_func) {  // No handler remove from queue directly after sending
      command_queue_.pop_front();
      this->sending_ = false;
    }
  } else {
    yield();
  }
  return (!command_queue_.empty());
}

// Dispatch the response to the registered handler
void ModbusComponent::on_modbus_data(const std::vector<uint8_t> &data) {
  ESP_LOGD(MODBUS_TAG, "Modbus data %zu", data.size());

  auto &current_command = this->command_queue_.front();
  if (current_command != nullptr) {
    ESP_LOGD(MODBUS_TAG, "Dispatch to handler 0x%X", current_command->register_address);
    current_command->on_data_func(current_command->function_code, current_command->register_address, data);
    this->sending_ = false;
    command_queue_.pop_front();
  }
}
void ModbusComponent::on_modbus_error(uint8_t function_code, uint8_t exception_code) {
  ESP_LOGE(MODBUS_TAG, "Modbus error function code: 0x%X exception: %d ", function_code, exception_code);
  this->sending_ = false;
  // Remove pending command waiting for a response
  auto &current_command = this->command_queue_.front();
  if (current_command != nullptr) {
    ESP_LOGE(MODBUS_TAG,
             "Modbus error - last command: expected response size=%d function code=0x%X  register adddress = 0x%X  "
             "registers count=%d "
             "payload size=%zu",
             current_command->expected_response_size, function_code, current_command->register_address,
             current_command->register_count, current_command->payload.size());
    command_queue_.pop_front();
  }
}

void ModbusComponent::on_register_data(ModbusFunctionCode function_code, uint16_t start_address,
                                       const std::vector<uint8_t> &data) {
  ESP_LOGD(MODBUS_TAG, "data for register address : 0x%X : ", start_address);

  auto vec_it = find_if(begin(register_ranges_), end(register_ranges_), [=](RegisterRange const &r) {
    return (r.start_address == start_address && r.register_type == function_code);
  });

  if (vec_it == register_ranges_.end()) {
    ESP_LOGE(MODBUS_TAG, "Handle incoming data : No matching range for sensor found - start_address :  0x%X",
             start_address);
    return;
  }
  auto map_it = sensormap.find(vec_it->first_sensorkey);
  if (map_it == sensormap.end()) {
    ESP_LOGE(MODBUS_TAG, "Handle incoming data : No sensor found in at start_address :  0x%X", start_address);
    return;
  }
  // loop through all sensors with the same start address
  while (map_it != sensormap.end() && map_it->second->start_address == start_address) {
    if (map_it->second->register_type == function_code) {
      RawData r;
      r.raw = data;
      r.modbus_ = this;
      map_it->second->raw_data_callback_.call(r);
      float val = map_it->second->parse_and_publish(data);
      ESP_LOGV(MODBUS_TAG, "Sensor : %s = %.02f ", map_it->second->get_name().c_str(), val);
    }
    map_it++;
  }
}

//
// Queue the modbus requests to be send.
// Once we get a response to the command it is removed from the queue and the next command is send
//
void ModbusComponent::update() {
  static int UPDATE_CNT;

  if (UPDATE_CNT++ == 0) {  // Not sure what happens here. Can't connect via Wifi without skipping the first update.
                            // Maybe interfering with dump_config ?
    ESP_LOGD(MODBUS_TAG, "Skipping update");
    return;
  }

  if (!command_queue_.empty()) {
    ESP_LOGW(MODBUS_TAG, "%d modbus commands already in queue", command_queue_.size());
  } else {
    ESP_LOGI(MODBUS_TAG, "updating modbus component");
  }

  for (auto &r : this->register_ranges_) {
    ESP_LOGD(MODBUS_TAG, "Range : %X Size: %x (%d) skip: %d", r.start_address, r.register_count, (int) r.register_type,
             r.skip_updates_counter);
    if (r.skip_updates_counter == 0) {
      ModbusCommandItem command_item =
          ModbusCommandItem::create_read_command(this, r.register_type, r.start_address, r.register_count);
      queue_command(command_item);
      yield();
      r.skip_updates_counter = r.skip_updates;  // reset counter to config value
    } else {
      r.skip_updates_counter--;
    }
  }
  ESP_LOGI(MODBUS_TAG, "Modbus update complete Free Heap  %u bytes", ESP.getFreeHeap());
}

// walk through the sensors and determine the registerranges to read
size_t ModbusComponent::create_register_ranges() {
  register_ranges_.clear();
  uint8_t n = 0;
  // map is already sorted by keys so we start with the lowest address ;
  auto ix = sensormap.begin();
  auto prev = ix;
  uint16_t current_start_address = ix->second->start_address;
  uint8_t buffer_offset = ix->second->offset;
  uint8_t buffer_gap = 0;
  uint8_t skip_updates = ix->second->skip_updates;
  auto first_sensorkey = ix->second->getkey();
  int total_register_count = 0;
  while (ix != sensormap.end()) {
    size_t diff = ix->second->start_address - prev->second->start_address;
    // use the lowest non zero value for the whole range
    // Because zero is the default value for skip_updates it is excluded from getting the min value.

    ESP_LOGV(MODBUS_TAG, "Register:: 0x%X %d %d  0x%llx (%d) buffer_offset = %d (0x%X) skip=%u",
             ix->second->start_address, ix->second->register_count, ix->second->offset, ix->second->getkey(),
             total_register_count, buffer_offset, buffer_offset, ix->second->skip_updates);
    if (current_start_address != ix->second->start_address ||
        //  ( prev->second->start_address + prev->second->offset != ix->second->start_address) ||
        ix->second->register_type != prev->second->register_type) {
      // Difference doesn't match so we have a gap
      if (n > 0) {
        RegisterRange r;
        r.start_address = current_start_address;
        r.register_count = total_register_count;
        if (prev->second->register_type == ModbusFunctionCode::READ_COILS ||
            prev->second->register_type == ModbusFunctionCode::READ_DISCRETE_INPUTS) {
          r.register_count = prev->second->offset + 1;
        }
        r.register_type = prev->second->register_type;
        r.first_sensorkey = first_sensorkey;
        r.skip_updates = skip_updates;
        r.skip_updates_counter = 0;
        ESP_LOGD(MODBUS_TAG, "Add range 0x%X %d skip:%d", r.start_address, r.register_count, r.skip_updates);
        register_ranges_.push_back(r);
      }
      skip_updates = ix->second->skip_updates;
      current_start_address = ix->second->start_address;
      first_sensorkey = ix->second->getkey();
      total_register_count = ix->second->register_count;
      buffer_offset = ix->second->offset;
      buffer_gap = ix->second->offset;
      n = 1;
    } else {
      n++;
      if (ix->second->offset != prev->second->offset || n == 1) {
        total_register_count += ix->second->register_count;
        buffer_offset += ix->second->get_register_size();
      }
      // Use the lowest non zero skip_upates value for the range
      if (ix->second->skip_updates != 0) {
        if (skip_updates != 0) {
          skip_updates = std::min(skip_updates, ix->second->skip_updates);
        } else {
          skip_updates = ix->second->skip_updates;
        }
      }
    }
    prev = ix++;
  }
  // Add the last range
  if (n > 0) {
    RegisterRange r;
    r.start_address = current_start_address;
    //    r.register_count = prev->second->offset>>1 + prev->second->get_register_size();
    r.register_count = total_register_count;
    if (prev->second->register_type == ModbusFunctionCode::READ_COILS ||
        prev->second->register_type == ModbusFunctionCode::READ_DISCRETE_INPUTS) {
      r.register_count = prev->second->offset + 1;
    }
    r.register_type = prev->second->register_type;
    r.first_sensorkey = first_sensorkey;
    r.skip_updates = skip_updates;
    r.skip_updates_counter = 0;
    register_ranges_.push_back(r);
  }
  return register_ranges_.size();
}

void ModbusComponent::dump_config() {
  ESP_LOGCONFIG(MODBUS_TAG, "EPSOLAR:");
  ESP_LOGCONFIG(MODBUS_TAG, "  Address: 0x%02X", this->address_);
  for (auto &item : this->sensormap) {
    item.second->log();
  }
  create_register_ranges();
}

void ModbusComponent::loop() { send_next_command_(); }

// Extract data from modbus response buffer
template<typename T> T get_data(const std::vector<uint8_t> &data, size_t offset) {
  if (sizeof(T) == sizeof(uint8_t)) {
    return T(data[offset]);
  }
  if (sizeof(T) == sizeof(uint16_t)) {
    return T((uint16_t(data[offset + 0]) << 8) | (uint16_t(data[offset + 1]) << 0));
  }

  if (sizeof(T) == sizeof(uint32_t)) {
    return get_data<uint16_t>(data, offset) << 16 | get_data<uint16_t>(data, (offset + 2));
  }

  if (sizeof(T) == sizeof(uint64_t)) {
    return static_cast<uint64_t>(get_data<uint32_t>(data, offset)) << 32 |
           (static_cast<uint64_t>(get_data<uint32_t>(data, offset + 4)));
  }
}

void ModbusComponent::on_write_register_response(ModbusFunctionCode function_code, uint16_t start_address,
                                                 const std::vector<uint8_t> &data) {
  ESP_LOGD(MODBUS_TAG, "Command ACK 0x%X %d ", get_data<uint16_t>(data, 0), get_data<int16_t>(data, 1));
}

std::atomic_bool ModbusComponent::sending_(false);

void FloatSensorItem::log() { LOG_SENSOR("", sensor_->get_name().c_str(), this->sensor_); }

void BinarySensorItem::log() { LOG_BINARY_SENSOR("", sensor_->get_name().c_str(), this->sensor_); }

void TextSensorItem::log() { LOG_TEXT_SENSOR("", sensor_->get_name().c_str(), this->sensor_); }

void ModbusSwitchItem::log() { LOG_SWITCH("", modbus_switch_->get_name().c_str(), this->modbus_switch_); }

// void ModbusSwitchItem::log() { LOG_SWITCH("", sensor_->get_name().c_str(), this->sensor_); }

// Extract bits from value and shift right according to the bitmask
// if the bitmask is 0x00F0  we want the values frrom bit 5 - 8.
// the result is then shifted right by the postion if the first right set bit in the mask
// Usefull for modbus data where more than one value is packed in a 16 bit register
// Example: on Epever the "Length of night" register 0x9065 encodes values of the whole night length of time as
// D15 - D8 =  hour, D7 - D0 = minute
// To get the hours use mask 0xFF00 and  0x00FF for the minute
template<typename N> N mask_and_shift_by_rightbit(N data, uint32_t mask) {
  auto result = (mask & data);
  if (result == 0) {
    return result;
  }
  for (int pos = 0; pos < sizeof(N) << 3; pos++) {
    if ((mask & (1 << pos)) != 0)
      return result >> pos;
  }
  return 0;
}

float FloatSensorItem::parse_and_publish(const std::vector<uint8_t> &data) {
  int64_t value = 0;  // int64_t because it can hold signed and unsigned 32 bits
  float result = NAN;

  switch (sensor_value_type) {
    case SensorValueType::U_WORD:
      value = mask_and_shift_by_rightbit(get_data<uint16_t>(data, this->offset), this->bitmask);  // default is 0xFFFF ;
      break;
    case SensorValueType::U_DWORD:
      value = get_data<uint32_t>(data, this->offset);
      value = mask_and_shift_by_rightbit((uint32_t) value, this->bitmask);
      break;
    case SensorValueType::U_DWORD_R:
      value = get_data<uint32_t>(data, this->offset);
      value = static_cast<uint32_t>(value & 0xFFFF) << 16 | (value & 0xFFFF0000) >> 16;
      value = mask_and_shift_by_rightbit((uint32_t) value, this->bitmask);
      break;
    case SensorValueType::S_WORD:
      value = mask_and_shift_by_rightbit(get_data<int16_t>(data, this->offset),
                                         this->bitmask);  // default is 0xFFFF ;
      break;
    case SensorValueType::S_DWORD:
      value = mask_and_shift_by_rightbit(get_data<int32_t>(data, this->offset), this->bitmask);
      break;
    case SensorValueType::S_DWORD_R: {
      value = get_data<uint32_t>(data, this->offset);
      // Currently the high word is at the low position
      // the sign bit is therefore at low before the switch
      uint32_t sign_bit = (value & 0x8000) << 16;
      value = mask_and_shift_by_rightbit(
          static_cast<int32_t>(((value & 0x7FFF) << 16 | (value & 0xFFFF0000) >> 16) | sign_bit), this->bitmask);
    } break;
    case SensorValueType::U_QWORD:
      // Ignore bitmask for U_QWORD
      value = get_data<uint64_t>(data, this->offset);
      break;

    case SensorValueType::S_QWORD:
      // Ignore bitmask for S_QWORD
      value = get_data<int64_t>(data, this->offset);
      break;
    case SensorValueType::U_QWORD_R:
      // Ignore bitmask for U_QWORD
      value = get_data<uint64_t>(data, this->offset);
      value = static_cast<uint64_t>(value & 0xFFFF) << 48 | (value & 0xFFFF000000000000) >> 48 |
              static_cast<uint64_t>(value & 0xFFFF0000) << 32 | (value & 0x0000FFFF00000000) >> 32 |
              static_cast<uint64_t>(value & 0xFFFF00000000) << 16 | (value & 0x00000000FFFF0000) >> 16;
      break;

    case SensorValueType::S_QWORD_R:
      // Ignore bitmask for S_QWORD
      value = get_data<int64_t>(data, this->offset);
      break;
    default:
      break;
  }
  result = float(value);

  // No need to publish if the value didn't change since the last publish
  // can reduce mqtt traffic considerably if many sensors are used
  ESP_LOGVV(MODBUS_TAG, " SENSOR : new: %lld  old: %lld ", value, this->last_value);
  if (value != this->last_value || this->sensor_->get_force_update()) {
    // this->sensor_->raw_state = result;
    this->sensor_->publish_state(result);
    this->last_value = value;
  } else {
    ESP_LOGV(MODBUS_TAG, "SENSOR %s value didn't change - don't publish", get_name().c_str());
  }
  return result;
}

inline bool coil_from_vector(int coil, const std::vector<uint8_t> &data) {
  auto data_byte = coil / 8;
  return (data[data_byte] & (1 << (coil % 8))) > 0;
}

float BinarySensorItem::parse_and_publish(const std::vector<uint8_t> &data) {
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
  // No need tp publish if the value didn't change since the last publish
  if (value != this->last_value) {
    this->sensor_->publish_state(value != 0.0);
    this->last_value = value;
    // Update the state of the connected switch
    if (this->modbus_switch != nullptr) {
      this->modbus_switch.get()->publish_state(value);
    }
  }
  return result;
}

float TextSensorItem::parse_and_publish(const std::vector<uint8_t> &data) {
  int64_t value = 0;
  float result = this->response_bytes_;
  std::ostringstream output;
  uint8_t max_items = this->response_bytes_;
  char buffer[4];
  for (auto b : data) {
    if (this->hex_encode) {
      sprintf(buffer, "%02x", b);
      output << buffer;
    } else {
      output << (char) b;
      if (--max_items == 0) {
        break;
      }
    }
  }
  this->sensor_->publish_state(output.str());
  return result;
}

float ModbusSwitchItem::parse_and_publish(const std::vector<uint8_t> &data) {
  bool value = (data[0] != 0);
  switch (this->register_type) {
    case ModbusFunctionCode::READ_DISCRETE_INPUTS:
      //      value = data[this->offset] & 1;  // Discret Input is always just one bit
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

  return value;
}

// factory methods
ModbusCommandItem ModbusCommandItem::create_read_command(
    ModbusComponent *modbusdevice, ModbusFunctionCode function_code, uint16_t start_address, uint16_t register_count,
    std::function<void(ModbusFunctionCode function_code, uint16_t start_address, const std::vector<uint8_t> &data)>
        &&handler) {
  ModbusCommandItem cmd;
  cmd.modbusdevice = modbusdevice;
  cmd.function_code = function_code;
  cmd.register_address = start_address;
  cmd.expected_response_size = register_count * 2;
  cmd.register_count = register_count;
  cmd.on_data_func = std::move(handler);
  // adjust expected response size for ReadCoils and DiscretInput
  if (cmd.function_code == ModbusFunctionCode::READ_COILS ||
      cmd.function_code == ModbusFunctionCode::READ_DISCRETE_INPUTS) {
    cmd.expected_response_size = (register_count + 7) / 8;
  }
  /*
  if (cmd.function_code == ModbusFunctionCode::READ_DISCRETE_INPUTS) {
    cmd.expected_response_size = 1;
  }
  */
  return cmd;
}

ModbusCommandItem ModbusCommandItem::create_read_command(ModbusComponent *modbusdevice,
                                                         ModbusFunctionCode function_code, uint16_t start_address,
                                                         uint16_t register_count) {
  ModbusCommandItem cmd;
  cmd.modbusdevice = modbusdevice;
  cmd.function_code = function_code;
  cmd.register_address = start_address;
  cmd.expected_response_size = register_count * 2;
  cmd.register_count = register_count;
  cmd.on_data_func = [modbusdevice](ModbusFunctionCode function_code, uint16_t start_address,
                                    const std::vector<uint8_t> data) {
    modbusdevice->on_register_data(function_code, start_address, data);
  };
  // adjust expected response size for ReadCoils and DiscretInput
  if (cmd.function_code == ModbusFunctionCode::READ_COILS) {
    cmd.expected_response_size = (register_count + 7) / 8;
  }
  if (cmd.function_code == ModbusFunctionCode::READ_DISCRETE_INPUTS) {
    // cmd.expected_response_size = 1;
    cmd.expected_response_size = (register_count + 7) / 8;
  }
  return cmd;
}

ModbusCommandItem ModbusCommandItem::create_write_multiple_command(ModbusComponent *modbusdevice,
                                                                   uint16_t start_address, uint16_t register_count,
                                                                   const std::vector<uint16_t> &values) {
  ModbusCommandItem cmd;
  cmd.modbusdevice = modbusdevice;
  cmd.function_code = ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS;
  cmd.register_address = start_address;
  cmd.expected_response_size = 4;  // Response to write commands is always 4 bytes
  cmd.register_count = register_count;
  cmd.on_data_func = [modbusdevice, cmd](ModbusFunctionCode function_code, uint16_t start_address,
                                         const std::vector<uint8_t> data) {
    modbusdevice->on_write_register_response(cmd.function_code, start_address, data);
  };
  for (auto v : values) {
    cmd.payload.push_back((v / 256) & 0xFF);
    cmd.payload.push_back(v & 0xFF);
  }
  return cmd;
}

ModbusCommandItem ModbusCommandItem::create_write_single_coil(ModbusComponent *modbusdevice, uint16_t address,
                                                              bool value) {
  ModbusCommandItem cmd;
  cmd.modbusdevice = modbusdevice;
  cmd.function_code = ModbusFunctionCode::WRITE_SINGLE_COIL;
  cmd.register_address = address;
  cmd.expected_response_size = 4;  // Response to write commands is always 4 bytes
  cmd.register_count = 1;
  cmd.on_data_func = [modbusdevice, cmd](ModbusFunctionCode function_code, uint16_t start_address,
                                         const std::vector<uint8_t> data) {
    modbusdevice->on_write_register_response(cmd.function_code, start_address, data);
  };
  cmd.payload.push_back(value ? 0xFF : 0);
  cmd.payload.push_back(0);
  return cmd;
}

ModbusCommandItem ModbusCommandItem::create_write_multiple_coils(ModbusComponent *modbusdevice, uint16_t start_address,
                                                                 const std::vector<bool> &values) {
  ModbusCommandItem cmd;
  cmd.modbusdevice = modbusdevice;
  cmd.function_code = ModbusFunctionCode::WRITE_MULTIPLE_COILS;
  cmd.register_address = start_address;
  cmd.expected_response_size = 4;  // Response to write commands is always 4 bytes
  cmd.register_count = values.size();
  cmd.on_data_func = [modbusdevice, cmd](ModbusFunctionCode function_code, uint16_t start_address,
                                         const std::vector<uint8_t> data) {
    modbusdevice->on_write_register_response(cmd.function_code, start_address, data);
  };

  uint8_t bitmask = 0;
  int bitcounter = 0;
  for (auto coil : values) {
    if (coil) {
      bitmask |= (1 << bitcounter);
    }
    bitcounter++;
    if (bitcounter % 8 == 0) {
      cmd.payload.push_back(bitmask);
      bitmask = 0;
    }
  }
  // add remaining bits
  if (bitcounter % 8) {
    cmd.payload.push_back(bitmask);
  }
  cmd.expected_response_size = 4;
  return cmd;
}

ModbusCommandItem ModbusCommandItem::create_write_single_command(ModbusComponent *modbusdevice, uint16_t start_address,
                                                                 int16_t value) {
  ModbusCommandItem cmd;
  cmd.modbusdevice = modbusdevice;
  cmd.function_code = ModbusFunctionCode::WRITE_SINGLE_REGISTER;
  cmd.register_address = start_address;
  cmd.expected_response_size = 4;  // Response to write commands is always 4 bytes
  cmd.register_count = 1;          // not used here anyways
  cmd.on_data_func = [modbusdevice, cmd](ModbusFunctionCode function_code, uint16_t start_address,
                                         const std::vector<uint8_t> data) {
    modbusdevice->on_write_register_response(cmd.function_code, start_address, data);
  };
  cmd.payload.push_back((value / 256) & 0xFF);
  cmd.payload.push_back((value % 256) & 0xFF);
  return cmd;
}

bool ModbusCommandItem::send() {
  ESP_LOGV(MODBUS_TAG, "Command sent %d 0x%X %d", uint8_t(this->function_code), this->register_address,
           this->register_count);
  modbusdevice->send(uint8_t(this->function_code), this->register_address, this->register_count, this->payload.size(),
                     this->payload.empty() ? nullptr : &this->payload[0]);
  return true;
}

void ModbusSwitch::write_state(bool state) {
  // This will be called every time the user requests a state change.
  if (parent_ == nullptr) {
    // switch not configued correctly
    ESP_LOGE(TAG, "ModbusSwitch: %s : missing parent", this->get_name().c_str());
    return;
  }
  ModbusCommandItem cmd;
  ESP_LOGD(TAG, "write_state for ModbusSwitch '%s': new value = %d  type = %d address = %X offset = %x",
           this->get_name().c_str(), state, (int) this->register_type, this->start_address, this->offset);
  switch (this->register_type) {
    case ModbusFunctionCode::WRITE_SINGLE_COIL:
      cmd = ModbusCommandItem::create_write_single_coil(parent_, this->start_address + this->offset, state);
      break;
    case ModbusFunctionCode::WRITE_SINGLE_REGISTER:
      cmd = ModbusCommandItem::create_write_single_command(parent_, this->start_address,
                                                           state ? 0xFFFF & this->bitmask : 0);
      break;
    default:
      ESP_LOGE(TAG, "Invalid function code for ModbusSwitch: %d", (int) this->register_type);
      return;
      break;
  }
  parent_->queue_command(std::move(cmd));
  publish_state(state);
}

}  // namespace modbus_component
}  // namespace esphome

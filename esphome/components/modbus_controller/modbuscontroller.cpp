#include <stdint.h>
#include <list>
#include <sstream>
#include <iomanip>

#include "esphome/core/log.h"
#include "esphome/core/application.h"
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_component.h"
#include "esphome/components/mqtt/mqtt_switch.h"
#endif
#include "modbuscontroller.h"

static const char *TAG = "ModbusController";

namespace esphome {
namespace modbus_controller {

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

void ModbusController::setup() { this->create_register_ranges(); }

/*
 To work with the existing modbus class and avoid polling for responses a command queue is used.
 send_next_command will submit the command at the top of the queue and set the corresponding callback
 to handle the response from the device.
 Once the response has been processed it is removed from the queue and the next command is sent
*/
bool ModbusController::send_next_command_() {
  uint32_t command_delay = millis() - this->last_command_timestamp_;
  if (!sending_ && (command_delay > this->command_throttle_) && !command_queue_.empty()) {
    this->sending_ = true;
    auto &command = command_queue_.front();
    ESP_LOGD(TAG, "Sending next modbus command %u %u", command_delay, this->command_throttle_);
    command->send();
    delay(2);
    this->last_command_timestamp_ = millis();
    if (!command->on_data_func) {  // No handler remove from queue directly after sending
      command_queue_.pop_front();
      this->sending_ = false;
    }
  }
  return (!command_queue_.empty());
}

// Dispatch the response to the registered handler
void ModbusController::on_modbus_data(const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "Modbus data %zu", data.size());

  auto &current_command = this->command_queue_.front();
  if (current_command != nullptr) {
    ESP_LOGD(TAG, "Dispatch to handler 0x%X", current_command->register_address);
    this->sending_ = false;
    current_command->on_data_func(current_command->function_code, current_command->register_address, data);

    command_queue_.pop_front();
  }
}
void ModbusController::on_modbus_error(uint8_t function_code, uint8_t exception_code) {
  ESP_LOGE(TAG, "Modbus error function code: 0x%X exception: %d ", function_code, exception_code);
  this->sending_ = false;
  // Remove pending command waiting for a response
  auto &current_command = this->command_queue_.front();
  if (current_command != nullptr) {
    ESP_LOGE(TAG,
             "Modbus error - last command: expected response size=%d function code=0x%X  register adddress = 0x%X  "
             "registers count=%d "
             "payload size=%zu",
             current_command->expected_response_size, function_code, current_command->register_address,
             current_command->register_count, current_command->payload.size());
    command_queue_.pop_front();
  }
}

void ModbusController::on_register_data(ModbusFunctionCode function_code, uint16_t start_address,
                                        const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "data for register address : 0x%X : ", start_address);

  auto vec_it = find_if(begin(register_ranges_), end(register_ranges_), [=](RegisterRange const &r) {
    return (r.start_address == start_address && r.register_type == function_code);
  });

  if (vec_it == register_ranges_.end()) {
    ESP_LOGE(TAG, "Handle incoming data : No matching range for sensor found - start_address :  0x%X", start_address);
    return;
  }
  auto map_it = sensormap.find(vec_it->first_sensorkey);
  if (map_it == sensormap.end()) {
    ESP_LOGE(TAG, "Handle incoming data : No sensor found in at start_address :  0x%X", start_address);
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
      ESP_LOGV(TAG, "Sensor : %s = %.02f ", map_it->second->get_sensorname().c_str(), val);
    }
    map_it++;
  }
}

void ModbusController::queue_command(const ModbusCommandItem &command) {
  // check if this commmand is already qeued.
  // not very effective but the queue is never really large
  for (auto &item : command_queue_) {
    if (item->register_address == command.register_address && item->register_count == command.register_count &&
        item->function_code == command.function_code) {
      ESP_LOGW(TAG, "Duplicate modbus command found");
      // update the payload of the queued command
      // replaces a previous command
      item->payload = command.payload;
      return;
    }
  }
  command_queue_.push_back(std::move(make_unique<ModbusCommandItem>(command)));
}

void ModbusController::update_range(RegisterRange &r) {
  ESP_LOGD(TAG, "Range : %X Size: %x (%d) skip: %d", r.start_address, r.register_count, (int) r.register_type,
           r.skip_updates_counter);
  if (r.skip_updates_counter == 0) {
    ModbusCommandItem command_item =
        ModbusCommandItem::create_read_command(this, r.register_type, r.start_address, r.register_count);
    queue_command(command_item);
    r.skip_updates_counter = r.skip_updates;  // reset counter to config value
  } else {
    r.skip_updates_counter--;
  }
}
//
// Queue the modbus requests to be send.
// Once we get a response to the command it is removed from the queue and the next command is send
//
void ModbusController::update() {
  static int UPDATE_CNT;

  if (UPDATE_CNT++ == 0) {  // Not sure what happens here. Can't connect via Wifi without skipping the first update.
                            // Maybe interfering with dump_config ?
    ESP_LOGD(TAG, "Skipping update");
    return;
  }

  if (!command_queue_.empty()) {
    ESP_LOGW(TAG, "%d modbus commands already in queue", command_queue_.size());
  } else {
    ESP_LOGI(TAG, "updating modbus component");
  }

  for (auto &r : this->register_ranges_) {
    update_range(r);
  }
  ESP_LOGI(TAG, "Modbus update complete Free Heap  %u bytes", ESP.getFreeHeap());
}

// walk through the sensors and determine the registerranges to read
size_t ModbusController::create_register_ranges() {
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

    ESP_LOGV(TAG, "Register '%s': 0x%X %d %d  0x%llx (%d) buffer_offset = %d (0x%X) skip=%u",
             ix->second->get_sensorname().c_str(), ix->second->start_address, ix->second->register_count,
             ix->second->offset, ix->second->getkey(), total_register_count, buffer_offset, buffer_offset,
             ix->second->skip_updates);
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
        ESP_LOGD(TAG, "Add range 0x%X %d skip:%d", r.start_address, r.register_count, r.skip_updates);
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

bool ModbusController::remove_register_range(uint16_t start_address) {
  bool found = false;

  for (auto it = this->register_ranges_.begin(); it != this->register_ranges_.end();) {
    if (it->start_address == start_address) {
      // First delete sensors from the map
      auto first_sensor = sensormap.find(it->first_sensorkey);
      if (first_sensor != sensormap.end()) {
        // loop through all sensors with the same start address
        auto last_sensor = first_sensor;
        while (last_sensor != sensormap.end() && last_sensor->second->start_address == start_address) {
          last_sensor++;
        }
        sensormap.erase(first_sensor, last_sensor);
      }
      // Remove the range itself
      it = this->register_ranges_.erase(it);
      found = true;
    } else {
      it++;
    }
  }
  return found;
}

void ModbusController::dump_config() {
  ESP_LOGCONFIG(TAG, "EPSOLAR:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  for (auto &item : this->sensormap) {
    item.second->log();
  }
  create_register_ranges();
}

void ModbusController::loop() { send_next_command_(); }

void ModbusController::on_write_register_response(ModbusFunctionCode function_code, uint16_t start_address,
                                                  const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "Command ACK 0x%X %d ", get_data<uint16_t>(data, 0), get_data<int16_t>(data, 1));
}

std::atomic_bool ModbusController::sending_(false);

// factory methods
ModbusCommandItem ModbusCommandItem::create_read_command(
    ModbusController *modbusdevice, ModbusFunctionCode function_code, uint16_t start_address, uint16_t register_count,
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
  return cmd;
}

ModbusCommandItem ModbusCommandItem::create_read_command(ModbusController *modbusdevice,
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

ModbusCommandItem ModbusCommandItem::create_write_multiple_command(ModbusController *modbusdevice,
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

ModbusCommandItem ModbusCommandItem::create_write_single_coil(ModbusController *modbusdevice, uint16_t address,
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

ModbusCommandItem ModbusCommandItem::create_write_multiple_coils(ModbusController *modbusdevice, uint16_t start_address,
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

ModbusCommandItem ModbusCommandItem::create_write_single_command(ModbusController *modbusdevice, uint16_t start_address,
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
  ESP_LOGV(TAG, "Command sent %d 0x%X %d", uint8_t(this->function_code), this->register_address, this->register_count);
  modbusdevice->send(uint8_t(this->function_code), this->register_address, this->register_count, this->payload.size(),
                     this->payload.empty() ? nullptr : &this->payload[0]);
  return true;
}

}  // namespace modbus_controller
}  // namespace esphome

#pragma once

#include <stdint.h>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/automation.h"
#include "esphome/core/application.h"
#include "esphome/core/application.h"

#include "esphome/components/mqtt/mqtt_component.h"
#include "esphome/components/mqtt/mqtt_switch.h"

#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <vector>
#include <list>
#include <atomic>

namespace esphome {
namespace modbus_component {

static const char *MODBUS_TAG = "ModbusComponent";

using namespace sensor;
struct ModbusCommandItem;
class ModbusComponent;

enum class ModbusFunctionCode {
  READ_COILS = 0x01,
  READ_DISCRETE_INPUTS = 0x02,
  READ_HOLDING_REGISTERS = 0x03,
  READ_INPUT_REGISTERS = 0x04,
  WRITE_SINGLE_COIL = 0x05,
  WRITE_SINGLE_REGISTER = 0x6,
  READ_EXCEPTION_STATUS = 0x7,   // not implemented
  DIAGNOSTICS = 0x8,             // not implemented
  GET_COMM_EVENT_COUNTER = 0xB,  // not implemented
  GET_COMM_EVENT_LOG = 0xC,      // not implemented
  WRITE_MULTIPLE_COILS = 0xF,
  WRITE_MULTIPLE_REGISTERS = 0x10,
  REPORT_SERVER_ID = 0x11,               // not implemented
  READ_FILE_RECORD = 0x14,               // not implemented
  WRITE_FILE_RECORD = 0x15,              // not implemented
  MASK_WRITE_REGISTER = 0x16,            // not implemented
  READ_WRITE_MULTIPLE_REGISTERS = 0x17,  // not implemented
  READ_FIFO_QUEUE = 0x18                 // not implemented

};

enum class SensorValueType : uint8_t {
  RAW = 0x00,      // variable length
  U_WORD = 0x01,   // 1 Register unsigned
  U_DWORD = 0x02,  // 2 Registers unsigned
  S_WORD = 0x03,   // 1 Register signed
  S_DWORD = 0x04,  // 2 Registers signed
  BIT = 0x05,
  U_DWORD_R = 0x06,  // 2 Registers unsigned
  S_DWORD_R = 0x07,  // 2 Registers unsigned
  U_QWORD = 0x8,
  S_QWORD = 0x9,
  U_QWORD_R = 0xA,
  S_QWORD_R = 0xB
};

struct RegisterRange {
  uint16_t start_address;
  uint8_t register_count;
  ModbusFunctionCode register_type;
  uint64_t first_sensorkey;
  uint8_t skip_updates;          // the config value
  uint8_t skip_updates_counter;  // the running value
};

// All sensors are stored in a map
// to enable binary sensors for values encoded as bits in the same register the key of each sensor
// the key is a 64 bit integer that combines the register properties
// Format:  start address (16 bit)| function_code (8 bit) | offset (8bit)| bitmask (32 bit)
inline uint64_t calc_key(ModbusFunctionCode function_code, uint16_t start_address, uint8_t offset = 0,
                         uint32_t bitmask = 0) {
  return uint64_t((uint32_t(start_address) << 16) + (uint16_t(function_code) << 8) + (offset & 0xFF)) << 32 | bitmask;
}
inline uint16_t register_from_key(uint64_t key) { return key >> 48; }

// Get a byte from a hex string
//  hex_byte_from_str("1122",1) returns uint_8 value 0x22 == 34
//  hex_byte_from_str("1122",0) returns 0x11
//  position is the offset in bytes.
//  Because each byte is encoded in 2 hex digits
//  the position of the original byte in the hex string is byte_pos * 2
inline uint8_t byte_from_hex_str(const std::string &value, uint8_t pos) {
  auto c = value.c_str()[pos * 2];
  uint8_t hi = (c >= 'A') ? (c >= 'a') ? (c - 'a' + 10) : (c - 'A' + 10) : (c - '0');
  c = value.c_str()[pos * 2 + 1];
  return ((c >= 'A') ? (c >= 'a') ? (c - 'a' + 10) : (c - 'A' + 10) : (c - '0')) | hi << 4;
}

inline uint16_t word_from_hex_str(const std::string &value, uint8_t pos) {
  return byte_from_hex_str(value, pos) << 8 | byte_from_hex_str(value, pos + 1);
}

inline uint32_t dword_from_hex_str(const std::string &value, uint8_t pos) {
  return word_from_hex_str(value, pos) << 16 | word_from_hex_str(value, pos + 2);
}

inline uint64_t qword_from_hex_str(const std::string &value, uint8_t pos) {
  return static_cast<uint64_t>(dword_from_hex_str(value, pos)) << 32 | dword_from_hex_str(value, pos + 4);
}

std::string get_hex_string(const std::vector<uint8_t> &data);

const std::function<float(int64_t)> DIVIDE_BY_100 = [](int64_t val) { return val / 100.0; };

class ModbusComponent;

class ModbusSwitch : public Component, public switch_::Switch {
 public:
  ModbusSwitch(ModbusFunctionCode register_type, uint16_t address, uint8_t offset, uint32_t bitmask)
      : Component(), switch_::Switch() {
    this->register_type = register_type;
    this->start_address = address;
    this->offset = offset;
    this->bitmask = bitmask;
  };
  void setup() override {
    if (connected_sensor_) {
      this->set_name(connected_sensor_->get_name());
    }
  }
  void write_state(bool state) override;
  void set_state(bool state) { this->state = state; }
  void set_modbus_parent(ModbusComponent *parent) { this->parent_ = parent; }
  void set_connected_sensor(binary_sensor::BinarySensor *connected_sensor) {
    this->connected_sensor_ = connected_sensor;
  }


  ModbusFunctionCode register_type;
  uint16_t start_address;
  uint8_t offset;
  uint32_t bitmask;

 private:
  ModbusComponent *parent_{nullptr};
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
};

struct RawData {
  std::vector<uint8_t> raw;
  ModbusComponent *modbus_;
};

struct SensorItem {
  ModbusFunctionCode register_type;
  uint16_t start_address;
  uint8_t offset;
  uint32_t bitmask;
  uint8_t register_count;
  uint8_t skip_updates;
  SensorValueType sensor_value_type;

  int64_t last_value;

  virtual Nameable *get_sensor() = 0;
  virtual std::string const &get_name() = 0;
  virtual void log() = 0;  // {}
  virtual float parse_and_publish(const std::vector<uint8_t> &data) = 0;

  void add_on_raw_data_received_callback(std::function<void(RawData)> callback) {
    this->raw_data_callback_.add(std::move(callback));
  }

  CallbackManager<void(RawData)> raw_data_callback_;

  uint64_t getkey() const { return calc_key(register_type, start_address, offset, bitmask); }
  size_t get_register_size() {
    size_t size = 0;
    switch (sensor_value_type) {
      case SensorValueType::RAW:
        size = 4;
        break;
      case SensorValueType::U_WORD:
        size = 2;
        break;
      case SensorValueType::U_DWORD:
        size = 4;
        break;
      case SensorValueType::S_WORD:
        size = 2;
        break;
      case SensorValueType::S_DWORD:
        size = 4;
        size = register_count;
        break;
      case SensorValueType::BIT:
        size = 1;
        break;
      case SensorValueType::U_DWORD_R:
        size = 4;
        break;
      case SensorValueType::S_DWORD_R:
        size = 4;
        break;
      case SensorValueType::U_QWORD:
      case SensorValueType::U_QWORD_R:
        size = 8;
        break;
      case SensorValueType::S_QWORD:
        size = 8;
        break;
      default:
        size = 2;
        break;
    }
    return size;
  }
};

struct FloatSensorItem : public SensorItem {
  sensor::Sensor *sensor_;
  Nameable *get_sensor() override { return sensor_; }
  FloatSensorItem(sensor::Sensor *sensor) : sensor_(sensor) {}
  std::string const &get_name() override { return sensor_->get_name(); }
  void log() override;
  float parse_and_publish(const std::vector<uint8_t> &data) override;
};

struct ModbusSwitchItem : public SensorItem {
  ModbusSwitchItem(ModbusSwitch *modbus_switch) : modbus_switch_(modbus_switch) {}
  ~ModbusSwitchItem(){};
  Nameable *get_sensor() override { return modbus_switch_; }
  std::string const &get_name() override { return modbus_switch_->get_name(); }
  void log() override;
  float parse_and_publish(const std::vector<uint8_t> &data) override;
  uint16_t response_bytes_;
  ModbusSwitch *modbus_switch_;
};

struct BinarySensorItem : public SensorItem {
  binary_sensor::BinarySensor *sensor_;
  std::unique_ptr<ModbusSwitch> modbus_switch;
  std::unique_ptr<mqtt::MQTTSwitchComponent> mqtt_switch;
  Nameable *get_sensor() override { return sensor_; }
  BinarySensorItem(binary_sensor::BinarySensor *sensor)
      : sensor_(sensor), modbus_switch(nullptr), mqtt_switch(nullptr) {}
  std::string const &get_name() override { return sensor_->get_name(); }
  void log() override;
  float parse_and_publish(const std::vector<uint8_t> &data) override;
};

struct TextSensorItem : public SensorItem {
  text_sensor::TextSensor *sensor_;
  Nameable *get_sensor() override { return sensor_; }
  TextSensorItem(text_sensor::TextSensor *sensor) : sensor_(sensor) {}
  std::string const &get_name() override { return sensor_->get_name(); }
  void log() override;
  float parse_and_publish(const std::vector<uint8_t> &data) override;
  uint16_t response_bytes_;
  bool hex_encode;
};

struct ModbusCommandItem {
  // keep memory consumption low.  Since all registers are 2 bytes and only write RTC needs to be written in 1 command 8
  // bytes is enough
  static const size_t MAX_PAYLOAD_BYTES = 240;
  ModbusComponent *modbusdevice;
  uint16_t register_address;
  uint16_t register_count;
  uint16_t expected_response_size;
  ModbusFunctionCode function_code;
  std::function<void(ModbusFunctionCode function_code, uint16_t start_address, const std::vector<uint8_t> &data)>
      on_data_func;
  std::vector<uint8_t> payload = {};
  bool send();
  // factory methods
  static ModbusCommandItem create_read_command(
      ModbusComponent *modbusdevice, ModbusFunctionCode function_code, uint16_t start_address, uint16_t register_count,
      std::function<void(ModbusFunctionCode function_code, uint16_t start_address, const std::vector<uint8_t> &data)>
          &&handler);

  static ModbusCommandItem create_read_command(ModbusComponent *modbusdevice, ModbusFunctionCode function_code,
                                               uint16_t start_address, uint16_t register_count);
  static ModbusCommandItem create_write_multiple_command(ModbusComponent *modbusdevice, uint16_t start_address,
                                                         uint16_t register_count, const std::vector<uint16_t> &values);
  static ModbusCommandItem create_write_single_command(ModbusComponent *modbusdevice, uint16_t start_address,
                                                       int16_t value);
  static ModbusCommandItem create_write_single_coil(ModbusComponent *modbusdevice, uint16_t address, bool value);
  static ModbusCommandItem create_write_multiple_coils(ModbusComponent *modbusdevice, uint16_t start_address,
                                                       const std::vector<bool> &values);
};

// class ModbusSensor ;
class ModbusComponent : public PollingComponent, public modbus::ModbusDevice {
 public:
  ModbusComponent(uint16_t throttle = 0) : PollingComponent(), modbus::ModbusDevice(), command_throttle_(throttle){};

  void add_sensor(sensor::Sensor *sensor, ModbusFunctionCode register_type, uint16_t start_address, uint8_t offset,
                  uint32_t bitmask, SensorValueType value_type = SensorValueType::U_WORD, int register_count = 1,
                  uint8_t skip_updates = 0) {
    auto new_item = make_unique<FloatSensorItem>(sensor);
    new_item->register_type = register_type;
    new_item->start_address = start_address;
    new_item->offset = offset;
    new_item->bitmask = bitmask;
    new_item->sensor_value_type = value_type;
    // used to cache prev. value.
    // because values are  only in the int32_t range INT64_MIN is a safe "marker" value
    new_item->last_value = INT64_MIN;
    new_item->register_count = register_count;
    new_item->skip_updates = skip_updates;
    sensormap[new_item->getkey()] = std::move(new_item);
  }
  void add_binarysensor(binary_sensor::BinarySensor *sensor, Application &app, ModbusFunctionCode register_type,
                        uint16_t start_address, uint8_t offset, uint32_t bitmask, bool create_switch = false,
                        uint8_t skip_updates = 0) {
    auto new_item = make_unique<BinarySensorItem>(sensor);
    new_item->register_type = register_type;
    new_item->start_address = start_address;
    new_item->offset = offset;
    new_item->bitmask = bitmask;
    new_item->sensor_value_type = SensorValueType::BIT;
    new_item->last_value = INT64_MIN;
    if (register_type == ModbusFunctionCode::READ_COILS || register_type == ModbusFunctionCode::READ_DISCRETE_INPUTS)
      new_item->register_count = offset + 1;
    else
      new_item->register_count = 1;

    new_item->skip_updates = skip_updates;
    // not sure we need it anymore
    auto key = new_item->getkey();

    // If this is coil with read/write we can created a switch item on the fly
    // if create_switch is true then the binary_sensor will be changed to internal and a switch with the same name is
    // created when the binary_sensor value is updated the change will be synced to the switch item and vice versa
    if (create_switch && (register_type == ModbusFunctionCode::READ_COILS)) {
      auto new_switch = make_unique<ModbusSwitch>(ModbusFunctionCode::READ_COILS, start_address, offset, bitmask);
      new_item->sensor_->set_internal(true);  // Make the BinarySensor internal and present a switch instead
      App.register_component(new_switch.get());
      App.register_switch(new_switch.get());
      auto mqtt_sw = make_unique<mqtt::MQTTSwitchComponent>(new_switch.get());
      new_switch->set_name(new_item->get_name());
      new_switch->start_address = new_item->start_address;
      new_switch->offset = new_item->offset;
      new_switch->bitmask = new_item->bitmask;
      new_switch->set_modbus_parent(this);
      new_switch->set_connected_sensor(new_item->sensor_);
      App.register_component(mqtt_sw.get());
      new_item->mqtt_switch = std::move(mqtt_sw);
      new_item->modbus_switch = std::move(new_switch);
    }
    sensormap[key] = std::move(new_item);
  }

  void add_textsensor(text_sensor::TextSensor *sensor, ModbusFunctionCode register_type, uint16_t start_address,
                      uint8_t offset, uint8_t register_count, uint16_t response_bytes, bool hex_encode,
                      uint8_t skip_updates) {
    auto new_item = make_unique<TextSensorItem>(sensor);
    new_item->register_type = register_type;
    new_item->start_address = start_address;
    new_item->offset = offset;
    new_item->sensor_value_type = SensorValueType::RAW;
    new_item->response_bytes_ = response_bytes;
    new_item->last_value = INT64_MIN;
    new_item->register_count = register_count;
    new_item->hex_encode = hex_encode;
    new_item->skip_updates = skip_updates;
    // not sure we need it anymore
    auto key = new_item->getkey();
    sensormap[key] = std::move(new_item);
  }

  void add_modbus_switch(ModbusSwitch *_switch, ModbusFunctionCode register_type, uint16_t start_address,
                         uint8_t offset, uint32_t bitmask) {
    /*
      Create a binary-sensor with a flag auto_switch . if true automatically create an assoociated switch object for
      this address and makes the sensor internal
      ... or maybe vice versa ?

    */

    auto new_item = make_unique<ModbusSwitchItem>(_switch);
    new_item->register_type = register_type;
    new_item->start_address = start_address;
    new_item->bitmask = bitmask;
    new_item->offset = offset;
    new_item->sensor_value_type = SensorValueType::BIT;
    new_item->last_value = INT64_MIN;
    new_item->register_count = 1;
    new_item->skip_updates = 0;
    new_item->modbus_switch_->set_modbus_parent(this);
  }
  size_t create_register_ranges();

  bool remove_register_range(uint16_t start_address) {
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

  // find a register by it's address regardless of the offset
  SensorItem *find_by_register_address(ModbusFunctionCode function_code, uint16_t register_address) {
    // Not extemly effienct but the number of registers isn't that large
    // and the operation is used only in special cases
    // like changing a property during setup
    for (auto &item : this->sensormap) {
      if (register_address == item.second->start_address + item.second->offset &&
          function_code == item.second->register_type) {
        return item.second.get();
      }
    }
    return nullptr;
  }

  void update() override;
  void setup() override;
  void loop() override;
  void on_modbus_data(const std::vector<uint8_t> &data) override;
  void on_modbus_error(uint8_t function_code, uint8_t exception_code) override;

  void dump_config() override;
  void on_write_register_response(ModbusFunctionCode function_code, uint16_t start_address,
                                  const std::vector<uint8_t> &data);
  void on_register_data(ModbusFunctionCode function_code, uint16_t start_address, const std::vector<uint8_t> &data);
  void set_command_throttle(uint16_t command_throttle) { this->command_throttle_ = command_throttle; }

  void queue_command(const ModbusCommandItem &command) {
    // check if this commmand is already qeued.
    // not very effective but the queue is never really large
    for (auto &item : command_queue_) {
      if (item->register_address == command.register_address && item->register_count == command.register_count &&
          item->function_code == command.function_code) {
        ESP_LOGW(MODBUS_TAG, "Duplicate modbus command found");
        // update the payload of the queued command
        // replaces a previous command
        item->payload = command.payload;
        return;
      }
    }
    command_queue_.push_back(std::move(make_unique<ModbusCommandItem>(command)));
  }

 protected:
  bool send_next_command_();
  // Collection of all sensors for this component
  // see calc_key how the key is contructed
  std::map<uint64_t, std::unique_ptr<SensorItem>> sensormap;
  // Continous range of modbus registers
  std::vector<RegisterRange> register_ranges_;
  // Hold the pending requests to be sent
  std::list<std::unique_ptr<ModbusCommandItem>> command_queue_;
  uint32_t last_command_timestamp_;
  uint16_t command_throttle_;
  static std::atomic_bool sending_;

  friend class RawDataCodeTrigger;
  friend class ModbusSwitch;
};

class RawDataCodeTrigger : public Trigger<RawData> {
 public:
  explicit RawDataCodeTrigger(ModbusComponent *parent, text_sensor::TextSensor *sensor) {
    auto id = sensor->get_object_id_hash();
    for (auto &item : parent->sensormap) {
      // if (id == item.second->get_sensor()->get_object_id_hash())
      {
        // return item.second.get();
        item.second->add_on_raw_data_received_callback([this](RawData data) {
          ESP_LOGI(MODBUS_TAG, "ON RAW");
          this->trigger(data);
        });
        break;
      }
    }
  }
};

}  // namespace modbus_component
}  // namespace esphome

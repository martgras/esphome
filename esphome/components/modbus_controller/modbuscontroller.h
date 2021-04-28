#pragma once

#include <stdint.h>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/automation.h"
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_component.h"
#include "esphome/components/mqtt/mqtt_switch.h"
#endif

#include <string>
#include <map>
#include <memory>
#include <vector>
#include <list>
#include <atomic>

namespace esphome {
namespace modbus_controller {

static const char *MODBUS_TAG = "ModbusController";

class ModbusController;

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
// sensormap is sorted by this key. The key ensures the correct order when creating consequtive ranges
// Format:  function_code (8 bit) | start address (16 bit)| offset (8bit)| bitmask (32 bit)
inline uint64_t calc_key(ModbusFunctionCode function_code, uint16_t start_address, uint8_t offset = 0,
                         uint32_t bitmask = 0) {
  return uint64_t((uint16_t(function_code) << 24) + (uint32_t(start_address) << 8) + (offset & 0xFF)) << 32 | bitmask;
}
inline uint16_t register_from_key(uint64_t key) { return (key >> 40) & 0xFFFF; }

// Get a byte from a hex string
//  hex_byte_from_str("1122",1) returns uint_8 value 0x22 == 34
//  hex_byte_from_str("1122",0) returns 0x11
//  position is the offset in bytes.
//  Because each byte is encoded in 2 hex digits
//  the position of the original byte in the hex string is byte_pos * 2
inline uint8_t c_to_hex(char c) { return (c >= 'A') ? (c >= 'a') ? (c - 'a' + 10) : (c - 'A' + 10) : (c - '0'); }
inline uint8_t byte_from_hex_str(const std::string &value, uint8_t pos) {
  if (value.length() < pos * 2 + 1)
    return 0;
  return (c_to_hex(value[pos * 2]) << 4) | c_to_hex(value[pos * 2 + 1]);
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

class ModbusController;

struct RawData {
  std::vector<uint8_t> raw;
  ModbusController *modbus_;
};

struct SensorItem {
  ModbusFunctionCode register_type;
  uint16_t start_address;
  uint8_t offset;
  uint32_t bitmask;
  uint8_t register_count;
  uint8_t skip_updates;
  SensorValueType sensor_value_type;

  int64_t last_value { INT64_MIN};

  virtual std::string const &get_sensorname() = 0;
  virtual void log() = 0;
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

class ModbusSensor : public Component, public sensor::Sensor, public SensorItem {
 public:
  ModbusSensor(const std::string &name)
      : Component(),
        sensor::Sensor(name){

        };
  ModbusSensor(const std::string &name, ModbusFunctionCode register_type, uint16_t start_address, uint8_t offset,
               uint32_t bitmask, SensorValueType value_type = SensorValueType::U_WORD, int register_count = 1,
               uint8_t skip_updates = 0)
      : Component(), sensor::Sensor(name) {
    this->register_type = register_type;
    this->start_address = start_address;
    this->offset = offset;
    this->bitmask = bitmask, this->sensor_value_type = value_type;
    this->register_count = register_count;
    this->skip_updates = skip_updates;
  }

  void update(){};
  void set_modbus_parent(ModbusController *parent) { this->parent_ = parent; }
  float parse_and_publish(const std::vector<uint8_t> &data) override;

  virtual void log() override;
  std::string const &get_sensorname() override { return this->get_name(); };

  void add_to_controller(ModbusController *master, ModbusFunctionCode register_type, uint16_t start_address,
                         uint8_t offset, uint32_t bitmask, SensorValueType value_type, int register_count,
                         uint8_t skip_updates);

 private:
  ModbusController *parent_{nullptr};
};

class ModbusSwitch : public Component, public switch_::Switch, public SensorItem {
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
  void set_modbus_parent(ModbusController *parent) { this->parent_ = parent; }
  void set_connected_sensor(binary_sensor::BinarySensor *connected_sensor) {
    this->connected_sensor_ = connected_sensor;
  }
  virtual float parse_and_publish(const std::vector<uint8_t> &data) override;
  virtual void log() override;
  std::string const &get_sensorname() override { return this->get_name(); };
  void add_to_controller(ModbusController *master, ModbusFunctionCode register_type, uint16_t start_address,
                         uint8_t offset, uint32_t bitmask);
  /*
    ModbusFunctionCode register_type;
    uint16_t start_address;
    uint8_t offset;
    uint32_t bitmask;
  */
 private:
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  ModbusController *parent_{nullptr};
};

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

/*
        config[CONF_NAME],
        cv.Optional(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Optional(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET): cv.int_,
        cv.Optional(CONF_BITMASK, default=0x1): cv.hex_uint32_t,
        cv.Optional(CONF_SKIP_UPDATES, default=0): cv.int_,
        cv.Optional(CONF_CREATE_SWITCH, default=False): cv.boolean,

*/

class ModbusBinarySensor : public Component, public binary_sensor::BinarySensor, public SensorItem {
 public:
  ModbusBinarySensor(const std::string &name) : binary_sensor::BinarySensor(name) {}
  ModbusBinarySensor(const std::string &name, ModbusFunctionCode register_type, uint16_t start_address, uint8_t offset,
                     uint32_t bitmask, uint8_t skip_updates = 0, bool create_switch = false)
      : Component(), binary_sensor::BinarySensor(name) {
    this->register_type = register_type;
    this->start_address = start_address;
    this->offset = offset;
    this->bitmask = bitmask, this->sensor_value_type = SensorValueType::BIT;
    this->skip_updates = skip_updates;
    this->create_switch = create_switch;
  }
  virtual float parse_and_publish(const std::vector<uint8_t> &data) override;
  std::string const &get_sensorname() override { return this->get_name(); };
  virtual void log() override;
  void update(){};
  void set_state(bool state) { this->state = state; }
  void set_modbus_parent(ModbusController *parent) { this->parent_ = parent; }
  void add_to_controller(ModbusController *master, ModbusFunctionCode register_type, uint16_t start_address,
                         uint8_t offset, uint32_t bitmask, bool create_switch = false, uint8_t skip_updates = 0);
  bool create_switch{false};
#ifdef USE_MQTT
  std::unique_ptr<mqtt::MQTTSwitchComponent> mqtt_switch;
#endif
  std::unique_ptr<ModbusSwitch> modbus_switch;

 private:
  ModbusController *parent_{nullptr};
};

struct ModbusCommandItem {
  // keep memory consumption low.  Since all registers are 2 bytes and only write RTC needs to be written in 1 command 8
  // bytes is enough
  static const size_t MAX_PAYLOAD_BYTES = 240;
  ModbusController *modbusdevice;
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
      ModbusController *modbusdevice, ModbusFunctionCode function_code, uint16_t start_address, uint16_t register_count,
      std::function<void(ModbusFunctionCode function_code, uint16_t start_address, const std::vector<uint8_t> &data)>
          &&handler);

  static ModbusCommandItem create_read_command(ModbusController *modbusdevice, ModbusFunctionCode function_code,
                                               uint16_t start_address, uint16_t register_count);
  static ModbusCommandItem create_write_multiple_command(ModbusController *modbusdevice, uint16_t start_address,
                                                         uint16_t register_count, const std::vector<uint16_t> &values);
  static ModbusCommandItem create_write_single_command(ModbusController *modbusdevice, uint16_t start_address,
                                                       int16_t value);
  static ModbusCommandItem create_write_single_coil(ModbusController *modbusdevice, uint16_t address, bool value);
  static ModbusCommandItem create_write_multiple_coils(ModbusController *modbusdevice, uint16_t start_address,
                                                       const std::vector<bool> &values);
};

class ModbusController : public PollingComponent, public modbus::ModbusDevice {
 public:
  ModbusController(uint16_t throttle = 0) : PollingComponent(), modbus::ModbusDevice(), command_throttle_(throttle){};
  size_t create_register_ranges();

  bool remove_register_range(uint16_t start_address);

  // find a register by it's address regardless of the offset
  SensorItem *find_by_register_address(ModbusFunctionCode function_code, uint16_t register_address) {
    // Not extemly effienct but the number of registers isn't that large
    // and the operation is used only in special cases
    // like changing a property during setup
    for (auto &item : this->sensormap) {
      if (register_address == item.second->start_address + item.second->offset &&
          function_code == item.second->register_type) {
        return item.second;
      }
    }
    return nullptr;
  }

  void update() override;
  void update_range(RegisterRange &r);
  void setup() override;
  void loop() override;
  void on_modbus_data(const std::vector<uint8_t> &data) override;
  void on_modbus_error(uint8_t function_code, uint8_t exception_code) override;

  void dump_config() override;
  void on_write_register_response(ModbusFunctionCode function_code, uint16_t start_address,
                                  const std::vector<uint8_t> &data);
  void on_register_data(ModbusFunctionCode function_code, uint16_t start_address, const std::vector<uint8_t> &data);
  void set_command_throttle(uint16_t command_throttle) { this->command_throttle_ = command_throttle; }

  void queue_command(const ModbusCommandItem &command);
  void add_sensor_item(SensorItem *item) { sensormap[item->getkey()] = item; }

 protected:
  bool send_next_command_();
  // Collection of all sensors for this component
  // see calc_key how the key is contructed
  std::map<uint64_t, SensorItem *> sensormap;
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
  explicit RawDataCodeTrigger(ModbusController *parent, text_sensor::TextSensor *sensor) {
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

}  // namespace modbus_controller
}  // namespace esphome

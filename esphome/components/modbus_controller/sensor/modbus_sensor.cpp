
#include "modbus_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace modbus_controller {

static const char *const TAG = "modbus_controller.sensor";

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

void ModbusSensor::dump_config() { LOG_SENSOR("", "Modbus Controller Sensor", this); }

void ModbusSensor::parse_and_publish(const std::vector<uint8_t> &data) {
  union {
    float float_value;
    uint32_t raw;
  } raw_to_float;

  int64_t value = 0;  // int64_t because it can hold signed and unsigned 32 bits
  float result = NAN;

  uint16_t buffer_offset = this->offset * 2;

  switch (sensor_value_type) {
    case SensorValueType::U_WORD:
      value =
          mask_and_shift_by_rightbit(get_data<uint16_t>(data, buffer_offset), this->bitmask);  // default is 0xFFFF ;
      result = static_cast<float>(value);
      break;
    case SensorValueType::U_DWORD:
      value = get_data<uint32_t>(data, buffer_offset);
      value = mask_and_shift_by_rightbit((uint32_t) value, this->bitmask);
      result = static_cast<float>(value);
      break;
    case SensorValueType::U_DWORD_R:
      value = get_data<uint32_t>(data, buffer_offset);
      value = static_cast<uint32_t>(value & 0xFFFF) << 16 | (value & 0xFFFF0000) >> 16;
      value = mask_and_shift_by_rightbit((uint32_t) value, this->bitmask);
      result = static_cast<float>(value);
      break;
    case SensorValueType::S_WORD:
      value = mask_and_shift_by_rightbit(get_data<int16_t>(data, buffer_offset),
                                         this->bitmask);  // default is 0xFFFF ;
      result = static_cast<float>(value);
      break;
    case SensorValueType::S_DWORD:
      value = mask_and_shift_by_rightbit(get_data<int32_t>(data, buffer_offset), this->bitmask);
      result = static_cast<float>(value);
      break;
    case SensorValueType::S_DWORD_R: {
      value = get_data<uint32_t>(data, buffer_offset);
      // Currently the high word is at the low position
      // the sign bit is therefore at low before the switch
      uint32_t sign_bit = (value & 0x8000) << 16;
      value = mask_and_shift_by_rightbit(
          static_cast<int32_t>(((value & 0x7FFF) << 16 | (value & 0xFFFF0000) >> 16) | sign_bit), this->bitmask);
      result = static_cast<float>(value);
    } break;
    case SensorValueType::U_QWORD:
      // Ignore bitmask for U_QWORD
      value = get_data<uint64_t>(data, buffer_offset);
      result = static_cast<float>(value);
      break;

    case SensorValueType::S_QWORD:
      // Ignore bitmask for S_QWORD
      value = get_data<int64_t>(data, buffer_offset);
      result = static_cast<float>(value);
      break;
    case SensorValueType::U_QWORD_R:
      // Ignore bitmask for U_QWORD
      value = get_data<uint64_t>(data, buffer_offset);
      value = static_cast<uint64_t>(value & 0xFFFF) << 48 | (value & 0xFFFF000000000000) >> 48 |
              static_cast<uint64_t>(value & 0xFFFF0000) << 32 | (value & 0x0000FFFF00000000) >> 32 |
              static_cast<uint64_t>(value & 0xFFFF00000000) << 16 | (value & 0x00000000FFFF0000) >> 16;
      result = static_cast<float>(value);
      break;

    case SensorValueType::S_QWORD_R:
      // Ignore bitmask for S_QWORD
      value = get_data<int64_t>(data, buffer_offset);
      result = static_cast<float>(value);
      break;
    case SensorValueType::FP32:
      raw_to_float.raw = get_data<uint32_t>(data, buffer_offset);
      result = raw_to_float.float_value;
      break;
    case SensorValueType::FP32_R: {
      auto tmp = get_data<uint32_t>(data, buffer_offset);
      raw_to_float.raw = static_cast<uint32_t>(tmp & 0xFFFF) << 16 | (tmp & 0xFFFF0000) >> 16;
      result = raw_to_float.float_value;
      break;
    }
    default:
      break;
  }

  this->publish_state(result);
}

}  // namespace modbus_controller
}  // namespace esphome

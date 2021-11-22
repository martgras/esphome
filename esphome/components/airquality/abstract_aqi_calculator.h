#pragma once
#include <cstdint>

namespace esphome {
namespace airquality {

enum Pollutant : uint8_t { PM25 = 0, PM10, NO2, O3, CO, SO2 };

class AbstractAQICalculator {
 public:
  virtual uint8_t calculate_index(uint16_t value, Pollutant pollutant_type) = 0;
};

}  // namespace airquality
}  // namespace esphome

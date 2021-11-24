#pragma once
#include <map>
#include <array>
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "aqi_calculator_factory.h"

namespace esphome {
namespace airquality {

struct Measurement {
  float minimum{NAN};
  float maximum{NAN};
  float avg{NAN};
  uint16_t samples{0};
};

// 24 hour history of measurements
class Measurements {
 public:
  // Overload index operator to access a measurement like an array item
  Measurement &operator[](int index) { return value_history_[index]; }
  Measurement &get_last() { return value_history_[last_index_]; }
  float add_to_history(float value);
  float get_avg(uint8_t hours);
  float get_max_avg(uint8_t hours);
  uint8_t get_history_size() {
    uint8_t idx = 0;
    while (!std::isnan(value_history_[idx++].maximum)) {
    }
    return idx;
  }

 protected:
  int last_index_{0};
  std::array<Measurement, 24> value_history_;
};

// calculate nowcast value based on the measurements of the last 12h hours.
// at least 2 hours of data are required for useful data
// to convert the value to an AQI index get the index for the value
float calculate_nowcast(Measurements &series);

class AirQualityComponent : public Component {
 public:
  AirQualityComponent() = default;

  void set_pm_2_5_sensor(sensor::Sensor *pm_2_5_sensor) {
    source_sensors_.emplace(PM25, pm_2_5_sensor);
    // value_history_.emplace(Pollutant::PM25,{});
  }
  void set_pm_10_0_sensor(sensor::Sensor *pm_10_0_sensor) { source_sensors_.emplace(PM10, pm_10_0_sensor); }
  void set_aqi_sensor(sensor::Sensor *aqi_sensor) { aqi_sensor_ = aqi_sensor; }
  void set_caqi_sensor(sensor::Sensor *caqi_sensor) { caqi_sensor_ = caqi_sensor; }
  void set_nowcast_sensor(sensor::Sensor *nowcast_sensor) { nowcast_sensor_ = nowcast_sensor; }

  void set_aqi_calculation_type(AQICalculatorType aqi_calc_type) { aqi_calc_type_ = aqi_calc_type; }

  void set_publish_window(uint32_t publish_window) { this->publish_window_ = publish_window; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  // void update() override;

 protected:
  uint16_t get_nowcast_index_(Pollutant pollutant);
  uint16_t calulate_aqi_(AQICalculatorType aqi_calc_type);
  bool in_publish_{false};
  float add_to_value_history_(Pollutant pollutant, float val);

  void publish_data_();

  sensor::Sensor *aqi_sensor_{nullptr};
  sensor::Sensor *caqi_sensor_{nullptr};
  sensor::Sensor *nowcast_sensor_{nullptr};
  // delay in ms between the first value callback and publishing
  // used to reduce publishes when source sensors are updated frequently
  uint32_t publish_window_{500};
  // Source sensors providing pollutant data
  std::map<Pollutant, sensor::Sensor *> source_sensors_;
  // last values from sensor
  std::map<Pollutant, float> source_sensor_values_;
  std::map<Pollutant, Measurements> measurements_;
  AQICalculatorType aqi_calc_type_;
  AQICalculatorFactory aqi_calculator_factory_ = AQICalculatorFactory();
};

}  // namespace airquality
}  // namespace esphome

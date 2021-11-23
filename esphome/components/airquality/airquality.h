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

// using Measurements = std::arrary<Measurement,24>;

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

  void set_aqi_calculation_type(AQICalculatorType aqi_calc_type) { aqi_calc_type_ = aqi_calc_type; }
  void set_publish_window(uint32_t publish_window) { this->publish_window_ = publish_window; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  // void update() override;
  uint16_t calc_airnow(Pollutant pollutant) ; 
 protected:
  uint8_t calulate_aqi_(AQICalculatorType aqi_calc_type);
  bool in_publish_{false};
  float add_to_value_history_(Pollutant pollutant, float val);

  void publish_on_data_complete_();

  sensor::Sensor *aqi_sensor_{nullptr};
  sensor::Sensor *caqi_sensor_{nullptr};
  // delay in ms between the first value callback and publishing
  // used to reduce publishes when source sensors are updated frequently
  uint32_t publish_window_{500};
  // Source sensors providing pollutant data
  std::map<Pollutant, sensor::Sensor *> source_sensors_;
  // last values from sensor
  std::map<Pollutant, float> source_sensor_values_;
  std::map<Pollutant, std::array<Measurement, 24>> value_history_;
  AQICalculatorType aqi_calc_type_;
  AQICalculatorFactory aqi_calculator_factory_ = AQICalculatorFactory();
};

}  // namespace airquality
}  // namespace esphome

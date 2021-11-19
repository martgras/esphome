#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "aqi_calculator_factory.h"

namespace esphome {
namespace airquality {

class AirQualityComponent : public Component {
 public:
  AirQualityComponent() = default;

  // void set_pm_1_0_sensor(sensor::Sensor *pm_1_0_sensor) { pm_1_0_sensor_ = pm_1_0_sensor; }
  void set_pm_2_5_sensor(sensor::Sensor *pm_2_5_sensor) { pm_2_5_sensor_ = pm_2_5_sensor; }
  void set_pm_10_0_sensor(sensor::Sensor *pm_10_0_sensor) { pm_10_0_sensor_ = pm_10_0_sensor; }
  void set_aqi_sensor(sensor::Sensor *aqi_sensor) { aqi_sensor_ = aqi_sensor; }
  void set_caqi_sensor(sensor::Sensor *caqi_sensor) { caqi_sensor_ = caqi_sensor; }

  void set_aqi_calculation_type(AQICalculatorType aqi_calc_type) { aqi_calc_type_ = aqi_calc_type; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  //void update() override;

 protected:
  uint8_t calulate_aqi_(AQICalculatorType aqi_calc_type);
  // void on_pm_1_0_callback_(float val) { pm_1_0_value_ = val; }
  void on_pm_2_5_callback_(float val) { pm_2_5_value_ = val; }
  void on_pm_10_0_callback_(float val) { pm_10_0_value_ = val; }
  //bool waiting_for_more data_ {false;}
  uint32_t pm_2_5_timestamp_;
  uint32_t pm_10_0_timestamp_;
  void publish_on_data_complete_() ;

  // float pm_1_0_value_{NAN};
  float pm_2_5_value_{NAN};
  float pm_10_0_value_{NAN};

  // sensor::Sensor *pm_1_0_sensor_{nullptr};
  sensor::Sensor *pm_2_5_sensor_{nullptr};
  sensor::Sensor *pm_10_0_sensor_{nullptr};
  sensor::Sensor *aqi_sensor_{nullptr};
  sensor::Sensor *caqi_sensor_{nullptr};

  AQICalculatorType aqi_calc_type_;
  AQICalculatorFactory aqi_calculator_factory_ = AQICalculatorFactory();
};

}  // namespace airquality
}  // namespace esphome

#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace airquality {

class AirQualityComponent : public PollingComponent {
 public:
  AirQualityComponent() = default;
  // these methods must be overridden in your component
  void setup() override;
  void dump_config() override;
  void update() override;

  // register the sensor with the component
  void set_aqi_sensor(sensor::Sensor *aqi_sensor) { this->aqi_sensor_ = aqi_sensor; }

 protected:
  sensor::Sensor *aqi_sensor_{nullptr};
};

}  // namespace airquality
}  // namespace esphome

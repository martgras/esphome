#include <map>
#include <ctime>
#include <math.h>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "airquality.h"

namespace esphome {
namespace airquality {

static const char *const TAG = "aitquality.sensor";

void AirQualityComponent::setup() {
  // Perform steps required to initialize the component
  ESP_LOGCONFIG(TAG, "Setting up air_quality sensor...");
}

void AirQualityComponent::dump_config() {
  // Log all your configuration details
  ESP_LOGCONFIG(TAG, "Air Quality:");
  // Log the sensor propertierties
  LOG_SENSOR(TAG, "AQI Sensor", this->aqi_sensor_);
}

// Because this component derives from PollingComponent update will be called
// regulary as defined by the update_interval property
void AirQualityComponent::update() {
  float new_value = 4711.0f;  // just hardcode the value for the sake of the example
  ESP_LOGD(TAG, "new AQI value = %f", new_value);

  // publish the new value of the AQI sensor
  if (this->aqi_sensor_) {
    aqi_sensor_->publish_state(4711.0f);
  }
}

}  // namespace airquality
}  // namespace esphome

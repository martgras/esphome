#include <map>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "airquality.h"

namespace esphome {
namespace airquality {

static const char *const TAG = "aitquality.sensor";

const char *get_pollutant_name(Pollutant p) {
  switch (p) {
    case PM25:
      return "PM 2.5";
      break;
    case PM10:
      return "PM 10";
      break;
    case NO2:
      return "NO2";
      break;
    case O3:
      return "O3";
      break;
    case CO:
      return "CO";
      break;
    case SO2:
      return "SO2";
      break;
  }
  return "";
}

void AirQualityComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up air_quality sensor...");

  // Use callbacks when the source sensors are updated
  for (auto const &it : this->source_sensors_) {
    it.second->add_on_state_callback([this, it](float val) {
      this->source_sensor_values_[it.first] = val;
      this->publish_on_data_complete_();
    });
  };
}

#include <esp_heap_caps.h>
#include <esp_system.h>

// directly publishing from the callback would trigger multiple publishes
// wait for publish_window_ ms for the other callback to update values
void AirQualityComponent::publish_on_data_complete_() {
  if (in_publish_)
    return;
  this->in_publish_ = true;
  set_timeout(publish_window_, [this]() {
    int8_t aqi_value = 0;
    int8_t caqi_value = 0;
    float aqi = 0.0f;

    ESP_LOGD(TAG, "Update from source sensors received");
    auto freemem = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGD(TAG, "Freemem =%d", freemem);
    if (this->aqi_sensor_ != nullptr) {
      int8_t aqi_value = this->calulate_aqi_(AQI_TYPE);
      if (aqi_value != -1) {
        this->aqi_sensor_->publish_state(aqi_value);
      }
    }
    if (this->caqi_sensor_ != nullptr) {
      int8_t caqi_value = this->calulate_aqi_(CAQI_TYPE);
      if (caqi_value != -1) {
        this->caqi_sensor_->publish_state(caqi_value);
      }
    }
    this->in_publish_ = false;
  });
}
void AirQualityComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Air Quality:");
  if (this->aqi_sensor_)
    LOG_SENSOR("  ", "AQI", this->aqi_sensor_);
  if (this->caqi_sensor_)
    LOG_SENSOR("  ", "CAQI", this->caqi_sensor_);
  for (auto const &it : this->source_sensors_) {
    ESP_LOGCONFIG(TAG, "  Pollutant: %s: Source Sensor %s", get_pollutant_name(it.first),
                  it.second->get_name().c_str());
  }
}

float AirQualityComponent::get_setup_priority() const { return setup_priority::DATA; }

uint8_t AirQualityComponent::calulate_aqi_(AQICalculatorType aqi_calc_type) {
  AbstractAQICalculator *calculator = this->aqi_calculator_factory_.get_calculator(aqi_calc_type);
  uint8_t aqi_value = 0;
  for (auto const &it : this->source_sensors_) {
    auto sensor_value = this->source_sensor_values_[it.first];
    aqi_value = std::max(aqi_value, calculator->calculate_index(sensor_value, it.first));
  }
  return aqi_value;
}

}  // namespace airquality
}  // namespace esphome

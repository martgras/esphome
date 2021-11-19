#include "esphome/core/log.h"
#include "airquality.h"

namespace esphome {
namespace airquality {

static const char *const TAG = "aitquality.sensor";

static const uint8_t PM_1_0_VALUE_INDEX = 5;
static const uint8_t PM_2_5_VALUE_INDEX = 6;
static const uint8_t PM_10_0_VALUE_INDEX = 7;

void AirQualityComponent::setup() { ESP_LOGCONFIG(TAG, "Setting up HM3301..."); }

void AirQualityComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Air Quality:");

  LOG_SENSOR("  ", "PM1.0", this->pm_1_0_sensor_);
  LOG_SENSOR("  ", "PM2.5", this->pm_2_5_sensor_);
  LOG_SENSOR("  ", "PM10.0", this->pm_10_0_sensor_);
  LOG_SENSOR("  ", "AQI", this->aqi_sensor_);
}

float AirQualityComponent::get_setup_priority() const { return setup_priority::DATA; }

void AirQualityComponent::update() {
  int16_t pm_1_0_value = -1;
  if (this->pm_1_0_sensor_ != nullptr) {
    pm_1_0_value = this->pm_1_0_sensor_->state;
  }
  // TODO state is float - int16_t expected check the dimension
  int16_t pm_2_5_value = -1;
  if (this->pm_2_5_sensor_ != nullptr) {
    pm_2_5_value = this->pm_2_5_sensor_->state;
  }

  int16_t pm_10_0_value = -1;
  if (this->pm_10_0_sensor_ != nullptr) {
    pm_10_0_value = this->pm_10_0_sensor_->state;
  }

  int8_t aqi_value = -1;
  if (this->aqi_sensor_ != nullptr && pm_2_5_value != -1 && pm_10_0_value != -1) {
    AbstractAQICalculator *calculator = this->aqi_calculator_factory_.get_calculator(this->aqi_calc_type_);
    aqi_value = calculator->get_aqi(pm_2_5_value, pm_10_0_value);
  }

  if (pm_1_0_value != -1) {
    this->pm_1_0_sensor_->publish_state(pm_1_0_value);
  }
  if (pm_2_5_value != -1) {
    this->pm_2_5_sensor_->publish_state(pm_2_5_value);
  }
  if (pm_10_0_value != -1) {
    this->pm_10_0_sensor_->publish_state(pm_10_0_value);
  }
  if (aqi_value != -1) {
    this->aqi_sensor_->publish_state(aqi_value);
  }

  this->status_clear_warning();
}

}  // namespace airquality
}  // namespace esphome

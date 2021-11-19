#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "airquality.h"

namespace esphome {
namespace airquality {

static const char *const TAG = "aitquality.sensor";

static const uint8_t PM_1_0_VALUE_INDEX = 5;
static const uint8_t PM_2_5_VALUE_INDEX = 6;
static const uint8_t PM_10_0_VALUE_INDEX = 7;

void AirQualityComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up air_quality sensor...");

  // TODO - decide if this is a better approach instead of polling the sensors
  // Use callbacks when the source sensors are updated
  //
  if (this->pm_2_5_sensor_ != nullptr) {
    this->pm_2_5_sensor_->add_on_state_callback([this](float val) {
      this->pm_2_5_value_ = val;
      this->pm_2_5_timestamp_ = millis();
      this->publish_on_data_complete_();
    });
  }
  if (this->pm_10_0_sensor_ != nullptr) {
    this->pm_10_0_sensor_->add_on_state_callback([this](float val) {
      this->pm_10_0_value_ = val;
      this->pm_10_0_timestamp_ = millis();
      this->publish_on_data_complete_();
    });
  }
}

// directly publishing from the callback would tirgger 2 publishes
// wait for the second sensor callback. if the time between the timestamps
// is smaller than 500ms consider data collection complete
void AirQualityComponent::publish_on_data_complete_() {
  // Publish only if both sensors were updated recently
  if (abs(pm_2_5_timestamp_ - pm_10_0_timestamp_) < 500) {
    // reset timestamps - they need a diff > 500
    pm_2_5_timestamp_ = 0;
    pm_10_0_timestamp_ = 1000;

    ESP_LOGD(TAG, "New sensor data received");
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
  }
}
void AirQualityComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Air Quality:");

  // LOG_SENSOR("  ", "PM1.0", this->pm_1_0_sensor_);
  LOG_SENSOR("  ", "PM2.5", this->pm_2_5_sensor_);
  LOG_SENSOR("  ", "PM10.0", this->pm_10_0_sensor_);
  LOG_SENSOR("  ", "AQI", this->aqi_sensor_);
}

float AirQualityComponent::get_setup_priority() const { return setup_priority::DATA; }

#if USE_POLLING
void AirQualityComponent::update() {
  int16_t pm_2_5_value = -1;
  if (this->pm_2_5_sensor_ != nullptr) {
    pm_2_5_value_ = this->pm_2_5_sensor_->state;
  }

  int16_t pm_10_0_value = -1;
  if (this->pm_10_0_sensor_ != nullptr) {
    pm_10_0_value_ = this->pm_10_0_sensor_->state;
  }

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

  this->status_clear_warning();
}
#endif

uint8_t AirQualityComponent::calulate_aqi_(AQICalculatorType aqi_calc_type) {
  if (pm_2_5_value_ != NAN && pm_10_0_value_ != NAN) {
    int16_t pm_2_5_value = -1;
    int16_t pm_10_0_value = -1;
    pm_2_5_value = static_cast<int16_t>(this->pm_2_5_value_);
    pm_10_0_value = static_cast<int16_t>(this->pm_10_0_value_);
    AbstractAQICalculator *calculator = this->aqi_calculator_factory_.get_calculator(aqi_calc_type);
    return calculator->get_aqi(pm_2_5_value, pm_10_0_value);
  } else {
    return -1;
  };
}

}  // namespace airquality
}  // namespace esphome

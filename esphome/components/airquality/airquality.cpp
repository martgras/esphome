#include <map>
#include <ctime>
#include <math.h>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "airquality.h"

namespace esphome {
namespace airquality {

static const char *const TAG = "aitquality.sensor";

const char *get_aqi_type(AQICalculatorType c) {
  switch (c) {
    case AQI_TYPE:
      return "AQI";
      break;
    case CAQI_TYPE:
      return "CAQI";
      break;
    default:
      return "";
      break;
  }
}

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
    default:
      return "";
      break;
  }
}

// average of the the averages of the last hours
float Measurements::get_avg(uint8_t hours) {
  float avg = 0;
  uint8_t idx = 0;
  // stop at the first NAN in history
  // marks the end of the history
  while (!std::isnan(value_history_[idx].avg)) {
    avg += value_history_[idx].avg;
    idx++;
  }
  return avg / static_cast<float>(idx);
}

// average of the the maximum of the last hours
float Measurements::get_max_avg(uint8_t hours) {
  float avg = 0;
  uint8_t idx = 0;
  while (!std::isnan(value_history_[idx].maximum)) {
    avg += value_history_[idx].maximum;
    idx++;
  }
  return avg / static_cast<float>(idx);
}

float Measurements::add_to_history(float val) {
  auto t_now = std::time(nullptr);
  uint8_t hour = ::localtime(&t_now)->tm_hour;
  auto m = this->value_history_[hour];

  if (std::isnan(m.maximum))
    m.maximum = val;
  else if (val > m.maximum)
    m.maximum = val;

  if (std::isnan(m.minimum))
    m.minimum = val;
  else if (val < m.minimum)
    m.minimum = val;

  if (std::isnan(m.avg)) {
    m.avg = val;
    m.samples = 1;
  } else {
    m.avg = (m.avg * m.samples + val) / static_cast<float>(m.samples + 1);
    m.samples++;
  }
  this->value_history_[hour] = m;
  this->last_index_ = hour;
  return val;
}

void AirQualityComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up air_quality sensor...");

  // Use callbacks when the source sensors are updated
  for (auto const &it : this->source_sensors_) {
    it.second->add_on_state_callback([this, it](float val) {
      this->source_sensor_values_[it.first] = val;
      this->add_to_value_history_(it.first, val);
    });
  };
}

void AirQualityComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Air Quality:");
  // loop over the configured decision grids
  for (auto &sens : this->aqi_sensors_) {
    LOG_SENSOR("  ", get_aqi_type(sens.first), sens.second);
  }
  for (auto const &it : this->source_sensors_) {
    ESP_LOGCONFIG(TAG, "  Pollutant: %s: Source Sensor %s", get_pollutant_name(it.first),
                  it.second->get_name().c_str());
  }
}

void AirQualityComponent::update() {
  int8_t aqi_value = 0;
  int8_t caqi_value = 0;
  float aqi = 0.0f;

  // loop over the configured decision grids
  for (auto &sens : this->aqi_sensors_) {
    uint16_t aqi_value = 0;
    aqi_value = std::max(aqi_value, calculate_aqi_(sens.first));
    sens.second->publish_state(aqi_value);
  }

  if (this->nowcast_sensor_ != nullptr) {
    uint16_t aqi_value = 0;
    // loop over all polutants configured
    for (auto const &it : this->source_sensors_) {
      auto sensor_value = this->source_sensor_values_[it.first];
      aqi_value = std::max(aqi_value, get_nowcast_index_(it.first));
    }
    this->nowcast_sensor_->publish_state(aqi_value);
  }
}

float AirQualityComponent::get_setup_priority() const { return setup_priority::DATA; }

// loops over all configured pollutants and returns the max aqi for all of them
uint16_t AirQualityComponent::calculate_aqi_(AQICalculatorType aqi_calc_type) {
  AQICalculatorBase *calculator = this->calculators_[aqi_calc_type];
  uint16_t aqi_value = 0;
  for (auto const &it : this->source_sensors_) {
    auto sensor_value = this->source_sensor_values_[it.first];
    aqi_value = std::max(aqi_value, calculator->calculate_index(sensor_value, it.first));
  }
  return aqi_value;
}

// Add current value to value_history and calc max,min, avg for every hour
float AirQualityComponent::add_to_value_history_(Pollutant pollutant, float val) {
  // create new entry if history for this pullutant doesn't exist
  if (!this->measurements_.count(pollutant)) {
    Measurements m;
    this->measurements_.emplace(pollutant, m);
  }
  this->measurements_[pollutant].add_to_history(val);
  ESP_LOGD(TAG, "Added val %f", val);
  return val;
}

// Calculate NowCast value and map it to AQI Index
uint16_t AirQualityComponent::get_nowcast_index_(Pollutant pollutant) {
  auto series = measurements_[pollutant];
  auto aqi_val = calculate_nowcast(series);
  AQICalculatorBase *calculator = this->calculators_[AQI_TYPE];
  uint16_t aqi = calculator->calculate_index(aqi_val, pollutant);
  ESP_LOGD(TAG, "Nowcast value = %f NowCast AQI = %d", aqi_val, aqi);
  return aqi;
}

// ref: https://forum.airnowtech.org/t/the-nowcast-for-pm2-5-and-pm10/172
float calculate_nowcast(Measurements &series) {
  auto t_now = std::time(nullptr);
  int current_hour = ::localtime(&t_now)->tm_hour;
  // find min - max for the last 12 hours
  float maxvalue = 0.0f;
  float minvalue = 100000.0f;
  int n_samples = 0;
  for (int i = 0; i < 12; i++) {
    auto h = current_hour - i;
    // warp hours
    if (h < 0)
      h += 24;
    // No more data
    if (std::isnan(series[h].avg)) {
      break;
    }
    if (series[h].avg > maxvalue)
      maxvalue = series[h].avg;
    if (series[h].avg < minvalue)
      minvalue = series[h].avg;
    n_samples++;
  }
  // need at least 2 hourly samples
  if (n_samples < 2)
    return 0;

  auto range = maxvalue - minvalue;
  float sum = 0.0f;
  float factor = range / maxvalue;
  if (factor < 0.5f)
    factor = 0.5f;
  if (factor > 1.0f)
    factor = 1.0f;

  ESP_LOGVV(TAG, "Nowcast samples = %d min max range = %f weight factor=%f", n_samples, range, factor);
  // sum up all measurements multiplied by weight_facor^hours
  for (int i = 0; i < n_samples; i++) {
    auto h = current_hour - i;
    // warp hours
    sum += powf(factor, i) * series[h].avg;
  }
  auto aqi_val = sum / ((1 - powf(factor, n_samples + 1)) / (1 - factor));
  ESP_LOGD(TAG, "NowCast = %f", aqi_val);
  return aqi_val;
}

const constexpr int AQICalc::index_grid_[AQICalc::get_num_levels()][2];
const constexpr int AQICalc::calculation_grids_[(Pollutant::SO2 + 1)][AQICalc::get_num_levels()][2];
const constexpr int CAQICalc::index_grid_[CAQICalc::get_num_levels()][2];
const constexpr int CAQICalc::calculation_grids_[(Pollutant::SO2 + 1)][CAQICalc::get_num_levels()][2];

}  // namespace airquality
}  // namespace esphome

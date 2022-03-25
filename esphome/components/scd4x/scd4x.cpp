#include "scd4x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace scd4x {

static const char *const TAG = "scd4x";

static const uint16_t SCD4X_CMD_GET_SERIAL_NUMBER = 0x3682;
static const uint16_t SCD4X_CMD_TEMPERATURE_OFFSET = 0x241d;
static const uint16_t SCD4X_CMD_ALTITUDE_COMPENSATION = 0x2427;
static const uint16_t SCD4X_CMD_AMBIENT_PRESSURE_COMPENSATION = 0xe000;
static const uint16_t SCD4X_CMD_AUTOMATIC_SELF_CALIBRATION = 0x2416;
static const uint16_t SCD4X_CMD_START_CONTINUOUS_MEASUREMENTS = 0x21b1;
static const uint16_t SCD4X_CMD_GET_DATA_READY_STATUS = 0xe4b8;
static const uint16_t SCD4X_CMD_READ_MEASUREMENT = 0xec05;
static const uint16_t SCD4X_CMD_PERFORM_FORCED_CALIBRATION = 0x362f;
static const uint16_t SCD4X_CMD_STOP_MEASUREMENTS = 0x3f86;

static const float SCD4X_TEMPERATURE_OFFSET_MULTIPLIER = (1 << 16) / 175.0f;

void SCD4XComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up scd4x...");

  // the sensor needs 1000 ms to enter the idle state
  this->set_timeout(1000, [this]() {
    // Check if measurement is ready before reading the value
    if (!this->write_command(SCD4X_CMD_GET_DATA_READY_STATUS)) {
      ESP_LOGE(TAG, "Failed to write data ready status command");
      this->mark_failed();
      return;
    }

    uint16_t raw_read_status;
    if (!this->read_data(raw_read_status)) {
      ESP_LOGE(TAG, "Failed to read data ready status");
      this->mark_failed();
      return;
    }

    uint32_t stop_measurement_delay = 0;
    // In order to query the device periodic measurement must be ceased
    if (raw_read_status) {
      ESP_LOGD(TAG, "Sensor has data available, stopping periodic measurement");
      if (!this->write_command(SCD4X_CMD_STOP_MEASUREMENTS)) {
        ESP_LOGE(TAG, "Failed to stop measurements");
        this->mark_failed();
        return;
      }
      // According to the SCD4x datasheet the sensor will only respond to other commands after waiting 500 ms after
      // issuing the stop_periodic_measurement command
      stop_measurement_delay = 500;
    }
    this->set_timeout(stop_measurement_delay, [this]() {
      if (!this->write_command(SCD4X_CMD_GET_SERIAL_NUMBER)) {
        ESP_LOGE(TAG, "Failed to write get serial command");
        this->error_code_ = COMMUNICATION_FAILED;
        this->mark_failed();
        return;
      }

      uint16_t raw_serial_number[3];
      if (!this->read_data(raw_serial_number, 3)) {
        ESP_LOGE(TAG, "Failed to read serial number");
        this->error_code_ = SERIAL_NUMBER_IDENTIFICATION_FAILED;
        this->mark_failed();
        return;
      }
      ESP_LOGD(TAG, "Serial number %02d.%02d.%02d", (uint16_t(raw_serial_number[0]) >> 8),
               uint16_t(raw_serial_number[0] & 0xFF), (uint16_t(raw_serial_number[1]) >> 8));

      if (!this->write_command(SCD4X_CMD_TEMPERATURE_OFFSET,
                               (uint16_t) (temperature_offset_ * SCD4X_TEMPERATURE_OFFSET_MULTIPLIER))) {
        ESP_LOGE(TAG, "Error setting temperature offset.");
        this->error_code_ = MEASUREMENT_INIT_FAILED;
        this->mark_failed();
        return;
      }

      // If pressure compensation available use it
      // else use altitude
      if (ambient_pressure_compensation_) {
        if (!this->update_ambient_pressure_compensation_(ambient_pressure_)) {
          ESP_LOGE(TAG, "Error setting ambient pressure compensation.");
          this->error_code_ = MEASUREMENT_INIT_FAILED;
          this->mark_failed();
          return;
        }
      } else {
        if (!this->write_command(SCD4X_CMD_ALTITUDE_COMPENSATION, altitude_compensation_)) {
          ESP_LOGE(TAG, "Error setting altitude compensation.");
          this->error_code_ = MEASUREMENT_INIT_FAILED;
          this->mark_failed();
          return;
        }
      }

      if (!this->write_command(SCD4X_CMD_AUTOMATIC_SELF_CALIBRATION, enable_asc_ ? 1 : 0)) {
        ESP_LOGE(TAG, "Error setting automatic self calibration.");
        this->error_code_ = MEASUREMENT_INIT_FAILED;
        this->mark_failed();
        return;
      }

      // Finally start sensor measurements
      if (!this->write_command(SCD4X_CMD_START_CONTINUOUS_MEASUREMENTS)) {
        ESP_LOGE(TAG, "Error starting continuous measurements.");
        this->error_code_ = MEASUREMENT_INIT_FAILED;
        this->mark_failed();
        return;
      }

      initialized_ = true;
      ESP_LOGD(TAG, "Sensor initialized");
    });
  });
}

void SCD4XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "scd4x:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    switch (this->error_code_) {
      case COMMUNICATION_FAILED:
        ESP_LOGW(TAG, "Communication failed! Is the sensor connected?");
        break;
      case MEASUREMENT_INIT_FAILED:
        ESP_LOGW(TAG, "Measurement Initialization failed!");
        break;
      case SERIAL_NUMBER_IDENTIFICATION_FAILED:
        ESP_LOGW(TAG, "Unable to read sensor firmware version");
        break;
      default:
        ESP_LOGW(TAG, "Unknown setup error!");
        break;
    }
  }
  ESP_LOGCONFIG(TAG, "  Automatic self calibration: %s", ONOFF(this->enable_asc_));
  if (this->ambient_pressure_compensation_) {
    ESP_LOGCONFIG(TAG, "  Altitude compensation disabled");
    ESP_LOGCONFIG(TAG, "  Ambient pressure compensation: %dmBar", this->ambient_pressure_);
  } else {
    ESP_LOGCONFIG(TAG, "  Ambient pressure compensation disabled");
    ESP_LOGCONFIG(TAG, "  Altitude compensation: %dm", this->altitude_compensation_);
  }
  ESP_LOGCONFIG(TAG, "  Temperature offset: %.2f °C", this->temperature_offset_);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

void SCD4XComponent::update() {
  if (!initialized_) {
    return;
  }

  if (this->ambient_pressure_source_ != nullptr) {
    float pressure = this->ambient_pressure_source_->state / 1000.0f;
    if (!std::isnan(pressure)) {
      set_ambient_pressure_compensation(this->ambient_pressure_source_->state / 1000.0f);
    }
  }

  // Check if data is ready
  if (!this->write_command(SCD4X_CMD_GET_DATA_READY_STATUS)) {
    this->status_set_warning();
    return;
  }

  uint16_t raw_read_status;
  if (!this->read_data(raw_read_status) || raw_read_status == 0x00) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Data not ready yet!");
    return;
  }

  if (!this->write_command(SCD4X_CMD_READ_MEASUREMENT)) {
    ESP_LOGW(TAG, "Error reading measurement!");
    this->status_set_warning();
    return;
  }

  // Read off sensor data
  uint16_t raw_data[3];
  if (!this->read_data(raw_data, 3)) {
    this->status_set_warning();
    return;
  }

  if (this->co2_sensor_ != nullptr)
    this->co2_sensor_->publish_state(raw_data[0]);

  if (this->temperature_sensor_ != nullptr) {
    const float temperature = -45.0f + (175.0f * (raw_data[1])) / (1 << 16);
    this->temperature_sensor_->publish_state(temperature);
  }

  if (this->humidity_sensor_ != nullptr) {
    const float humidity = (100.0f * raw_data[2]) / (1 << 16);
    this->humidity_sensor_->publish_state(humidity);
  }

  this->status_clear_warning();
}
// Note pressure in bar here. Convert to hPa
void SCD4XComponent::set_ambient_pressure_compensation(float pressure_in_bar) {
  ambient_pressure_compensation_ = true;
  uint16_t new_ambient_pressure = (uint16_t) (pressure_in_bar * 1000);
  // remove millibar from comparison to avoid frequent updates +/- 10 millibar doesn't matter
  if (initialized_ && (new_ambient_pressure / 10 != ambient_pressure_ / 10)) {
    update_ambient_pressure_compensation_(new_ambient_pressure);
    ambient_pressure_ = new_ambient_pressure;
  } else {
    ESP_LOGD(TAG, "ambient pressure compensation skipped - no change required");
  }
}

bool SCD4XComponent::update_ambient_pressure_compensation_(uint16_t pressure_in_hpa) {
  if (this->write_command(SCD4X_CMD_AMBIENT_PRESSURE_COMPENSATION, pressure_in_hpa)) {
    ESP_LOGD(TAG, "setting ambient pressure compensation to %d hPa", pressure_in_hpa);
    return true;
  } else {
    ESP_LOGE(TAG, "Error setting ambient pressure compensation.");
    return false;
  }
}

}  // namespace scd4x
}  // namespace esphome

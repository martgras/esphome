#include "sgp4x.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cinttypes>

namespace esphome {
namespace sgp4x {

static const char *const TAG = "sgp4x";

void SGP4xComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SGP40...");

  // Serial Number identification
  if (!this->write_command_(SGP40_CMD_GET_SERIAL_ID)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  uint16_t raw_serial_number[3];

  if (!this->read_data_(raw_serial_number, 3)) {
    this->mark_failed();
    return;
  }
  this->serial_number_ = (uint64_t(raw_serial_number[0]) << 24) | (uint64_t(raw_serial_number[1]) << 16) |
                         (uint64_t(raw_serial_number[2]));
  ESP_LOGD(TAG, "Serial Number: %" PRIu64, this->serial_number_);

  // Featureset identification for future use
  if (!this->write_command_(SGP40_CMD_GET_FEATURESET)) {
    ESP_LOGD(TAG, "raw_featureset write_command_ failed");
    this->mark_failed();
    return;
  }
  uint16_t raw_featureset[1];
  if (!this->read_data_(raw_featureset, 1)) {
    ESP_LOGD(TAG, "raw_featureset read_data_ failed");
    this->mark_failed();
    return;
  }

  this->featureset_ = raw_featureset[0];
  if ((this->featureset_ & 0x1FF) == SGP40_FEATURESET) {
    sgp_type_ = SGP40;
    self_test_time_ = SPG40_SELFTEST_TIME;
    measure_time_ = SGP40_MEASURE_TIME;
    if (this->nox_sensor_) {
      ESP_LOGE(TAG, "Measuring NOx requires a SGP41 sensor but a SGP40 sensor is detected");
      // disable the sensor
      this->nox_sensor_->set_disabled_by_default(true);
      // make sure it's not visiable in HA
      this->nox_sensor_->set_internal(true);
      this->nox_sensor_->state = NAN;
      // remove pointer to sensor
      this->nox_sensor_ = nullptr;
    }
  } else {
    if ((this->featureset_ & 0x1FF) == SGP41_FEATURESET) {
      sgp_type_ = SGP41;
      self_test_time_ = SPG41_SELFTEST_TIME;
      measure_time_ = SGP41_MEASURE_TIME;
    } else {
      ESP_LOGD(TAG, "Product feature set failed 0x%0X , expecting 0x%0X", uint16_t(this->featureset_ & 0x1FF),
               SGP40_FEATURESET);
      this->mark_failed();
      return;
    }
  }

  ESP_LOGD(TAG, "Product version: 0x%0X", uint16_t(this->featureset_ & 0x1FF));

  if (this->store_baseline_) {
    // Hash with compilation time
    // This ensures the baseline storage is cleared after OTA
    uint32_t hash = fnv1_hash(App.get_compilation_time());
    this->pref_ = global_preferences->make_preference<SGP4xBaselines>(hash, true);

    if (this->pref_.load(&this->voc_baselines_storage_)) {
      this->voc_state0_ = this->voc_baselines_storage_.state0;
      this->voc_state1_ = this->voc_baselines_storage_.state1;
      ESP_LOGI(TAG, "Loaded VOC baseline state0: 0x%04X, state1: 0x%04X", this->voc_baselines_storage_.state0,
               voc_baselines_storage_.state1);
    }

    // Initialize storage timestamp
    this->seconds_since_last_store_ = 0;

    if (this->voc_baselines_storage_.state0 > 0 && this->voc_baselines_storage_.state1 > 0) {
      ESP_LOGI(TAG, "Setting VOC baseline from save state0: 0x%04X, state1: 0x%04X",
               this->voc_baselines_storage_.state0, voc_baselines_storage_.state1);
      voc_algorithm_.set_states(this->voc_baselines_storage_.state0, this->voc_baselines_storage_.state1);
    }
  }

  this->self_test_();

  /* The official spec for this sensor at
  https://sensirion.com/media/documents/296373BB/6203C5DF/Sensirion_Gas_Sensors_Datasheet_SGP40.pdf indicates this
  sensor should be driven at 1Hz. Comments from the developers at: https://github.com/Sensirion/embedded-sgp/issues/136
  indicate the algorithm should be a bit resilient to slight timing variations so the software timer should be accurate
  enough for this.

  This block starts sampling from the sensor at 1Hz, and is done seperately from the call
  to the update method. This seperation is to support getting accurate measurements but
  limit the amount of communication done over wifi for power consumption or to keep the
  number of records reported from being overwhelming.
  */
  ESP_LOGD(TAG, "Component requires sampling of 1Hz, setting up background sampler");
  this->set_interval(1000, [this]() { this->update_gas_indices(); });
}

void SGP4xComponent::self_test_() {
  ESP_LOGD(TAG, "Self-test started");
  if (!this->write_command_(SGP40_CMD_SELF_TEST)) {
    this->error_code_ = COMMUNICATION_FAILED;
    ESP_LOGD(TAG, "Self-test communication failed");
    this->mark_failed();
  }

  this->set_timeout(self_test_time_, [this]() {
    uint16_t reply[2];
    if (!this->read_data_(reply, 1)) {
      ESP_LOGD(TAG, "Self-test read_data_ failed");
      this->mark_failed();
      return;
    }

    if (reply[0] == 0xD400) {
      this->self_test_complete_ = true;
      ESP_LOGD(TAG, "Self-test completed");
      return;
    } else {
      ESP_LOGD(TAG, "Self-test failed 0x%X", reply[0]);
      return;
    }

    ESP_LOGD(TAG, "Self-test failed 0x%X", reply[0]);
    this->mark_failed();
  });
}

/**
 * @brief Combined the measured gasses, temperature, and humidity
 * to calculate the VOC Index
 *
 * @param temperature The measured temperature in degrees C
 * @param humidity The measured relative humidity in % rH
 * @return int32_t The VOC Index
 */
bool SGP4xComponent::measure_gas_indices_(int32_t &voc, int32_t &nox) {
  uint16_t voc_sraw;
  uint16_t nox_sraw;
  if (!measure_raw_(voc_sraw, nox_sraw))
    return false;

  this->status_clear_warning();

  voc = voc_algorithm_.process(voc_sraw);
  if (nox_sensor_) {
    nox = nox_algorithm_.process(nox_sraw);
  }
  ESP_LOGV(TAG, "VOC = %d, NOx = %d", voc, nox);
  // Store baselines after defined interval or if the difference between current and stored baseline becomes too
  // much
  if (this->store_baseline_ && this->seconds_since_last_store_ > SHORTEST_BASELINE_STORE_INTERVAL) {
    voc_algorithm_.get_states(this->voc_state0_, this->voc_state1_);
    if ((uint32_t) abs(this->voc_baselines_storage_.state0 - this->voc_state0_) > MAXIMUM_STORAGE_DIFF ||
        (uint32_t) abs(this->voc_baselines_storage_.state1 - this->voc_state1_) > MAXIMUM_STORAGE_DIFF) {
      this->seconds_since_last_store_ = 0;
      this->voc_baselines_storage_.state0 = this->voc_state0_;
      this->voc_baselines_storage_.state1 = this->voc_state1_;

      if (this->pref_.save(&this->voc_baselines_storage_)) {
        ESP_LOGI(TAG, "Stored VOC baseline state0: 0x%04X ,state1: 0x%04X", this->voc_baselines_storage_.state0,
                 voc_baselines_storage_.state1);
      } else {
        ESP_LOGW(TAG, "Could not store VOC baselines");
      }
    }
  }

  return true;
}

/**
 * @brief Return the raw gas measurement
 *
 * @param temperature The measured temperature in degrees C
 * @param humidity The measured relative humidity in % rH
 * @return uint16_t The current raw gas measurement
 */
bool SGP4xComponent::measure_raw_(uint16_t &voc_raw, uint16_t &nox_raw) {
  float humidity = NAN;
  static uint32_t nox_conditioning_start = millis();

  if (!this->self_test_complete_) {
    ESP_LOGD(TAG, "Self-test not yet complete");
    return false;
  }
  if (this->humidity_sensor_ != nullptr) {
    humidity = this->humidity_sensor_->state;
  }
  if (std::isnan(humidity) || humidity < 0.0f || humidity > 100.0f) {
    humidity = 50;
  }

  float temperature = NAN;
  if (this->temperature_sensor_ != nullptr) {
    temperature = float(this->temperature_sensor_->state);
  }
  if (std::isnan(temperature) || temperature < -40.0f || temperature > 85.0f) {
    temperature = 25;
  }

  uint8_t command[8];
  size_t response_words;
  command[0] = 0x26;
  // Use SGP40 measure command if we don't care about NOx
  if (nox_sensor_ == nullptr) {
    command[1] = SGP40_SUBCMD_MEASURE_RAW;
    response_words = 1;
  } else {
    // SGP41 sensor must use NOx conditioning command for the first 10 seconds
    if (millis() - nox_conditioning_start < 10000) {
      command[1] = SGP41_SUBCMD_NOX_CONDITIONING;
      response_words = 1;
    } else {
      command[1] = SGP41_SUBCMD_MEASURE_RAW;
      response_words = 2;
    }
  }

  uint16_t rhticks = llround((uint16_t)((humidity * 65535) / 100));
  command[2] = rhticks >> 8;
  command[3] = rhticks & 0xFF;
  command[4] = generate_crc_(command + 2, 2);
  uint16_t tempticks = (uint16_t)(((temperature + 45) * 65535) / 175);
  command[5] = tempticks >> 8;
  command[6] = tempticks & 0xFF;
  command[7] = generate_crc_(command + 5, 2);

  if (this->write(command, 8) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGD(TAG, "write error");
    return false;
  }
  delay(measure_time_);
  uint16_t raw_data[2];
  raw_data[1] = 0;
  if (!this->read_data_(raw_data, response_words)) {
    this->status_set_warning();
    ESP_LOGD(TAG, "read_data_ error");
    return false;
  }
  voc_raw = raw_data[0];
  nox_raw = raw_data[1];  // either 0 or the measured NOx ticks
  return true;
}

uint8_t SGP4xComponent::generate_crc_(const uint8_t *data, uint8_t datalen) {
  // calculates 8-Bit checksum with given polynomial
  uint8_t crc = SGP40_CRC8_INIT;

  for (uint8_t i = 0; i < datalen; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ SGP40_CRC8_POLYNOMIAL;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void SGP4xComponent::update_gas_indices() {
  this->seconds_since_last_store_ += 1;

  if (!this->measure_gas_indices_(this->voc_index_, this->nox_index_)) {
    // Set values to UINT16_MAX to indicate failure
    this->voc_index_ = this->nox_index_ = UINT16_MAX;
    ESP_LOGE(TAG, "measure gas indices failed");
    return;
  }
  if (this->samples_read_ < this->samples_to_stabilize_) {
    this->samples_read_++;
    ESP_LOGD(TAG, "Sensor has not collected enough samples yet. (%d/%d) VOC index is: %u", this->samples_read_,
             this->samples_to_stabilize_, this->voc_index_);
    return;
  }
}

void SGP4xComponent::update() {
  if (this->samples_read_ < this->samples_to_stabilize_) {
    return;
  }
  if (this->nox_sensor_) {
    if (this->voc_index_ != UINT16_MAX) {
      this->status_clear_warning();
      this->voc_sensor_->publish_state(this->voc_index_);
    } else {
      this->status_set_warning();
    }
  }
  if (this->nox_sensor_) {
    if (this->nox_index_ != UINT16_MAX) {
      this->status_clear_warning();
      this->nox_sensor_->publish_state(this->nox_index_);
    } else {
      this->status_set_warning();
    }
  }
}

void SGP4xComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "SGP40:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  store_baseline: %d", this->store_baseline_);

  if (this->is_failed()) {
    switch (this->error_code_) {
      case COMMUNICATION_FAILED:
        ESP_LOGW(TAG, "Communication failed! Is the sensor connected?");
        break;
      default:
        ESP_LOGW(TAG, "Unknown setup error!");
        break;
    }
  } else {
    ESP_LOGCONFIG(TAG, "  Type: %s", sgp_type_ == SGP41 ? "SGP41" : "SPG40");
    ESP_LOGCONFIG(TAG, "  Serial number: %" PRIu64, this->serial_number_);
    ESP_LOGCONFIG(TAG, "  Minimum Samples: %f", GasIndexAlgorithm_INITIAL_BLACKOUT);
  }
  LOG_UPDATE_INTERVAL(this);

  if (this->humidity_sensor_ != nullptr && this->temperature_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Compensation:");
    LOG_SENSOR("    ", "Temperature Source:", this->temperature_sensor_);
    LOG_SENSOR("    ", "Humidity Source:", this->humidity_sensor_);
  } else {
    ESP_LOGCONFIG(TAG, "  Compensation: No source configured");
  }
}

bool SGP4xComponent::write_command_(uint16_t command) {
  // Warning ugly, trick the I2Ccomponent base by setting register to the first 8 bit.
  return this->write_byte(command >> 8, command & 0xFF);
}

uint8_t SGP4xComponent::sht_crc_(uint8_t data1, uint8_t data2) {
  uint8_t bit;
  uint8_t crc = 0xFF;

  crc ^= data1;
  for (bit = 8; bit > 0; --bit) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0x131;
    } else {
      crc = (crc << 1);
    }
  }

  crc ^= data2;
  for (bit = 8; bit > 0; --bit) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0x131;
    } else {
      crc = (crc << 1);
    }
  }

  return crc;
}

bool SGP4xComponent::read_data_(uint16_t *data, uint8_t len) {
  const uint8_t num_bytes = len * 3;
  std::vector<uint8_t> buf(num_bytes);

  if (this->read(buf.data(), num_bytes) != i2c::ERROR_OK) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    const uint8_t j = 3 * i;
    uint8_t crc = sht_crc_(buf[j], buf[j + 1]);
    if (crc != buf[j + 2]) {
      ESP_LOGE(TAG, "CRC8 Checksum invalid! 0x%02X != 0x%02X", buf[j + 2], crc);
      return false;
    }
    data[i] = (buf[j] << 8) | buf[j + 1];
  }

  return true;
}

}  // namespace sgp4x
}  // namespace esphome

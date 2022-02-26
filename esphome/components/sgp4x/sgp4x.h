#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/application.h"
#include "esphome/core/preferences.h"
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>

#include <cmath>

namespace esphome {
namespace sgp4x {

struct SGP4xBaselines {
  int32_t state0;
  int32_t state1;
} PACKED;  // NOLINT

enum SGP_TYPE { SGP40, SGP41 };
// commands and constants
static const uint8_t SGP40_FEATURESET = 0x0020;  ///< The required set for this library
static const uint8_t SGP41_FEATURESET = 0x0040;
static const uint8_t SGP40_CRC8_POLYNOMIAL = 0x31;  ///< Seed for SGP40's CRC polynomial
static const uint8_t SGP40_CRC8_INIT = 0xFF;        ///< Init value for CRC
static const uint8_t SGP40_WORD_LEN = 2;            ///< 2 bytes per word
// Commands

static const uint16_t SGP40_CMD_GET_SERIAL_ID = 0x3682;
static const uint16_t SGP40_CMD_GET_FEATURESET = 0x202f;
static const uint16_t SGP40_CMD_SELF_TEST = 0x280e;
static const uint8_t SGP40_SUBCMD_MEASURE_RAW = 0x0F;
static const uint8_t SGP41_SUBCMD_MEASURE_RAW = 0x19;
static const uint8_t SGP41_SUBCMD_NOX_CONDITIONING = 0x12;

// Shortest time interval of 3H for storing baseline values.
// Prevents wear of the flash because of too many write operations
const uint32_t SHORTEST_BASELINE_STORE_INTERVAL = 10800;
static const uint16_t SPG40_SELFTEST_TIME = 250;  // 250 ms for self test
static const uint16_t SPG41_SELFTEST_TIME = 320;  // 320 ms for self test
static const uint16_t SGP40_MEASURE_TIME = 30;
static const uint16_t SGP41_MEASURE_TIME = 55;
// Store anyway if the baseline difference exceeds the max storage diff value
const uint32_t MAXIMUM_STORAGE_DIFF = 50;

class SGP4xComponent;

/// This class implements support for the Sensirion sgp4x i2c GAS (VOC) sensors.
class SGP4xComponent : public PollingComponent, public sensor::Sensor, public i2c::I2CDevice {
 public:
  // SGP4xComponent()  {};
  void set_humidity_sensor(sensor::Sensor *humidity) { humidity_sensor_ = humidity; }
  void set_temperature_sensor(sensor::Sensor *temperature) { temperature_sensor_ = temperature; }

  void setup() override;
  void update() override;
  void update_gas_indices();
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void set_store_baseline(bool store_baseline) { store_baseline_ = store_baseline; }
  void set_voc_sensor(sensor::Sensor *voc_sensor) { voc_sensor_ = voc_sensor; }
  void set_nox_sensor(sensor::Sensor *nox_sensor) { nox_sensor_ = nox_sensor; }

 protected:
  /// Input sensor for humidity and temperature compensation.
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  bool write_command_(uint16_t command);
  bool read_data_(uint16_t *data, uint8_t len);
  int16_t sensirion_init_sensors_();
  int16_t sgp40_probe_();
  uint8_t sht_crc_(uint8_t data1, uint8_t data2);
  uint64_t serial_number_;
  uint16_t featureset_;
  bool measure_gas_indices_(int32_t &voc, int32_t &nox);
  uint8_t generate_crc_(const uint8_t *data, uint8_t datalen);
  bool measure_raw_(uint16_t &voc_raw, uint16_t &nox_raw);
  ESPPreferenceObject pref_;
  uint32_t seconds_since_last_store_;
  SGP4xBaselines voc_baselines_storage_;
  bool self_test_complete_;
  bool store_baseline_;
  VOCGasIndexAlgorithm voc_algorithm_;
  int32_t voc_state0_;
  int32_t voc_state1_;
  int32_t voc_index_ = 0;
  sensor::Sensor *voc_sensor_{nullptr};
  uint8_t samples_read_ = 0;
  uint8_t samples_to_stabilize_ = static_cast<int8_t>(GasIndexAlgorithm_INITIAL_BLACKOUT) * 2;
  uint16_t self_test_time_;
  uint16_t measure_time_;
  SGP_TYPE sgp_type_{SGP40};
  int32_t nox_index_ = 0;
  NOxGasIndexAlgorithm nox_algorithm_;
  sensor::Sensor *nox_sensor_{nullptr};
  /**
   * @brief Request the sensor to perform a self-test, returning the result
   *
   * @return true: success false:failure
   */
  void self_test_();
  enum ErrorCode {
    COMMUNICATION_FAILED,
    MEASUREMENT_INIT_FAILED,
    INVALID_ID,
    UNSUPPORTED_ID,
    UNKNOWN
  } error_code_{UNKNOWN};
};
}  // namespace sgp4x
}  // namespace esphome

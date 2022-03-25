#pragma once
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace i2c_sensirion {

/**
 * Implementation of a i2c functions for Sensirion sensors
 * Sensirion data requires crc checking.
 * Each 16 bit word is/must be followed 8 bit CRC code
 * (Applies to read and write - note the i2c command code doesn't need a CRC)
 * Format:
 *   | 16 Bit Command Code | 16 bit Data word 1 | CRC of DW 1 | 16 bit Data word 1 | CRC of DW 2 | ..
 */
class SensirionI2CDevice : public i2c::I2CDevice {
 public:
  /** Read data words from i2c device.
   * handles crc check used by Sensirion sensors
   * @param data pointer to raw result
   * @param len number of words to read
   * @return true if reading succeded
   */
  bool read_data(uint16_t *data, uint8_t len);

  /** Read 1 data word from i2c device.
   * @param data reference to raw result
   * @return true if reading succeded
   */
  bool read_data(uint16_t &data) { return this->read_data(&data, 1); }

  /** get data words from i2c register.
   * handles crc check used by Sensirion sensors
   * @param  i2c register
   * @param data pointer to raw result
   * @param len number of words to read
   * @param delay milliseconds to to wait between sending the i2c commmand and reading the result
   * @return true if reading succeded
   */
  bool get_register(uint16_t i2c_register, uint16_t *data, uint8_t len, uint8_t delay);

  /** Read 1 data word from i2c register.
   * @param  i2c register
   * @param data reference to raw result
   * @param delay milliseconds to to wait between sending the i2c commmand and reading the result
   * @return true if reading succeded
   */
  bool get_register(uint16_t i2c_register, uint16_t &data, uint8_t delay) {
    return this->get_register(i2c_register, &data, 1, delay);
  }

  /** Write a command to the i2c device.
   * @param command i2c command to send
   * @return true if reading succeded
   */
  template<class T> bool write_command(T command) { return write_command(command, nullptr, 0); }

  /** Write a command and one data word to the i2c device .
   * @param command i2c command to send
   * @param data argument for the i2c command
   * @return true if reading succeded
   */
  template<class T> bool write_command(T command, uint16_t data) { return write_command(command, &data, 1); }

  /** Write a command with arguments as words
   * @param command i2c command to send - an be uint8_t or uint16_t
   * @param data vector<uint16> arguments for the i2c command
   * @return true if reading succeded
   */
  template<class T> bool write_command(T command, const std::vector<uint16_t> &data) {
    return write_command_(command, sizeof(T), data.data, data.size());
  }

  /** Write a command with arguments as words
   * @param command i2c command to send - an be uint8_t or uint16_t
   * @param data arguments for the i2c command
   * @param len number of arguments (words)
   * @return true if reading succeded
   */
  template<class T> bool write_command(T command, const uint16_t *data, uint8_t len) {
    // limit to 8 or 16 bit only
    static_assert(sizeof(command) == 1 || sizeof(command) == 2, "only 8 or 16 bit command types are supported.");
    return write_command_(command, sizeof(T), data, len);
  }

 protected:
  uint8_t crc_polynomial_{0x31u};  // default for sensirion
  /** Write a command with arguments as words
   * @param command i2c command to send can be uint8_t or uint16_t
   * @param command_len either 1 for short 8 bit command or 2 for 16 bit command codes
   * @param data arguments for the i2c command
   * @param data_len number of arguments (words)
   * @return true if reading succeded
   */
  bool write_command_(uint16_t command, uint8_t command_len, const uint16_t *data, uint8_t data_len);

  /** 8-bit CRC checksum that is transmitted after each data word for read and write operation
   * @param command i2c command to send
   * @param data data word for which the crc8 checksum is calculated
   * @param len number of arguments (words)
   * @return 8 Bit CRC
   */
  uint8_t sht_crc_(uint16_t data);

  /** 8-bit CRC checksum that is transmitted after each data word for read and write operation
   * @param command i2c command to send
   * @param data1 high byte of data word
   * @param data2 low byte of data word
   * @return 8 Bit CRC
   */
  uint8_t sht_crc_(uint8_t data1, uint8_t data2) { return sht_crc_(encode_uint16(data1, data2)); }

  /** last error code from i2c operation
   */
  i2c::ErrorCode last_error_;
};

}  // namespace i2c_sensirion
}  // namespace esphome

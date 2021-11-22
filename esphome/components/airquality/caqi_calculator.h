#pragma once

#include "esphome/core/log.h"
#include "abstract_aqi_calculator.h"

namespace esphome {
namespace airquality {

class CAQICalculator : public AbstractAQICalculator {
 public:
  uint8_t calculate_index(uint16_t value, Pollutant pollutant_type) override {
    int grid_index = get_grid_index_(value, pollutant_type);
    int aqi_lo = index_grid_[grid_index][0];
    int aqi_hi = index_grid_[grid_index][1];
    int conc_lo = calculation_grids_[pollutant_type][grid_index][0];
    int conc_hi = calculation_grids_[pollutant_type][grid_index][1];
    return (value - conc_lo) * (aqi_hi - aqi_lo) / (conc_hi - conc_lo) + aqi_lo;
  }

 protected:
  static const int AMOUNT_OF_LEVELS = 5;

  int index_grid_[AMOUNT_OF_LEVELS][2] = {{0, 25}, {26, 50}, {51, 75}, {76, 100}, {101, 400}};

  int pm2_5_calculation_grid_[AMOUNT_OF_LEVELS][2] = {{0, 15}, {16, 30}, {31, 55}, {56, 110}, {111, 400}};

  int pm10_0_calculation_grid_[AMOUNT_OF_LEVELS][2] = {{0, 25}, {26, 50}, {51, 90}, {91, 180}, {181, 400}};

  // Note must be defined using the order of enum Pollutant
  const int calculation_grids_[(Pollutant::SO2 + 1)][AMOUNT_OF_LEVELS][2] = {
      // PM25
      {{0, 15}, {16, 30}, {31, 55}, {56, 110}, {111, 0x7FFF}},
      // PM 10
      {{0, 25}, {26, 50}, {51, 90}, {91, 180}, {181, 0x7FFF}},
      // No2
      {{0, 50}, {51, 100}, {101, 200}, {201, 400}, {400, 0x7FFF}},
      //  O3
      {{0, 60}, {61, 120}, {121, 180}, {181, 240}, {241, 0x7FFF}},
      // CO - 8h avg
      {{0, 5}, {6, 8}, {9, 10}, {11, 20}, {21, 0x7FFF}},
      // SO2 1h avg
      {{0, 50}, {51, 100}, {101, 350}, {350, 500}, {501, 0x7FFF}}};

  int get_grid_index_(uint16_t value, Pollutant pollutant_type) {
    for (int i = 0; i < AMOUNT_OF_LEVELS; i++) {
      if (value >= calculation_grids_[pollutant_type][i][0] && value <= calculation_grids_[pollutant_type][i][1]) {
        return i;
      }
    }
    return -1;
  }
};

}  // namespace airquality
}  // namespace esphome

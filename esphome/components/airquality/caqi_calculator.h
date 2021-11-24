#pragma once
#include "aqi_calculatorbase.h"

namespace esphome {
namespace airquality {

class CAQICalc : public CalcGrid<5> {
 public:
  uint16_t calculate_index(uint16_t value, Pollutant pollutant_type) override {
    return calculate_index_(value, pollutant_type, index_grid_, calculation_grids_);
  }

 protected:
  static const constexpr int index_grid_[get_num_levels()][2] = {{0, 25}, {26, 50}, {51, 75}, {76, 100}, {101, 600}};

  // Note must be defined using the order of enum Pollutant
  // Based on revisised CAQI  - City Background hourly
  static const constexpr int calculation_grids_[(Pollutant::SO2 + 1)][get_num_levels()][2] = {
      // PM25
      {{0, 15}, {16, 30}, {31, 55}, {56, 110}, {111, 600}},
      // PM 10
      {{0, 25}, {26, 50}, {51, 90}, {91, 180}, {181, 600}},
      // No2
      {{0, 50}, {51, 100}, {101, 200}, {201, 400}, {400, 600}},
      //  O3
      {{0, 60}, {61, 120}, {121, 180}, {181, 240}, {241, 600}},
      // CO
      {{0, 5000}, {5001, 7500}, {75001, 10000}, {10001, 20000}, {20000, 30000}},
      // SO2 1h avg
      {{0, 50}, {51, 100}, {101, 350}, {350, 500}, {501, 600}}};
};
}  // namespace airquality
}  // namespace esphome

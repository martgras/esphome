#pragma once

#include <map>
#include "aqi_calculatorbase.h"

namespace esphome {
namespace airquality {

class AQICalc : public CalcGrid<7> {
 public:
  uint16_t calculate_index(uint16_t value, Pollutant pollutant_type) override {
    return calculate_index_(value, pollutant_type, index_grid_, calculation_grids_);
  }
  // protected:
  static const int AMOUNT_OF_LEVELS = get_num_levels();
  static const constexpr int index_grid_[AMOUNT_OF_LEVELS][2] = {{0, 51},    {51, 100},  {101, 150}, {151, 200},
                                                                 {201, 300}, {301, 400}, {401, 500}};
  // Note must be defined using the order of enum Pollutant
  static const constexpr int calculation_grids_[(Pollutant::SO2 + 1)][AMOUNT_OF_LEVELS][2] = {
      // pm2_5 24h avg
      {{0, 12}, {13, 35}, {36, 55}, {56, 150}, {151, 250}, {251, 350}, {351, 500}},
      // PM 10 24h avg
      {{0, 54}, {55, 154}, {155, 254}, {255, 354}, {355, 424}, {425, 504}, {425, 604}},
      // No2 1h avr
      {{0, 53}, {54, 100}, {101, 360}, {361, 649}, {650, 1249}, {1250, 1649}, {1650, 2049}},
      //  o3  not exaclty correct AQI uses 8h and 1h avg for
      {{0, 54}, {55, 124}, {125, 164}, {165, 204}, {205, 404}, {405, 504}, {505, 604}},
      // CO - 8h avg
      {{0, 4}, {5, 9}, {10, 12}, {13, 14}, {15, 30}, {31, 40}, {41, 50}},
      // SO2 1h avg
      {{0, 35}, {36, 75}, {76, 185}, {186, 304}, {305, 604}, {605, 804}, {805, 1004}}};
};

}  // namespace airquality
}  // namespace esphome

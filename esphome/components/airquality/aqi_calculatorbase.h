#pragma once
#include <cstdint>

namespace esphome {
namespace airquality {
// insert any new Pollutant before LAST_DO_NOT_USE_
enum Pollutant : uint8_t { PM25 = 0, PM10, NO2, O3, CO, SO2, LAST_DO_NOT_USE_ };
enum AQICalculatorType : uint8_t { AQI_TYPE = 0, CAQI_TYPE };

/// Interface base class 
class AQICalculatorBase {
 public:
 /// AQI style index based on value and polluntant
  virtual uint16_t calculate_index(uint16_t value, Pollutant pollutant_type) = 0;
};

/// Base class for AQI Stype grids
/// performs all the mapping
/// derived classes only have to provide the grids
/// AMOUNT_OF_LEVELS: number of index levels (AQI == 7 , CAQI == 5)
template<std::size_t AMOUNT_OF_LEVELS> class CalcGrid : public AQICalculatorBase {
 public:
  constexpr static std::size_t get_num_levels() { return AMOUNT_OF_LEVELS; }
 protected:
  /// get the index of the range the value is in
  static int get_grid_index_(int16_t value, const int grid[AMOUNT_OF_LEVELS][2]) {
    for (int i = 0; i < AMOUNT_OF_LEVELS; i++) {
      if (value >= grid[i][0] && value <= grid[i][1]) {
        return i;
      }
    }
    return AMOUNT_OF_LEVELS;
  }

  /// perform the AQI sytle mapping of value to the AQI style index using the grids provided by derived class
  static uint16_t calculate_index_(uint16_t value, Pollutant pollutant_type, const int index_grid[AMOUNT_OF_LEVELS][2],
                                   const int calculation_grids[(Pollutant::SO2 + 1)][AMOUNT_OF_LEVELS][2]) {
    int grid_index = get_grid_index_(value, calculation_grids[pollutant_type]);
    // max level eceeded
    if (grid_index == AMOUNT_OF_LEVELS)
      return index_grid[AMOUNT_OF_LEVELS - 1][1];
    int aqi_lo = index_grid[grid_index][0];
    int aqi_hi = index_grid[grid_index][1];
    int conc_lo = calculation_grids[pollutant_type][grid_index][0];
    int conc_hi = calculation_grids[pollutant_type][grid_index][1];
    return (value - conc_lo) * (aqi_hi - aqi_lo) / (conc_hi - conc_lo) + aqi_lo;
  }
};

}  // namespace airquality
}  // namespace esphome

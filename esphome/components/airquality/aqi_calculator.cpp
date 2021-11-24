#include "aqi_calculator.h"

namespace esphome {
namespace airquality {

const constexpr int AQICalc::index_grid_[AQICalc::AMOUNT_OF_LEVELS][2];
const constexpr int AQICalc::calculation_grids_[(Pollutant::SO2 + 1)][AQICalc::AMOUNT_OF_LEVELS][2];

}  // namespace airquality
}  // namespace esphome

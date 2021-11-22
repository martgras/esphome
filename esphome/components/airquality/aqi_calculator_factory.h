#pragma once

#include <memory>
#include "caqi_calculator.h"
#include "aqi_calculator.h"

namespace esphome {
namespace airquality {

enum AQICalculatorType { CAQI_TYPE = 0, AQI_TYPE = 1 };

class AQICalculatorFactory {
 public:
  AbstractAQICalculator *get_calculator(AQICalculatorType type) {
    if (type == CAQI_TYPE) {
      return caqi_calculator_.get();
    } else if (type == AQI_TYPE) {
      return aqi_calculator_.get();
    }
    return nullptr;
  }

 protected:
  std::unique_ptr<AbstractAQICalculator> caqi_calculator_ = make_unique<CAQICalculator>();
  std::unique_ptr<AbstractAQICalculator> aqi_calculator_ = make_unique<AQICalculator>();
};

}  // namespace airquality
}  // namespace esphome

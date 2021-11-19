import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_PM_2_5,
    CONF_PM_10_0,
    CONF_PM_1_0,
    STATE_CLASS_MEASUREMENT,
    ICON_CHEMICAL_WEAPON,
)


airquality_ns = cg.esphome_ns.namespace("airquality")
AirQualityComponent = airquality_ns.class_("AirQualityComponent", cg.PollingComponent)
AQICalculatorType = airquality_ns.enum("AQICalculatorType")


CONF_AQI = "aqi"
CONF_CAQI = "caqi"
CONF_CALCULATION_TYPE = "calculation_type"
UNIT_INDEX = "index"

AQI_CALCULATION_TYPE = {
    "CAQI": AQICalculatorType.CAQI_TYPE,
    "AQI": AQICalculatorType.AQI_TYPE,
}


def _validate(config):
    if CONF_AQI in config and CONF_PM_2_5 not in config:
        raise cv.Invalid("AQI sensor requires PM 2.5")
    if CONF_AQI in config and CONF_PM_10_0 not in config:
        raise cv.Invalid("AQI sensor requires PM 10 sensors")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AirQualityComponent),
            cv.Optional(CONF_PM_1_0): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_PM_2_5): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_PM_10_0): cv.use_id(sensor.Sensor),
            #  VOC and CO2 TBD
            #            cv.Optional(CONF_VOC): cv.use_id(
            #                sensor.Sensor
            #            ),
            #            cv.Optional(CONF_CO2): cv.use_id(
            #                sensor.Sensor
            #            ),
            cv.Optional(CONF_AQI): sensor.sensor_schema(
                unit_of_measurement=UNIT_INDEX,
                icon=ICON_CHEMICAL_WEAPON,
                accuracy_decimals=0,
                # device_class=DEVICE_CLASS_AQI,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CAQI): sensor.sensor_schema(
                unit_of_measurement=UNIT_INDEX,
                icon=ICON_CHEMICAL_WEAPON,
                accuracy_decimals=0,
                # device_class=DEVICE_CLASS_AQI,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    ).extend(cv.polling_component_schema("60s")),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_PM_1_0 in config:
        sens = await cg.get_variable(config[CONF_PM_1_0])
        cg.add(var.set_pm_1_0_sensor(sens))

    if CONF_PM_2_5 in config:
        sens = await cg.get_variable(config[CONF_PM_1_0])
        cg.add(var.set_pm_2_5_sensor(sens))

    if CONF_PM_10_0 in config:
        sens = await cg.get_variable(config[CONF_PM_1_0])
        cg.add(var.set_pm_10_0_sensor(sens))

    if CONF_AQI in config:
        sens = await sensor.new_sensor(config[CONF_AQI])
        cg.add(var.set_aqi_sensor(sens))
    if CONF_AQI in config:
        sens = await sensor.new_sensor(config[CONF_CAQI])
        cg.add(var.set_caqi_sensor(sens))

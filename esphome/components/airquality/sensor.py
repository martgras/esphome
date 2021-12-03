import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    ICON_CHEMICAL_WEAPON,
)

airquality_ns = cg.esphome_ns.namespace("airquality")
AirQualityComponent = airquality_ns.class_("AirQualityComponent", cg.PollingComponent)

CONF_AQI = "aqi"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AirQualityComponent),
            cv.Required(CONF_AQI): sensor.sensor_schema(
                icon=ICON_CHEMICAL_WEAPON,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    ).extend(cv.polling_component_schema("60s")),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    sens = await sensor.new_sensor(config[CONF_AQI])
    cg.add(var.set_aqi_sensor(sens))

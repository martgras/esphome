import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    ICON_RADIATOR,
    DEVICE_CLASS_NITROUS_OXIDE,
    DEVICE_CLASS_VOLATILE_ORGANIC_COMPOUNDS,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["i2c"]

CODEOWNERS = ["@SenexCrenshaw", "@martgras"]

sgp4x_ns = cg.esphome_ns.namespace("sgp4x")
SGP4xComponent = sgp4x_ns.class_(
    "SGP4xComponent", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

CONF_COMPENSATION = "compensation"
CONF_HUMIDITY_SOURCE = "humidity_source"
CONF_NOX = "nox"
CONF_VOC = "voc"
CONF_STORE_BASELINE = "store_baseline"
CONF_TEMPERATURE_SOURCE = "temperature_source"
CONF_VOC_BASELINE = "voc_baseline"


def validate_sensors(config):
    if CONF_VOC not in config and CONF_NOX not in config:
        raise cv.Invalid(
            f"At least one sensor is required. Define {CONF_VOC} and/or {CONF_NOX}"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SGP4xComponent),
            cv.Optional(CONF_VOC): sensor.sensor_schema(
                icon=ICON_RADIATOR,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_VOLATILE_ORGANIC_COMPOUNDS,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_NOX): sensor.sensor_schema(
                icon=ICON_RADIATOR,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_NITROUS_OXIDE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_STORE_BASELINE, default=True): cv.boolean,
            cv.Optional(CONF_VOC_BASELINE): cv.hex_uint16_t,
            cv.Optional(CONF_COMPENSATION): cv.Schema(
                {
                    cv.Required(CONF_HUMIDITY_SOURCE): cv.use_id(sensor.Sensor),
                    cv.Required(CONF_TEMPERATURE_SOURCE): cv.use_id(sensor.Sensor),
                },
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x59)),
    #     cv.has_at_least_one_key(CONF_VOC, CONF_NOX)
    validate_sensors,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_COMPENSATION in config:
        compensation_config = config[CONF_COMPENSATION]
        sens = await cg.get_variable(compensation_config[CONF_HUMIDITY_SOURCE])
        cg.add(var.set_humidity_sensor(sens))
        sens = await cg.get_variable(compensation_config[CONF_TEMPERATURE_SOURCE])
        cg.add(var.set_temperature_sensor(sens))

    cg.add(var.set_store_baseline(config[CONF_STORE_BASELINE]))

    if CONF_VOC_BASELINE in config:
        cg.add(var.set_voc_baseline(CONF_VOC_BASELINE))

    if CONF_VOC in config:
        sens = await sensor.new_sensor(config[CONF_VOC])
        cg.add(var.set_voc_sensor(sens))

    if CONF_NOX in config:
        sens = await sensor.new_sensor(config[CONF_NOX])
        cg.add(var.set_nox_sensor(sens))

    #    cg.add_library("sensirion/Sensirion Gas Index Algorithm", "^3.1.0")
    cg.add_library("https://github.com/martgras/arduino-gas-index-algorithm.git", "")

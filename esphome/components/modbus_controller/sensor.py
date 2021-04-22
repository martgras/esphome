from esphome.components import sensor
import esphome.config_validation as cv
import esphome.codegen as cg

from esphome.const import CONF_ID, CONF_ADDRESS, CONF_OFFSET
from . import (
    modbus_controller_ns,
    MODBUS_FUNCTION_CODE,
    CONF_BITMASK,
    CONF_VALUE_TYPE,
    CONF_REGISTER_COUNT,
    SENSOR_VALUE_TYPE,
)
from .const import (
    CONF_MODBUSCOMPONENT_ID,
    CONF_MODBUS_FUNCTIONCODE,
    CONF_SKIP_UPDATES,
)

DEPENDENCIES = ["modbus_controller"]
CODEOWNERS = ["@martgras"]


ModbusSensor = modbus_controller_ns.class_("ModbusSensor", sensor.Sensor, cg.Component)

CONFIG_SCHEMA = sensor.SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ModbusSensor),
        cv.Required(CONF_MODBUSCOMPONENT_ID): cv.use_id("Modbus"),
        cv.Optional(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Optional(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET): cv.int_,
        cv.Optional(CONF_BITMASK, default=0xFFFFFFFF): cv.hex_uint32_t,
        cv.Optional(CONF_VALUE_TYPE): cv.enum(SENSOR_VALUE_TYPE),
        cv.Optional(CONF_REGISTER_COUNT, default=1): cv.int_,
        cv.Optional(CONF_SKIP_UPDATES, default=0): cv.int_,
    }
).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_MODBUS_FUNCTIONCODE],
        config[CONF_ADDRESS],
        config[CONF_OFFSET],
        config[CONF_BITMASK],
        config[CONF_VALUE_TYPE],
        config[CONF_REGISTER_COUNT],
        config[CONF_SKIP_UPDATES],
    )
    yield cg.register_component(var, config)
    yield sensor.register_sensor(var, config)

    paren = yield cg.get_variable(config["modbuscomponent_id"])
    cg.add(var.set_modbus_parent(paren))

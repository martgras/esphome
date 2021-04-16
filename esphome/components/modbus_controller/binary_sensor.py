from esphome.components import binary_sensor
import esphome.config_validation as cv
import esphome.codegen as cg

from esphome.cpp_types import App
from esphome.const import CONF_ID, CONF_ADDRESS, CONF_OFFSET
from . import (
    modbus_controller_ns,
    MODBUS_FUNCTION_CODE,
    CONF_BITMASK,
    CONF_CREATE_SWITCH,
)
from .const import (
    CONF_MODBUSCOMPONENT_ID,
    CONF_MODBUS_FUNCTIONCODE,
    CONF_SKIP_UPDATES,
)

DEPENDENCIES = ["modbus_controller"]
CODEOWNERS = ["@martgras"]


ModbusBinarySensor = modbus_controller_ns.class_(
    "ModbusBinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ModbusBinarySensor),
        cv.Required(CONF_MODBUSCOMPONENT_ID): cv.use_id("Modbus"),
        cv.Optional(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Optional(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET): cv.int_,
        cv.Optional(CONF_BITMASK, default=0x1): cv.hex_uint32_t,
        cv.Optional(CONF_SKIP_UPDATES, default=0): cv.int_,
        cv.Optional(CONF_CREATE_SWITCH, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        App,
        config[CONF_MODBUS_FUNCTIONCODE],
        config[CONF_ADDRESS],
        config[CONF_OFFSET],
        config[CONF_BITMASK],
        config[CONF_CREATE_SWITCH],
        config[CONF_SKIP_UPDATES],
    )
    yield cg.register_component(var, config)
    yield binary_sensor.register_binary_sensor(var, config)

    paren = yield cg.get_variable(config["modbuscomponent_id"])
    cg.add(var.set_modbus_parent(paren))

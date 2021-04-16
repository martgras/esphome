from esphome.components import text_sensor
import esphome.config_validation as cv
import esphome.codegen as cg


from esphome.const import CONF_ID, CONF_ADDRESS, CONF_OFFSET
from . import modbus_controller_ns, MODBUS_FUNCTION_CODE
from .const import (
    CONF_MODBUSCOMPONENT_ID,
    CONF_MODBUS_FUNCTIONCODE,
    CONF_REGISTER_COUNT,
    CONF_RESPONSE_SIZE,
    CONF_HEX_ENCODE,
    CONF_SKIP_UPDATES,
)

DEPENDENCIES = ["modbus_controller"]
CODEOWNERS = ["@martgras"]


ModbusTextSensor = modbus_controller_ns.class_(
    "ModbusTextSensor", text_sensor.TextSensor, cg.Component
)

CONFIG_SCHEMA = text_sensor.TEXT_SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ModbusTextSensor),
        cv.Required(CONF_MODBUSCOMPONENT_ID): cv.use_id("Modbus"),
        cv.Optional(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Optional(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET): cv.int_,
        cv.Optional(CONF_REGISTER_COUNT, default=1): cv.int_,
        cv.Optional(CONF_RESPONSE_SIZE, default=0): cv.int_,
        cv.Optional(CONF_HEX_ENCODE, default="true"): cv.boolean,
        cv.Optional(CONF_SKIP_UPDATES, default=0): cv.int_,
    }
).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_MODBUS_FUNCTIONCODE],
        config[CONF_ADDRESS],
        config[CONF_OFFSET],
        config[CONF_REGISTER_COUNT],
        config[CONF_RESPONSE_SIZE],
        config[CONF_HEX_ENCODE],
        config[CONF_SKIP_UPDATES],
    )
    yield cg.register_component(var, config)
    yield text_sensor.register_text_sensor(var, config)

    paren = yield cg.get_variable(config["modbuscomponent_id"])
    cg.add(var.set_modbus_parent(paren))

from esphome.components import select
import esphome.config_validation as cv
import esphome.codegen as cg


from esphome.const import CONF_ADDRESS, CONF_ID
from .. import (
    add_modbus_base_properties,
    modbus_controller_ns,
    modbus_calc_properties,
    validate_modbus_register,
    ModbusItemBaseSchema,
    SensorItem,
    MODBUS_REGISTER_TYPE,
    SENSOR_VALUE_TYPE,
)
from ..const import (
    CONF_BITMASK,
    CONF_FORCE_NEW_RANGE,
    CONF_MODBUS_CONTROLLER_ID,
    CONF_REGISTER_COUNT,
    CONF_RESPONSE_SIZE,
    CONF_SKIP_UPDATES,
    CONF_REGISTER_TYPE,
    CONF_VALUE_TYPE,
)

DEPENDENCIES = ["modbus_controller"]
CODEOWNERS = ["@stegm"]


ModbusSelect = modbus_controller_ns.class_(
    "ModbusSelect", cg.Component, select.Select, SensorItem
)


CONFIG_SCHEMA = cv.All(
    select.SELECT_SCHEMA.extend(cv.COMPONENT_SCHEMA)
    .extend(ModbusItemBaseSchema)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(ModbusSelect),
            cv.Optional(CONF_REGISTER_TYPE): cv.enum(MODBUS_REGISTER_TYPE),
            cv.Optional(CONF_REGISTER_COUNT, default=0): cv.positive_int,
            cv.Optional(CONF_RESPONSE_SIZE, default=2): cv.positive_int,
            cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(SENSOR_VALUE_TYPE),
        }
    ),
    validate_modbus_register,
)


async def to_code(config):
    byte_offset, reg_count = modbus_calc_properties(config)
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_REGISTER_TYPE],
        config[CONF_ADDRESS],
        byte_offset,
        config[CONF_BITMASK],
        config[CONF_VALUE_TYPE],
        reg_count,
        config[CONF_RESPONSE_SIZE],
        config[CONF_SKIP_UPDATES],
        config[CONF_FORCE_NEW_RANGE],
    )

    await cg.register_component(var, config)
    await select.register_select(var, config, options=[])  # TODO

    paren = await cg.get_variable(config[CONF_MODBUS_CONTROLLER_ID])
    cg.add(paren.add_sensor_item(var))
    await add_modbus_base_properties(
        var, config, ModbusSelect, cg.std_string, cg.std_string
    )

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import (
    sensor as core_sensor,
    uart,
    binary_sensor as core_binary_sensor,
    text_sensor as core_text_sensor,
    switch as core_switch,
)
from esphome.core import coroutine
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_OFFSET,
    CONF_NAME,
)
from .const import (
    CONF_CTRL_PIN,
    CONF_VALUE_TYPE,
    CONF_REGISTER_COUNT,
    CONF_MODBUS_FUNCTIONCODE,
    CONF_COMMAND_THROTTLE,
    CONF_RESPONSE_SIZE,
    CONF_BITMASK,
    CONF_SKIP_UPDATES,
    CONF_RAW_ENCODE,
)

CODEOWNERS = ["@martgras"]

AUTO_LOAD = [
    "uart",
    "sensor",
    "binary_sensor",
    "text_sensor",
    "status",
    "switch",
]


# pylint: disable=invalid-name
modbus_controller_ns = cg.esphome_ns.namespace("modbus_controller")
ModbusController = modbus_controller_ns.class_(
    "ModbusController", cg.PollingComponent, uart.UARTDevice
)

ModbusSwitch = modbus_controller_ns.class_(
    "ModbusSwitch", core_switch.Switch, cg.Component
)

ModbusTextSensor = modbus_controller_ns.class_("ModbusTextSensor", cg.Nameable)
ModbusSensor = modbus_controller_ns.class_(
    "ModbusSensor", core_sensor.Sensor, cg.Component
)

ModbusBinarySensor = modbus_controller_ns.class_(
    "ModbusBinarySensor", core_binary_sensor.BinarySensor, cg.Component
)

ModbusFunctionCode_ns = cg.esphome_ns.namespace("modbus_controller::ModbusFunctionCode")
ModbusFunctionCode = ModbusFunctionCode_ns.enum("ModbusFunctionCode")
MODBUS_FUNCTION_CODE = {
    "read_coils": ModbusFunctionCode.READ_COILS,
    "read_discrete_inputs": ModbusFunctionCode.READ_DISCRETE_INPUTS,
    "read_holding_registers": ModbusFunctionCode.READ_HOLDING_REGISTERS,
    "read_input_registers": ModbusFunctionCode.READ_INPUT_REGISTERS,
    "write_single_coil": ModbusFunctionCode.WRITE_SINGLE_COIL,
    "write_single_register": ModbusFunctionCode.WRITE_SINGLE_REGISTER,
    "write_multiple_coils": ModbusFunctionCode.WRITE_MULTIPLE_COILS,
    "write_multiple_registers": ModbusFunctionCode.WRITE_MULTIPLE_REGISTERS,
}

SensorValueType_ns = cg.esphome_ns.namespace("modbus_controller::SensorValueType")
SensorValueType = SensorValueType_ns.enum("SensorValueType")
SENSOR_VALUE_TYPE = {
    "RAW": SensorValueType.RAW,
    "U_WORD": SensorValueType.U_WORD,
    "S_WORD": SensorValueType.S_WORD,
    "U_DWORD": SensorValueType.U_DWORD,
    "U_DWORD_R": SensorValueType.U_DWORD_R,
    "S_DWORD": SensorValueType.S_DWORD,
    "S_DWORD_R": SensorValueType.S_DWORD_R,
    "U_QWORD": SensorValueType.U_QWORD,
    "U_QWORDU_R": SensorValueType.U_QWORD_R,
    "S_QWORD": SensorValueType.S_QWORD,
    "U_QWORD_R": SensorValueType.S_QWORD_R,
}

RawEncodingType = cg.esphome_ns.namespace("modbus_controller::RawEncoding")
RAW_ENCODING = {
    "NONE": RawEncodingType.NONE,
    "HEXBYTES": RawEncodingType.HEXBYTES,
    "COMMA": RawEncodingType.COMMA,
}


sensor_entry = core_sensor.SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ModbusSensor),
        cv.Required(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Required(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET, default=0): cv.int_,
        cv.Optional(CONF_BITMASK, default=0xFFFFFFFF): cv.hex_uint32_t,
        cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(SENSOR_VALUE_TYPE),
        cv.Optional(CONF_REGISTER_COUNT, default=1): cv.int_,
        cv.Optional(CONF_SKIP_UPDATES, default=0): cv.int_,
    }
)

binary_sensor_entry = core_binary_sensor.BINARY_SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ModbusBinarySensor),
        cv.Required(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Required(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET, default=0): cv.int_,
        cv.Optional(CONF_BITMASK, default=0x1): cv.hex_uint32_t,
        cv.Optional(CONF_SKIP_UPDATES, default=0): cv.int_,
    }
)

switch_entry = core_switch.SWITCH_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ModbusSwitch),
        cv.Required(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Required(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET, default=0): cv.int_,
        cv.Optional(CONF_BITMASK, default=0x1): cv.hex_uint32_t,
    }
).extend(cv.COMPONENT_SCHEMA)

text_sensor_entry = core_text_sensor.TEXT_SENSOR_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ModbusTextSensor),
        cv.Required(CONF_MODBUS_FUNCTIONCODE): cv.enum(MODBUS_FUNCTION_CODE),
        cv.Required(CONF_ADDRESS): cv.int_,
        cv.Optional(CONF_OFFSET, default=0): cv.int_,
        cv.Optional(CONF_REGISTER_COUNT, default=1): cv.int_,
        cv.Optional(CONF_RESPONSE_SIZE, default=0): cv.int_,
        cv.Optional(CONF_RAW_ENCODE, default="NONE"): cv.enum(RAW_ENCODING),
        cv.Optional(CONF_SKIP_UPDATES, default=0): cv.int_,
    }
).extend(cv.COMPONENT_SCHEMA)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModbusController),
            cv.Optional(CONF_CTRL_PIN): pins.output_pin,
            cv.Optional(CONF_ADDRESS, default=0x1): cv.hex_uint8_t,
            cv.Optional(
                CONF_COMMAND_THROTTLE, default="0ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional("sensors"): cv.All(
                cv.ensure_list(sensor_entry), cv.Length(min=0)
            ),
            cv.Optional("binary_sensors"): cv.All(
                cv.ensure_list(binary_sensor_entry), cv.Length(min=0)
            ),
            cv.Optional("text_sensors"): cv.All(
                cv.ensure_list(text_sensor_entry), cv.Length(min=0)
            ),
            cv.Optional("switches"): cv.All(
                cv.ensure_list(switch_entry), cv.Length(min=0)
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_COMMAND_THROTTLE])
    yield cg.add(var.set_command_throttle(config[CONF_COMMAND_THROTTLE]))
    yield cg.register_component(var, config)
    yield register_modbus_device(var, config)
    if config.get("sensors"):
        conf = config["sensors"]
        for cfg in conf:
            sens = yield new_sensor(cfg)
            cg.add(
                sens.add_to_controller(
                    var,
                    cfg[CONF_MODBUS_FUNCTIONCODE],
                    cfg[CONF_ADDRESS],
                    cfg[CONF_OFFSET],
                    cfg[CONF_BITMASK],
                    cfg[CONF_VALUE_TYPE],
                    cfg[CONF_REGISTER_COUNT],
                    cfg[CONF_SKIP_UPDATES],
                )
            )
    if config.get("binary_sensors"):
        conf = config["binary_sensors"]
        for cfg in conf:
            sens = yield new_binary_sensor(cfg)
            cg.add(
                sens.add_to_controller(
                    var,
                    cfg[CONF_MODBUS_FUNCTIONCODE],
                    cfg[CONF_ADDRESS],
                    cfg[CONF_OFFSET],
                    cfg[CONF_BITMASK],
                    cfg[CONF_SKIP_UPDATES],
                )
            )
    if config.get("text_sensors"):
        conf = config["text_sensors"]
        for cfg in conf:
            sens = yield new_text_sensor(cfg)
            cg.add(
                sens.add_to_controller(
                    var,
                    cfg[CONF_MODBUS_FUNCTIONCODE],
                    cfg[CONF_ADDRESS],
                    cfg[CONF_OFFSET],
                    cfg[CONF_REGISTER_COUNT],
                    cfg[CONF_RESPONSE_SIZE],
                    cfg[CONF_RAW_ENCODE],
                    cfg[CONF_SKIP_UPDATES],
                )
            )
    if config.get("switches"):
        conf = config["switches"]
        for cfg in conf:
            sens = yield new_modbus_switch(cfg)
            cg.add(
                sens.add_to_controller(
                    var,
                    cfg[CONF_MODBUS_FUNCTIONCODE],
                    cfg[CONF_ADDRESS],
                    cfg[CONF_OFFSET],
                    cfg[CONF_BITMASK],
                )
            )


@coroutine
def new_sensor(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_NAME])
    yield cg.register_component(var, config)
    yield core_sensor.register_sensor(var, config)
    yield var


@coroutine
def new_binary_sensor(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_NAME])
    yield cg.register_component(var, config)
    yield core_binary_sensor.register_binary_sensor(var, config)
    yield var


@coroutine
def new_text_sensor(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_NAME])
    yield core_text_sensor.register_text_sensor(var, config)
    yield var


@coroutine
def new_modbus_switch(config):

    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_MODBUS_FUNCTIONCODE],
        config[CONF_ADDRESS],
        config[CONF_OFFSET],
        config[CONF_BITMASK],
    )
    yield cg.register_component(var, config)
    yield core_switch.register_switch(var, config)
    yield var


@coroutine
def register_modbus_device(var, config):
    cg.add(var.set_address(config[CONF_ADDRESS]))
    if CONF_CTRL_PIN in config:
        cg.add(var.set_ctrl_pin(config[CONF_CTRL_PIN]))
    yield uart.register_uart_device(var, config)

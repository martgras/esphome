import esphome.codegen as cg
from esphome import automation
from esphome.components import (
    modbus,
)

CODEOWNERS = ["@martgras"]
AUTO_LOAD = [
    "modbus",
    "binary_sensor",
    "text_sensor",
    "status",
    "switch",
    "modbus_component",
]


# pylint: disable=invalid-name
text_sensor_ns = cg.esphome_ns.namespace("text_sensor")
TextSensor = text_sensor_ns.class_("TextSensor", cg.Nameable)

modbus_component_ns = cg.esphome_ns.namespace("modbus_component")
ModbusComponent = modbus_component_ns.class_(
    "ModbusComponent", cg.PollingComponent, modbus.ModbusDevice
)
RawData = modbus_component_ns.struct("RawData")
RawDataCodeTrigger = modbus_component_ns.class_(
    "RawDataCodeTrigger", automation.Trigger.template(RawData)
)

ModbusFunctionCode_ns = cg.esphome_ns.namespace("modbus_component::ModbusFunctionCode")
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

SensorValueType_ns = cg.esphome_ns.namespace("modbus_component::SensorValueType")
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

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import (
    CONF_ID,
)


CODEOWNERS = ["@lorenzo-deluca"]
AUTO_LOAD = ["uart", "api", "binary_sensor"]
MULTI_CONF = True

CONF_BENTEL_KYO32_ID = "bentel_kyo32_id"


# pylint: disable=invalid-name
bentel_kyo32_ns = cg.esphome_ns.namespace("bentel_kyo32")
Bentel_Kyo32 = bentel_kyo32_ns.class_(
    "Bentel_Kyo32", cg.PollingComponent, uart.UARTDevice
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Bentel_Kyo32),
        }
    )
    .extend(cv.polling_component_schema("30s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await register_uart_device(var, config)


async def register_uart_device(var, config):
    await cg.register_component(var, config)
    return await uart.register_uart_device(var, config)

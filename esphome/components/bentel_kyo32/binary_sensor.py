import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_DEVICE_CLASS,
    CONF_NAME,
    DEVICE_CLASS_CONNECTIVITY,
)

from . import Bentel_Kyo32, CONF_BENTEL_KYO32_ID

CODEOWNERS = ["@lorenzo-deluca"]

CONF_KYO_COMUNICATION = "kyo_comunication"
CONF_ZONE = "zone"
CONF_ZONA = "zona"
CONF_SABOTAGE = "sabotage"

zone_entry = {
    cv.Optional(CONF_NAME): cv.string_strict,
    cv.Required(CONF_ZONE): cv.int_range(1, 32),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BENTEL_KYO32_ID): cv.use_id(Bentel_Kyo32),
        cv.Optional(CONF_KYO_COMUNICATION): binary_sensor.BINARY_SENSOR_SCHEMA.extend(
            {
                cv.Optional(
                    CONF_DEVICE_CLASS, default=DEVICE_CLASS_CONNECTIVITY
                ): binary_sensor.device_class,
            }
        ),
        # TODO add the remaining sensors like stato_sirena
        # define the various zones
        cv.Optional(CONF_ZONA): cv.All(cv.ensure_list(zone_entry), cv.Length(min=1)),
        cv.Optional(CONF_SABOTAGE): cv.All(
            cv.ensure_list(zone_entry), cv.Length(min=1)
        ),
    },
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BENTEL_KYO32_ID])

    if CONF_KYO_COMUNICATION in config:
        sensor = await binary_sensor.new_binary_sensor(config[CONF_KYO_COMUNICATION])
        await binary_sensor.register_binary_sensor(
            sensor, config[CONF_KYO_COMUNICATION]
        )
        cg.add(parent.set_communication_sensor(sensor))
    if CONF_ZONA in config:
        for zone in config[CONF_ZONA]:
            cg.add(parent.add_zona_sensor(zone[CONF_ZONE], zone[CONF_NAME]))

    # slightly different variant name is auto generated in code if not provided
    if CONF_SABOTAGE in config:
        for zone in config[CONF_SABOTAGE]:
            if CONF_NAME in zone:
                cg.add(parent.add_zona_sensor(zone[CONF_ZONE], zone[CONF_NAME]))
            else:
                cg.add(parent.add_zona_sensor(zone[CONF_ZONE]))

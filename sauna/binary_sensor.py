import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

from . import sauna_ns

DEPENDENCIES = ["binary_sensor"]

Sauna = sauna_ns.class_("Sauna", cg.Component)

CONF_RUNNING = "running"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(Sauna),
        cv.Optional(CONF_RUNNING): binary_sensor.binary_sensor_schema(),
    }
)

async def to_code(config):
    var = await cg.get_variable(config[CONF_ID])

    if CONF_RUNNING in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_RUNNING])
        cg.add(var.set_running_binary_sensor(bs))


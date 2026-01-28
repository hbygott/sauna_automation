import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from . import sauna_ns

DEPENDENCIES = ["sensor"]

sauna = sauna_ns.class_("Sauna", cg.Component)

CONF_CURRENT_TEMP = "current_temp"
CONF_SETPOINT_TEMP = "setpoint_temp"
CONF_TIMER = "timer"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(sauna),

        cv.Optional(CONF_CURRENT_TEMP): sensor.sensor_schema(),
        cv.Optional(CONF_SETPOINT_TEMP): sensor.sensor_schema(),
        cv.Optional(CONF_TIMER): sensor.sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_CURRENT_TEMP in config:
        s = await sensor.new_sensor(config[CONF_CURRENT_TEMP])
        cg.add(var.set_current_temp_sensor(s))

    if CONF_SETPOINT_TEMP in config:
        s = await sensor.new_sensor(config[CONF_SETPOINT_TEMP])
        cg.add(var.set_setpoint_temp_sensor(s))

    if CONF_TIMER in config:
        s = await sensor.new_sensor(config[CONF_TIMER])
        cg.add(var.set_timer_sensor(s))

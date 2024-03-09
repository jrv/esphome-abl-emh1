import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ICON, CONF_ID

from . import CONF_ABL_EMH1_ID, ABLeMH1

DEPENDENCIES = ["abl_emh1"]

CONF_MODE = "mode"
CONF_SERIAL_NUMBER = "serial_number"

ICON_MODE = "mdi:information"
ICON_SERIAL_NUMBER = "mdi:information"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ABL_EMH1_ID): cv.use_id(ABLeMH1),
        cv.Optional(CONF_MODE): text_sensor.TEXT_SENSOR_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(CONF_ICON, default=ICON_MODE): cv.icon,
            }
        ),
        cv.Optional(CONF_SERIAL_NUMBER): text_sensor.TEXT_SENSOR_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(CONF_ICON, default=ICON_SERIAL_NUMBER): cv.icon,
            }
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ABL_EMH1_ID])
    for key in [CONF_MODE, CONF_SERIAL_NUMBER]:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await text_sensor.register_text_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))

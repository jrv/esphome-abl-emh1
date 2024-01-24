import esphome.codegen as cg
from esphome.components import emh1_modbus
import esphome.config_validation as cv
from esphome.const import CONF_ID

AUTO_LOAD = ["emh1_modbus", "sensor", "text_sensor"]
CODEOWNERS = ["@jrv"]
MULTI_CONF = True

CONF_ABL_EMH1_ID = "abl_emh1_id"

abl_emh1_ns = cg.esphome_ns.namespace("abl_emh1")
ABLeMH1 = abl_emh1_ns.class_(
    "ABLeMH1", cg.PollingComponent, emh1_modbus.eMH1ModbusDevice
)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(ABLeMH1)})
    .extend(cv.polling_component_schema("30s"))
    .extend(
        emh1_modbus.emh1_modbus_device_schema(0x01)
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await emh1_modbus.register_emh1_modbus_device(var, config)

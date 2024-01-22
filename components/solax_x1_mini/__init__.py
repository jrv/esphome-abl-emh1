import esphome.codegen as cg
from esphome.components import emh1_modbus
import esphome.config_validation as cv
from esphome.const import CONF_ID

AUTO_LOAD = ["emh1_modbus", "sensor", "text_sensor"]
CODEOWNERS = ["@jrv"]
MULTI_CONF = True

CONF_SOLAX_X1_MINI_ID = "solax_x1_mini_id"

solax_x1_mini_ns = cg.esphome_ns.namespace("solax_x1_mini")
SolaxX1Mini = solax_x1_mini_ns.class_(
    "SolaxX1Mini", cg.PollingComponent, emh1_modbus.eMH1ModbusDevice
)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(SolaxX1Mini)})
    .extend(cv.polling_component_schema("30s"))
    .extend(
        emh1_modbus.emh1_modbus_device_schema(0x0A, "3132333435363737363534333231")
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await emh1_modbus.register_emh1_modbus_device(var, config)

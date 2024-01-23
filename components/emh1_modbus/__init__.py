from esphome import pins
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_FLOW_CONTROL_PIN, CONF_ID
from esphome.cpp_helpers import gpio_pin_expression

CODEOWNERS = ["@jrv"]

DEPENDENCIES = ["uart"]
MULTI_CONF = False

CONF_EMH1_MODBUS_ID = "emh1_modbus_id"
CONF_SERIAL_NUMBER = "serial_number"

emh1_modbus_ns = cg.esphome_ns.namespace("emh1_modbus")
eMH1Modbus = emh1_modbus_ns.class_("eMH1Modbus", cg.Component, uart.UARTDevice)
eMH1ModbusDevice = emh1_modbus_ns.class_("eMH1ModbusDevice")

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(eMH1Modbus),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


def validate_serial_number(value):
  return "serial number needs some work"

def as_hex_array(value):
    cpp_array = [
        f"0x{part}" for part in [value[i : i + 2] for i in range(0, len(value), 2)]
    ]
    return cg.RawExpression(f"(uint8_t*)(const uint8_t[16]){{{','.join(cpp_array)}}}")


async def to_code(config):
    cg.add_global(emh1_modbus_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    await uart.register_uart_device(var, config)

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))


def emh1_modbus_device_schema(default_address):
    schema = {
        cv.GenerateID(CONF_EMH1_MODBUS_ID): cv.use_id(eMH1Modbus),
    }
    if default_address is None:
        schema[cv.Required(CONF_ADDRESS)] = cv.hex_uint8_t
    else:
        schema[cv.Optional(CONF_ADDRESS, default=default_address)] = cv.hex_uint8_t
   return cv.Schema(schema)


async def register_emh1_modbus_device(var, config):
    parent = await cg.get_variable(config[CONF_EMH1_MODBUS_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(parent.register_device(var))

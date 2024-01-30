import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_MODE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    ICON_COUNTER,
    ICON_EMPTY,
    ICON_TIMER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_EMPTY,
    UNIT_WATT,
)

from . import CONF_ABL_EMH1_ID, ABLeMH1

DEPENDENCIES = ["abl_emh1"]

CONF_ENERGY_TODAY = "energy_today"
CONF_ENERGY_TOTAL = "energy_total"
CONF_L1_CURRENT = "l1_current"
CONF_L2_CURRENT = "l2_current"
CONF_L3_CURRENT = "l3_current"
CONF_MAX_CURRENT = "max_current"
CONF_OUTLET_STATE = "outlet_state"
CONF_EN1_STATUS = "en1_status"
CONF_EN2_STATUS = "en2_status"
CONF_DUTY_CYCLE_REDUCED = "duty_cycle_reduced"
CONF_UCP_STATUS = "ucp_status"

UNIT_HOURS = "h"
UNIT_KILO_WATT_HOURS = "kWh"

ICON_MODE = "mdi:heart-pulse"
ICON_ERROR_BITS = "mdi:alert-circle-outline"

SENSORS = [
    CONF_ENERGY_TODAY,
    CONF_ENERGY_TOTAL,
		CONF_L1_CURRENT,
		CONF_L2_CURRENT,
		CONF_L3_CURRENT,
		CONF_MAX_CURRENT,
    CONF_OUTLET_STATE,
		CONF_EN1_STATUS,
		CONF_EN2_STATUS,
		CONF_DUTY_CYCLE_REDUCED,
		CONF_UCP_STATUS
]

# pylint: disable=too-many-function-args
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ABL_EMH1_ID): cv.use_id(ABLeMH1),
        cv.Optional(CONF_ENERGY_TODAY): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILO_WATT_HOURS,
            icon=ICON_COUNTER,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_ENERGY_TOTAL): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILO_WATT_HOURS,
            icon=ICON_COUNTER,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_L1_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_L2_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_L3_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MAX_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_OUTLET_STATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_EN1_STATUS): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_EN2_STATUS): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_DUTY_CYCLE_REDUCED): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_UCP_STATUS): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_MODE): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),

    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ABL_EMH1_ID])
    for key in SENSORS:
        if key in config:
            conf = config[key]
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))

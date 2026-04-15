import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

duco_gateway_ns = cg.esphome_ns.namespace("duco_gateway")
DucoGateway = duco_gateway_ns.class_("DucoGateway", cg.Component, uart.UARTDevice)

# Node addresses
CONF_SERIAL_SWITCH_PIN = "serial_switch_pin"
CONF_EXT_SENSOR_NODE = "ext_sensor_node"
CONF_POLL_INTERVAL = "poll_interval"

# Sensor IDs
CONF_VENT_MODE_SENSOR   = "vent_mode_sensor"
CONF_VENT_PCT_SENSOR    = "vent_percentage_sensor"
CONF_FAN_SPEED_SENSOR   = "fan_speed_sensor"
CONF_COUNTDOWN_SENSOR   = "countdown_sensor"
CONF_BOX_TEMP_SENSOR    = "box_temperature_sensor"
CONF_BOX_RH_SENSOR      = "box_humidity_sensor"
CONF_EXT_CO2_SENSOR     = "ext_co2_sensor"
CONF_EXT_TEMP_SENSOR    = "ext_temperature_sensor"
CONF_EXT_RH_SENSOR      = "ext_humidity_sensor"
CONF_RH_SENSOR_NODE     = "rh_sensor_node"
CONF_VENT_MODE_TEXT     = "vent_mode_text_sensor"

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(DucoGateway),
        cv.Optional(CONF_SERIAL_SWITCH_PIN): cv.uint8_t,
        cv.Optional(CONF_EXT_SENSOR_NODE, default=3): cv.uint8_t,
        cv.Optional(CONF_POLL_INTERVAL, default="15s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_VENT_MODE_SENSOR):  cv.use_id(sensor.Sensor),
        cv.Optional(CONF_VENT_PCT_SENSOR):   cv.use_id(sensor.Sensor),
        cv.Optional(CONF_FAN_SPEED_SENSOR):  cv.use_id(sensor.Sensor),
        cv.Optional(CONF_COUNTDOWN_SENSOR):  cv.use_id(sensor.Sensor),
        cv.Optional(CONF_BOX_TEMP_SENSOR):   cv.use_id(sensor.Sensor),
        cv.Optional(CONF_BOX_RH_SENSOR):     cv.use_id(sensor.Sensor),
        cv.Optional(CONF_EXT_CO2_SENSOR):    cv.use_id(sensor.Sensor),
        cv.Optional(CONF_EXT_TEMP_SENSOR):   cv.use_id(sensor.Sensor),
        cv.Optional(CONF_EXT_RH_SENSOR):     cv.use_id(sensor.Sensor),
        cv.Optional(CONF_RH_SENSOR_NODE, default=4): cv.uint8_t,
        cv.Optional(CONF_VENT_MODE_TEXT):    cv.use_id(text_sensor.TextSensor),
    })
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_SERIAL_SWITCH_PIN in config:
        cg.add(var.set_serial_switch_pin(config[CONF_SERIAL_SWITCH_PIN]))

    cg.add(var.set_ext_sensor_node(config[CONF_EXT_SENSOR_NODE]))
    cg.add(var.set_rh_sensor_node(config[CONF_RH_SENSOR_NODE]))
    cg.add(var.set_poll_interval_ms(config[CONF_POLL_INTERVAL].total_milliseconds))

    for conf_key, method in [
        (CONF_VENT_MODE_SENSOR,  "set_vent_mode_sensor"),
        (CONF_VENT_PCT_SENSOR,   "set_vent_percentage_sensor"),
        (CONF_FAN_SPEED_SENSOR,  "set_fan_speed_sensor"),
        (CONF_COUNTDOWN_SENSOR,  "set_countdown_sensor"),
        (CONF_BOX_TEMP_SENSOR,   "set_box_temperature_sensor"),
        (CONF_BOX_RH_SENSOR,     "set_box_humidity_sensor"),
        (CONF_EXT_CO2_SENSOR,    "set_ext_co2_sensor"),
        (CONF_EXT_TEMP_SENSOR,   "set_ext_temperature_sensor"),
        (CONF_EXT_RH_SENSOR,     "set_ext_humidity_sensor"),
        (CONF_VENT_MODE_TEXT,    "set_vent_mode_text_sensor"),
    ]:
        if conf_key in config:
            s = await cg.get_variable(config[conf_key])
            cg.add(getattr(var, method)(s))
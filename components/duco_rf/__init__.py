"""ESPHome component: duco_rf — Ducobox CC1101 RF gateway."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cc1101, sensor, text_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["cc1101"]
AUTO_LOAD = ["sensor", "text_sensor"]

duco_rf_ns = cg.esphome_ns.namespace("duco_rf")
DucoRF = duco_rf_ns.class_("DucoRF", cg.Component)

CONF_CC1101_ID        = "cc1101_id"
CONF_VENT_MODE_SENSOR = "vent_mode_sensor"
CONF_VENT_MODE_TEXT   = "vent_mode_text_sensor"
CONF_NETWORK_ID_TEXT  = "network_id_text_sensor"
CONF_DEVICE_ADDRESS_TEXT = "device_address_text_sensor"
CONF_LOG_RF_MESSAGES = "log_rf_messages"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DucoRF),
    cv.Required(CONF_CC1101_ID):                        cv.use_id(cc1101.CC1101Component),
    cv.Optional(CONF_VENT_MODE_SENSOR):                 cv.use_id(sensor.Sensor),
    cv.Optional(CONF_VENT_MODE_TEXT):                   cv.use_id(text_sensor.TextSensor),
    cv.Optional(CONF_NETWORK_ID_TEXT):                  cv.use_id(text_sensor.TextSensor),
    cv.Optional(CONF_DEVICE_ADDRESS_TEXT):              cv.use_id(text_sensor.TextSensor),
    cv.Optional(CONF_LOG_RF_MESSAGES, default=False):   cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cc1101_var = await cg.get_variable(config[CONF_CC1101_ID])
    cg.add(var.set_cc1101(cc1101_var))
    cg.add(var.set_log_rf_messages(config[CONF_LOG_RF_MESSAGES]))

    if CONF_VENT_MODE_SENSOR in config:
        s = await cg.get_variable(config[CONF_VENT_MODE_SENSOR])
        cg.add(var.set_vent_mode_sensor(s))

    if CONF_VENT_MODE_TEXT in config:
        s = await cg.get_variable(config[CONF_VENT_MODE_TEXT])
        cg.add(var.set_vent_mode_text_sensor(s))

    if CONF_NETWORK_ID_TEXT in config:
        s = await cg.get_variable(config[CONF_NETWORK_ID_TEXT])
        cg.add(var.set_network_id_text_sensor(s))

    if CONF_DEVICE_ADDRESS_TEXT in config:
        s = await cg.get_variable(config[CONF_DEVICE_ADDRESS_TEXT])
        cg.add(var.set_device_address_text_sensor(s))

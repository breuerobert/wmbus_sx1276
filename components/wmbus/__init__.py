import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.log import Fore, color
from esphome.components import time
from esphome.components import mqtt
from esphome.components import wifi
from esphome.components import ethernet
from esphome.components.network import IPAddress
from esphome.components import spi
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_DATA,
    CONF_FREQUENCY,
    CONF_IP_ADDRESS,
    CONF_PORT,
    CONF_FORMAT,
    CONF_TIME_ID,
    CONF_MQTT_ID,
    CONF_MQTT,
    CONF_BROKER,
    CONF_USERNAME,
    CONF_PASSWORD,
    CONF_RETAIN,
)

from esphome.const import SOURCE_FILE_EXTENSIONS

CONF_TRANSPORT = "transport"

CONF_LED_PIN = "led_pin"
CONF_LED_BLINK_TIME = "led_blink_time"
CONF_LOG_ALL = "log_all"
CONF_ALL_DRIVERS = "all_drivers"
CONF_INFO_COMP_ID = "info_comp_id"
CONF_WMBUS_MQTT_ID = "wmbus_mqtt_id"
CONF_WMBUS_MQTT_RAW = "mqtt_raw"
CONF_WMBUS_MQTT_RAW_PREFIX = "mqtt_raw_prefix"
CONF_WMBUS_MQTT_RAW_PARSED = "mqtt_raw_parsed"
CONF_WMBUS_MQTT_RAW_FORMAT = "mqtt_raw_format"
CONF_CLIENTS = 'clients'
CONF_ETH_REF = "wmbus_eth_id"
CONF_WIFI_REF = "wmbus_wifi_id"
CONF_BANDWIDTH = "bandwidth"
CONF_BITRATE = "bitrate"
CONF_DEVIATION = "deviation"
CONF_DIO0_PIN = "dio0_pin"
CONF_PREAMBLE_DETECT = "preamble_detect"
CONF_PREAMBLE_ERRORS = "preamble_errors"
CONF_PREAMBLE_POLARITY = "preamble_polarity"
CONF_RST_PIN = "rst_pin"
CONF_SYNC_VALUE = "sync_value"

CODEOWNERS = ["@breuerobert"]

DEPENDENCIES = ["time", "spi"]
AUTO_LOAD = ["sensor", "text_sensor"]

wmbus_ns = cg.esphome_ns.namespace('wmbus')
WMBusComponent = wmbus_ns.class_('WMBusComponent', cg.Component)
InfoComponent = wmbus_ns.class_('InfoComponent', cg.Component)
Client = wmbus_ns.struct('Client')
Format = wmbus_ns.enum("Format")
RawFormat = wmbus_ns.enum("RawFormat")
Transport = wmbus_ns.enum("Transport")
MqttClient = wmbus_ns.struct('MqttClient')
SX127x = wmbus_ns.class_("SX127x", spi.SPIDevice)
SX127xBw = wmbus_ns.enum("SX127xBw")

FORMAT = {
    "HEX": Format.FORMAT_HEX,
    "RTLWMBUS": Format.FORMAT_RTLWMBUS,
}
validate_format = cv.enum(FORMAT, upper=True)

RAW_FORMAT = {
    "JSON": RawFormat.RAW_FORMAT_JSON,
    "RTLWMBUS": RawFormat.RAW_FORMAT_RTLWMBUS,
}
validate_raw_format = cv.enum(RAW_FORMAT, upper=True)

TRANSPORT = {
    "TCP": Transport.TRANSPORT_TCP,
    "UDP": Transport.TRANSPORT_UDP,
}
validate_transport = cv.enum(TRANSPORT, upper=True)

BW = {
    "2_6kHz": SX127xBw.SX127X_BW_2_6,
    "3_1kHz": SX127xBw.SX127X_BW_3_1,
    "3_9kHz": SX127xBw.SX127X_BW_3_9,
    "5_2kHz": SX127xBw.SX127X_BW_5_2,
    "6_3kHz": SX127xBw.SX127X_BW_6_3,
    "7_8kHz": SX127xBw.SX127X_BW_7_8,
    "10_4kHz": SX127xBw.SX127X_BW_10_4,
    "12_5kHz": SX127xBw.SX127X_BW_12_5,
    "15_6kHz": SX127xBw.SX127X_BW_15_6,
    "20_8kHz": SX127xBw.SX127X_BW_20_8,
    "25_0kHz": SX127xBw.SX127X_BW_25_0,
    "31_3kHz": SX127xBw.SX127X_BW_31_3,
    "41_7kHz": SX127xBw.SX127X_BW_41_7,
    "50_0kHz": SX127xBw.SX127X_BW_50_0,
    "62_5kHz": SX127xBw.SX127X_BW_62_5,
    "83_3kHz": SX127xBw.SX127X_BW_83_3,
    "100_0kHz": SX127xBw.SX127X_BW_100_0,
    "125_0kHz": SX127xBw.SX127X_BW_125_0,
    "166_7kHz": SX127xBw.SX127X_BW_166_7,
    "200_0kHz": SX127xBw.SX127X_BW_200_0,
    "250_0kHz": SX127xBw.SX127X_BW_250_0,
    "500_0kHz": SX127xBw.SX127X_BW_500_0,
}

CLIENT_SCHEMA = cv.Schema({
    cv.GenerateID():                              cv.declare_id(Client),
    cv.Required(CONF_NAME):                       cv.string_strict,
    cv.Required(CONF_IP_ADDRESS):                 cv.ipv4address,
    cv.Required(CONF_PORT):                       cv.port,
    cv.Optional(CONF_TRANSPORT, default="TCP"):   cv.templatable(validate_transport),
    cv.Optional(CONF_FORMAT, default="RTLWMBUS"): cv.templatable(validate_format),
})

WMBUS_MQTT_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_WMBUS_MQTT_ID):        cv.declare_id(MqttClient),
    cv.Required(CONF_USERNAME):               cv.string_strict,
    cv.Required(CONF_PASSWORD):               cv.string_strict,
    cv.Required(CONF_BROKER):                 cv.ipv4address,
    cv.Optional(CONF_PORT,    default=1883):  cv.port,
    cv.Optional(CONF_RETAIN,  default=False): cv.boolean,
})

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(CONF_INFO_COMP_ID):                  cv.declare_id(InfoComponent),
        cv.GenerateID():                                   cv.declare_id(WMBusComponent),
        cv.OnlyWith(CONF_MQTT_ID, "mqtt"):                 cv.use_id(mqtt.MQTTClientComponent),
        cv.OnlyWith(CONF_TIME_ID, "time"):                 cv.use_id(time.RealTimeClock),
        cv.OnlyWith(CONF_WIFI_REF, "wifi"):                cv.use_id(wifi.WiFiComponent),
        cv.OnlyWith(CONF_ETH_REF, "ethernet"):             cv.use_id(ethernet.EthernetComponent),
        cv.Optional(CONF_LED_PIN):                         pins.gpio_output_pin_schema,
        cv.Optional(CONF_LED_BLINK_TIME, default="200ms"): cv.positive_time_period,
        cv.Optional(CONF_LOG_ALL,        default=False):   cv.boolean,
        cv.Optional(CONF_ALL_DRIVERS,    default=False):   cv.boolean,
        cv.Optional(CONF_CLIENTS):                         cv.ensure_list(CLIENT_SCHEMA),
        cv.Optional(CONF_MQTT):                            cv.ensure_schema(WMBUS_MQTT_SCHEMA),
        cv.Optional(CONF_WMBUS_MQTT_RAW, default=False): cv.boolean,
        cv.Optional(CONF_WMBUS_MQTT_RAW_PREFIX, default=""): cv.string,
        cv.Optional(CONF_WMBUS_MQTT_RAW_FORMAT, default="JSON"): cv.templatable(validate_raw_format),
        cv.Optional(CONF_WMBUS_MQTT_RAW_PARSED, default=True): cv.boolean,
        cv.Optional(CONF_BANDWIDTH, default="200_0kHz"):   cv.enum(BW),
        cv.Optional(CONF_BITRATE, default=100000):         cv.int_range(min=500, max=300000),
        cv.Optional(CONF_DEVIATION, default=50000):        cv.int_range(min=0, max=100000),
        cv.Optional(CONF_DIO0_PIN):                        pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_FREQUENCY, default=868950000):    cv.int_range(min=300000000, max=928000000),
        cv.Optional(CONF_PREAMBLE_DETECT, default=2):      cv.int_range(min=0, max=4),
        cv.Optional(CONF_PREAMBLE_ERRORS, default=10):      cv.int_range(min=0, max=31),
        cv.Optional(CONF_PREAMBLE_POLARITY, default=0x55): cv.All(
            cv.hex_int, cv.one_of(0xAA, 0x55)
        ),
        cv.Required(CONF_RST_PIN):                         pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_SYNC_VALUE, default=[0x54, 0x3D]): cv.ensure_list(cv.hex_uint8_t),
    })
    .extend(spi.spi_device_schema(True, 8e6, "mode0"))
)

def validate_raw_data(value):
    if isinstance(value, str):
        return value.encode("utf-8")
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        return cv.Schema([cv.hex_uint8_t])(value)
    raise cv.Invalid(
        "data must either be a string wrapped in quotes or a list of bytes"
    )

def safe_ip(ip):
    if ip is None:
        return IPAddress(0, 0, 0, 0)
    return IPAddress(str(ip))

async def to_code(config):
    var_adv = cg.new_Pvariable(config[CONF_INFO_COMP_ID])
    await cg.register_component(var_adv, {})

    if (config.get(CONF_MQTT_ID) and config.get(CONF_MQTT)):
        print(color(Fore.RED, "Only one MQTT can be configured!"))
        exit()

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.create_sx127x())
    dio0_pin = await cg.gpio_pin_expression(config[CONF_DIO0_PIN])
    cg.add(var.set_dio0_pin(dio0_pin))
    rst_pin = await cg.gpio_pin_expression(config[CONF_RST_PIN])
    cg.add(var.set_rst_pin(rst_pin))
    cg.add(var.set_bandwidth(config[CONF_BANDWIDTH]))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_deviation(config[CONF_DEVIATION]))
    cg.add(var.set_bitrate(config[CONF_BITRATE]))
    cg.add(var.set_preamble_detect(config[CONF_PREAMBLE_DETECT]))
    cg.add(var.set_preamble_polarity(config[CONF_PREAMBLE_POLARITY]))
    cg.add(var.set_preamble_errors(config[CONF_PREAMBLE_ERRORS]))
    cg.add(var.set_sync_value(config[CONF_SYNC_VALUE]))
    await spi.register_spi_device(var.get_sx127x(), config)

    time = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time(time))

    if config.get(CONF_ETH_REF):
        eth = await cg.get_variable(config[CONF_ETH_REF])
        cg.add(var.set_eth(eth))

    if config.get(CONF_WIFI_REF):
        wifi = await cg.get_variable(config[CONF_WIFI_REF])
        cg.add(var.set_wifi(wifi))

    if config.get(CONF_MQTT_ID):
        mqtt = await cg.get_variable(config[CONF_MQTT_ID])
        cg.add(var.set_mqtt(mqtt))

    if config.get(CONF_MQTT):
        conf = config.get(CONF_MQTT)
        cg.add_define("USE_WMBUS_MQTT")
        cg.add_library("knolleary/PubSubClient", "2.8")
        cg.add(var.set_mqtt(conf[CONF_USERNAME],
                            conf[CONF_PASSWORD],
                            safe_ip(conf[CONF_BROKER]),
                            conf[CONF_PORT],
                            conf[CONF_RETAIN]))

    cg.add(var.set_mqtt_raw(config[CONF_WMBUS_MQTT_RAW]))
    cg.add(var.set_mqtt_raw_prefix(config[CONF_WMBUS_MQTT_RAW_PREFIX]))
    cg.add(var.set_mqtt_raw_format(config[CONF_WMBUS_MQTT_RAW_FORMAT]))
    cg.add(var.set_mqtt_raw_parsed(config[CONF_WMBUS_MQTT_RAW_PARSED]))

    cg.add(var.set_log_all(config[CONF_LOG_ALL]))

    for conf in config.get(CONF_CLIENTS, []):
        cg.add(var.add_client(conf[CONF_NAME],
                              safe_ip(conf[CONF_IP_ADDRESS]),
                              conf[CONF_PORT],
                              conf[CONF_TRANSPORT],
                              conf[CONF_FORMAT]))

    if CONF_LED_PIN in config:
        led_pin = await cg.gpio_pin_expression(config[CONF_LED_PIN])
        cg.add(var.set_led_pin(led_pin))
        cg.add(var.set_led_blink_time(config[CONF_LED_BLINK_TIME].total_milliseconds))

    cg.add_platformio_option("build_src_filter", ["+<*>", "-<.git/>", "-<.svn/>"])

    if config[CONF_ALL_DRIVERS]:
        cg.add_platformio_option("build_src_filter", ["+<**/wmbus/driver_*.cpp>"])
    else:
        cg.add_platformio_option("build_src_filter", ["-<**/wmbus/driver_*.cpp>"])

    cg.add_platformio_option("build_src_filter", ["+<**/wmbus/driver_unknown.cpp>"])

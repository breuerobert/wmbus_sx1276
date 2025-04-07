#pragma once

#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#ifdef USE_WMBUS_MQTT
#include <PubSubClient.h>
#elif defined(USE_MQTT)
#include "esphome/components/mqtt/mqtt_client.h"
#endif
#ifdef USE_ETHERNET
#include "esphome/components/ethernet/ethernet_component.h"
#elif defined(USE_WIFI)
#include "esphome/components/wifi/wifi_component.h"
#endif
#include "esphome/components/time/real_time_clock.h"

#include <map>
#include <utility>
#include <string>

#include "rf_sx127x.h"
#include "m_bus_data.h"
#include "crc.h"

#include "utils.h"

#include <WiFiClient.h>
#include <WiFiUdp.h>


namespace esphome {
namespace wmbus {

  enum Format : uint8_t {
    FORMAT_HEX      = 0,
    FORMAT_RTLWMBUS = 1,
  };

  enum RawFormat : uint8_t {
    RAW_FORMAT_JSON     = 0,
    RAW_FORMAT_RTLWMBUS = 1,
  };

  enum Transport : uint8_t {
    TRANSPORT_TCP = 0,
    TRANSPORT_UDP = 1,
  };

  struct Client {
    std::string name;
    network::IPAddress ip;
    uint16_t port;
    Transport transport;
    Format format;
  };

  struct MqttClient {
    std::string name;
    std::string password;
    network::IPAddress ip;
    uint16_t port;
    bool retained;
  };

  class WMBusListener {
    public:
      WMBusListener(const uint32_t id, const std::string type, const std::string key);
      uint32_t id;
      std::string type;
      std::string myKey;
      std::vector<unsigned char> key{};
      std::map<std::pair<std::string, std::string>, sensor::Sensor *> fields{};
      void add_sensor(std::string field, std::string unit, sensor::Sensor *sensor) {
        this->fields[std::pair<std::string, std::string>(field, unit)] = sensor;
      };
      std::map<std::string, text_sensor::TextSensor *> text_fields{};
      void add_sensor(std::string field, text_sensor::TextSensor *sensor) {
        this->text_fields[field] = sensor;
      };

      void dump_config();
      int char_to_int(char input);
      bool hex_to_bin(const std::string &src, std::vector<unsigned char> *target) { return hex_to_bin(src.c_str(), target); };
      bool hex_to_bin(const char* src, std::vector<unsigned char> *target);
  };
  class InfoComponent : public Component {
    public:
      void setup() override;
      float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  };

  class WMBusComponent : public Component {
    public:
      void set_bandwidth(SX127xBw bandwidth) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_bandwidth(bandwidth); }
      void set_bitrate(uint32_t bitrate) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_bitrate(bitrate); }
      void set_deviation(uint32_t deviation) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_deviation(deviation); }
      void set_dio0_pin(InternalGPIOPin *dio0_pin) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_dio0_pin(dio0_pin); }
      void set_frequency(uint32_t frequency) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_frequency(frequency); }
      void set_preamble_errors(uint8_t preamble_errors) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_preamble_errors(preamble_errors); }
      void set_preamble_polarity(uint8_t preamble_polarity) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_preamble_polarity(preamble_polarity); }
      void set_preamble_detect(uint8_t preamble_detect) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_preamble_detect(preamble_detect); }
      void set_rst_pin(InternalGPIOPin *rst_pin) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_rst_pin(rst_pin); }
      void set_sync_value(const std::vector<uint8_t> &sync_value) { if(this->sx127x_ == nullptr) { return; } this->sx127x_->set_sync_value(sync_value); }
      void setup() override;
      void loop() override;
      void dump_config() override;
      float get_setup_priority() const override { return setup_priority::LATE; }
      void set_led_pin(GPIOPin *led) { this->led_pin_ = led; }
      void set_led_blink_time(uint32_t led_blink_time) { this->led_blink_time_ = led_blink_time; }
      void register_wmbus_listener(const uint32_t meter_id, const std::string type, const std::string key);
      void add_sensor(uint32_t meter_id, std::string field, std::string unit, sensor::Sensor *sensor) {
        if (this->wmbus_listeners_.count(meter_id) != 0) {
          this->wmbus_listeners_[meter_id]->add_sensor(field, unit, sensor);
        }
      }
      void create_sx127x(){ sx127x_ = new SX127x(); }
      esphome::wmbus::SX127x& get_sx127x(){ return *sx127x_; }
      void add_sensor(uint32_t meter_id, std::string field, text_sensor::TextSensor  *sensor) {
        if (this->wmbus_listeners_.count(meter_id) != 0) {
          this->wmbus_listeners_[meter_id]->add_sensor(field, sensor);
        }
      }
#ifdef USE_ETHERNET
      void set_eth(ethernet::EthernetComponent *eth_component) { this->net_component_ = eth_component; }
#elif defined(USE_WIFI)
      void set_wifi(wifi::WiFiComponent *wifi_component) { this->net_component_ = wifi_component; }
#endif
      void set_time(time::RealTimeClock *time) { this->time_ = time; }
#ifdef USE_WMBUS_MQTT
      void set_mqtt(const std::string name,
                    const std::string password,
                    const network::IPAddress ip,
                    const uint16_t port,
                    const bool retained) {
        this->mqtt_ = new MqttClient{name, password, ip, port, retained};
      }
#elif defined(USE_MQTT)
      void set_mqtt(mqtt::MQTTClientComponent *mqtt_client) { this->mqtt_client_ = mqtt_client; }
#endif
      void set_mqtt_raw(bool send_raw) { this->mqtt_raw = send_raw; }
      void set_mqtt_raw_prefix(std::string prefix) { this->mqtt_raw_prefix = prefix; }
      void set_mqtt_raw_parsed(bool parsed) { this->mqtt_raw_parsed = parsed; }
      void set_mqtt_raw_format(RawFormat format) { this->mqtt_raw_format = format; }
      void set_log_all(bool log_all) { this->log_all_ = log_all; }
      void add_client(const std::string name,
                      const network::IPAddress ip,
                      const uint16_t port,
                      const Transport transport,
                      const Format format) {
        clients_.push_back({name, ip, port, transport, format});
      }

    protected:
      const LogString *format_to_string(Format format);
      const LogString *transport_to_string(Transport transport);
      void send_to_clients(WMbusFrame &mbus_data);
      void send_mqtt_raw(Telegram &t, WMbusFrame &mbus_data);
      void led_blink();
      void led_handler();
      HighFrequencyLoopRequester high_freq_;
      GPIOPin *led_pin_{nullptr};
      std::map<uint32_t, WMBusListener *> wmbus_listeners_{};
      std::vector<Client> clients_{};
      WiFiClient tcp_client_;
      WiFiUDP udp_client_;
      time::RealTimeClock *time_{nullptr};
      uint32_t led_blink_time_{0};
      uint32_t led_on_millis_{0};
      bool led_on_{false};
      bool log_all_{false};
      RxLoop rf_mbus_;
      esphome::wmbus::SX127x* sx127x_{nullptr};
#ifdef USE_ETHERNET
      ethernet::EthernetComponent *net_component_{nullptr};
#elif defined(USE_WIFI)
      wifi::WiFiComponent *net_component_{nullptr};
#endif
#ifdef USE_WMBUS_MQTT
      PubSubClient mqtt_client_;
      MqttClient *mqtt_{nullptr};
#elif defined(USE_MQTT)
      mqtt::MQTTClientComponent *mqtt_client_{nullptr};
#endif
      bool mqtt_raw{false};
      bool mqtt_raw_parsed{true};
      bool mqtt_raw_format{RAW_FORMAT_JSON};
      std::string mqtt_raw_prefix{""};
      time_t frame_timestamp_;
  };

}  // namespace wmbus
}  // namespace esphome

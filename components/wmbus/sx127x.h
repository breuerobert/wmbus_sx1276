#pragma once

#include "sx127x_reg.h"
#include "esphome/components/spi/spi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mbus.h"
#include "utils_my.h"
#include "decode3of6.h"
#include "m_bus_data.h"

#include <vector>

#if __cplusplus < 201402L
namespace std {
    template<class T, class... Args>
    std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}
#endif

extern QueueHandle_t frameQueue;

namespace esphome {
namespace wmbus {

enum SX127xBw : uint8_t {
  SX127X_BW_2_6,
  SX127X_BW_3_1,
  SX127X_BW_3_9,
  SX127X_BW_5_2,
  SX127X_BW_6_3,
  SX127X_BW_7_8,
  SX127X_BW_10_4,
  SX127X_BW_12_5,
  SX127X_BW_15_6,
  SX127X_BW_20_8,
  SX127X_BW_25_0,
  SX127X_BW_31_3,
  SX127X_BW_41_7,
  SX127X_BW_50_0,
  SX127X_BW_62_5,
  SX127X_BW_83_3,
  SX127X_BW_100_0,
  SX127X_BW_125_0,
  SX127X_BW_166_7,
  SX127X_BW_200_0,
  SX127X_BW_250_0,
  SX127X_BW_500_0,
};

typedef struct LoopData {
  uint16_t bytesRx;             // Total bytes read so far
  uint8_t  lengthField;         // The L-field in the WMBUS packet (first byte of preamble info)
  uint16_t length;              // Total number of bytes expected in the packet
  uint16_t bytesLeft;           // Bytes left to be read
} LoopData;

class SX127x : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_8MHZ> {
 public:
  void set_bandwidth(SX127xBw bandwidth) { this->bandwidth_ = bandwidth; }
  void set_bitrate(uint32_t bitrate) { this->bitrate_ = bitrate; }
  void set_deviation(uint32_t deviation) { this->deviation_ = deviation; }
  void set_dio0_pin(InternalGPIOPin *dio0_pin) { this->dio0_pin_ = dio0_pin; }
  void set_frequency(uint32_t frequency) { this->frequency_ = frequency; }
  void set_preamble_errors(uint8_t preamble_errors) { this->preamble_errors_ = preamble_errors; }
  void set_preamble_polarity(uint8_t preamble_polarity) { this->preamble_polarity_ = preamble_polarity; }
  void set_preamble_detect(uint8_t preamble_detect) { this->preamble_detect_ = preamble_detect; }
  void set_rst_pin(InternalGPIOPin *rst_pin) { this->rst_pin_ = rst_pin; }
  void set_sync_value(const std::vector<uint8_t> &sync_value) { this->sync_value_ = sync_value; }
  void run_image_cal();
  void configure();
  void loop();
  void dump_config();

 protected:
  void restart_receiver();
  void set_mode_(SX127xOpMode mode);
  void read_fifo_(std::vector<uint8_t> &packet);
  void read_fifo_byte(uint8_t* fifo_byte);
  void flush_fifo();
  void write_register_(uint8_t reg, uint8_t value);
  uint8_t read_register_(uint8_t reg);
  std::vector<uint8_t> read_fifo_chunk(SX127x *radio, bool readingLastChunk);
  bool process_preamble(std::vector<uint8_t> &chunk, LoopData &loop_data, WMbusData &data_in);
  std::vector<uint8_t> sync_value_;
  InternalGPIOPin *dio0_pin_{nullptr};
  InternalGPIOPin *rst_pin_{nullptr};
  SX127xBw bandwidth_;
  uint32_t bitrate_;
  uint32_t deviation_;
  uint32_t frequency_;
  uint8_t preamble_detect_;
  uint8_t preamble_errors_;
  uint8_t preamble_polarity_;
  WMbusData data_in{0};
};

}  // namespace wmbus
}  // namespace esphome

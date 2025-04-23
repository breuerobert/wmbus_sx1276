#include "sx127x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#define ADAPTIVE_TIMEOUT_MS 30

#define FIFO_THRESHOLD_SIZE 48

#define WMBUS_MODE_C_PREAMBLE 0x54
#define WMBUS_BLOCK_A_PREAMBLE 0xCD
#define WMBUS_BLOCK_B_PREAMBLE 0x3D

QueueHandle_t frameQueue = xQueueCreate(10, sizeof(esphome::wmbus::WMbusFrame*));

namespace esphome {
namespace wmbus {

static const char *const TAG = "sx127x";

static const uint32_t FXOSC = 32000000u;
static const uint16_t RAMP[16] = {3400, 2000, 1000, 500, 250, 125, 100, 62, 50, 40, 31, 25, 20, 15, 12, 10};
static const uint32_t BW_HZ[22] = {2604,  3125,  3906,  5208,  6250,  7812,   10416,  12500,  15625,  20833,  25000,
                                   31250, 41666, 50000, 62500, 83333, 100000, 125000, 166666, 200000, 250000, 500000};
static const uint8_t BW_FSK_OOK[22] = {RX_BW_2_6,   RX_BW_3_1,   RX_BW_3_9,   RX_BW_5_2,  RX_BW_6_3,   RX_BW_7_8,
                                       RX_BW_10_4,  RX_BW_12_5,  RX_BW_15_6,  RX_BW_20_8, RX_BW_25_0,  RX_BW_31_3,
                                       RX_BW_41_7,  RX_BW_50_0,  RX_BW_62_5,  RX_BW_83_3, RX_BW_100_0, RX_BW_125_0,
                                       RX_BW_166_7, RX_BW_200_0, RX_BW_250_0, RX_BW_250_0};

uint8_t SX127x::read_register_(uint8_t reg) {
  this->enable();
  this->write_byte(reg & 0x7F);
  uint8_t value = this->read_byte();
  this->disable();
  return value;
}

void SX127x::write_register_(uint8_t reg, uint8_t value) {
  this->enable();
  this->write_byte(reg | 0x80);
  this->write_byte(value);
  this->disable();
}

void SX127x::read_fifo_(std::vector<uint8_t> &packet) {
  this->enable();
  this->write_byte(REG_FIFO & 0x7F);
  this->read_array(packet.data(), packet.size());
  this->disable();
}

void SX127x::read_fifo_byte(uint8_t* fifo_byte) {
  this->enable();
  this->write_byte(REG_FIFO & 0x7F);
  this->read_array(fifo_byte, 1);
  this->disable();
}

void SX127x::configure() {
  ESP_LOGCONFIG(TAG, "Setting up SX127x...");

  // setup reset
  this->rst_pin_->setup();

  // setup dio0
  if (this->dio0_pin_) {
    this->dio0_pin_->setup();
  }

  // start spi
  this->spi_setup();

  // toggle chip reset
  this->rst_pin_->digital_write(false);
  delayMicroseconds(1000);
  this->rst_pin_->digital_write(true);
  delayMicroseconds(10000);

  // check silicon version to make sure hw is ok
  if (this->read_register_(REG_VERSION) != 0x12) {
    return;
  }

  // enter sleep mode
  this->write_register_(REG_OP_MODE, MODE_SLEEP);
  delayMicroseconds(1000);

  // set freq
  uint64_t frf = ((uint64_t) this->frequency_ << 19) / FXOSC;
  this->write_register_(REG_FRF_MSB, (uint8_t) ((frf >> 16) & 0xFF));
  this->write_register_(REG_FRF_MID, (uint8_t) ((frf >> 8) & 0xFF));
  this->write_register_(REG_FRF_LSB, (uint8_t) ((frf >> 0) & 0xFF));

  // enter standby mode
  this->write_register_(REG_OP_MODE, MODE_STDBY);
  delayMicroseconds(1000);

  // run image cal
  this->run_image_cal();

  // set correct modulation and go back to sleep
  this->write_register_(REG_OP_MODE, MOD_FSK | MODE_SLEEP);
  delayMicroseconds(1000);

  // set the channel bw
  this->write_register_(REG_RX_BW, BW_FSK_OOK[this->bandwidth_]);

  // set fdev
  uint32_t fdev = std::min((this->deviation_ * 4096) / 250000, (uint32_t) 0x3FFF);
  this->write_register_(REG_FDEV_MSB, (uint8_t) ((fdev >> 8) & 0xFF));
  this->write_register_(REG_FDEV_LSB, (uint8_t) ((fdev >> 0) & 0xFF));

  // set bitrate
  uint64_t bitrate = (FXOSC + this->bitrate_ / 2) / this->bitrate_;  // round up
  this->write_register_(REG_BITRATE_MSB, (uint8_t) ((bitrate >> 8) & 0xFF));
  this->write_register_(REG_BITRATE_LSB, (uint8_t) ((bitrate >> 0) & 0xFF));

  // configure rx and afc
  uint8_t trigger = (this->preamble_detect_ > 0) ? TRIGGER_PREAMBLE : TRIGGER_RSSI;
  this->write_register_(REG_AFC_FEI, AFC_AUTO_CLEAR_ON);
  this->write_register_(REG_RX_CONFIG, AFC_AUTO_ON | AGC_AUTO_ON | trigger);

  // configure packet mode
  this->write_register_(REG_PACKET_CONFIG_2, PACKET_MODE);
  this->write_register_(REG_PAYLOAD_LENGTH_LSB, 0x00); // unlimited packet mode if payload is 0
  this->write_register_(REG_FIFO_THRESH, FIFO_THRESHOLD_SIZE);
  this->write_register_(REG_PACKET_CONFIG_1, CRC_OFF | RF_PACKETCONFIG1_DCFREE_OFF | RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF);
  this->write_register_(REG_DIO_MAPPING1, DIO0_MAPPING_00);

  // config bit synchronizer
  uint8_t polarity = (this->preamble_polarity_ == 0xAA) ? PREAMBLE_AA : PREAMBLE_55;
  uint8_t size = this->sync_value_.size() - 1;
  this->write_register_(REG_SYNC_CONFIG, AUTO_RESTART_PLL_LOCK | polarity | SYNC_ON | size);
  for (uint32_t i = 0; i < this->sync_value_.size(); ++i) {
    this->write_register_(REG_SYNC_VALUE1 + i, this->sync_value_[i]);
  }

  // config preamble detector
  if (this->preamble_detect_ > 0) {
    uint8_t size = (this->preamble_detect_ - 1) << PREAMBLE_DETECTOR_SIZE_SHIFT;
    uint8_t tol = this->preamble_errors_ << PREAMBLE_DETECTOR_TOL_SHIFT;
    this->write_register_(REG_PREAMBLE_DETECT, PREAMBLE_DETECTOR_ON | size | tol);
  } else {
    this->write_register_(REG_PREAMBLE_DETECT, PREAMBLE_DETECTOR_OFF);
  }
}

std::vector<uint8_t> SX127x::read_fifo_chunk(SX127x *radio, bool readingLastChunk) {
  std::vector<uint8_t> chunk;
  chunk.reserve(255); // avoid unneeded realocations for performance reasons
  if(!readingLastChunk) {
    chunk.resize(FIFO_THRESHOLD_SIZE);
    radio->read_fifo_(chunk);
  }
  // Poll FIFO until empty, appending each byte.
  uint8_t fifo_byte;
  while ((radio->read_register_(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOEMPTY) != RF_IRQFLAGS2_FIFOEMPTY) {
    radio->read_fifo_byte(&fifo_byte);
    chunk.push_back(fifo_byte);
  }
  return chunk;
}

void SX127x::restart_receiver() {
  uint8_t rx_config = this->read_register_(REG_RX_CONFIG);

  // If AFC is enabled and/or Frf was changed, restart with PLL lock.
  rx_config |= RESTART_RX_WITH_PLL_LOCK;
  this->write_register_(REG_RX_CONFIG, rx_config);

  this->set_mode_(MODE_RX);

  ESP_LOGV(TAG, "Receiver restarted successfully.");
}

bool SX127x::process_preamble(std::vector<uint8_t> &chunk, LoopData &loop_data, WMbusData &data_in) {
  // We need at least 3 bytes to process a preamble.
  if (chunk.size() < 3) return false;

  // Try Mode C first.
  if (chunk[0] == WMBUS_MODE_C_PREAMBLE) {
    data_in.mode = 'C';
    switch (chunk[1]) {
      case WMBUS_BLOCK_A_PREAMBLE: {  // Mode C1
        loop_data.lengthField = chunk[2];
        loop_data.length = 2 + packetSize(loop_data.lengthField);
        data_in.block = 'A';
        break;
      }
      case WMBUS_BLOCK_B_PREAMBLE: {  // Mode C2
        loop_data.lengthField = chunk[2];
        loop_data.length = 2 + 1 + loop_data.lengthField;
        data_in.block = 'B';
        break;
      }
      default: {
        // Undefined Mode C preamble.
        return false;
      }
    }
    // Discard first 2 bytes (the preamble header for Mode C)
    chunk.erase(chunk.begin(), chunk.begin() + 2);
    return true;
  }
  // Try Mode T1 decoding.
  uint8_t decoded_preamble[2] = {0};
  if (decode3OutOf6(chunk.data(), decoded_preamble)) {
    loop_data.lengthField = decoded_preamble[0];
    data_in.lengthField = loop_data.lengthField;
    loop_data.length = byteSize(packetSize(loop_data.lengthField));
    data_in.mode = 'T';
    data_in.block = 'A';
    return true;
  }
  return false;
}

void SX127x::flush_fifo() {
  while ((this->read_register_(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOEMPTY) == 0) {
    uint8_t dummy;
    this->read_fifo_byte(&dummy);
  }
}

void SX127x::loop() {
  while (true) {
    this->flush_fifo();
    restart_receiver();

    LoopData loop_data{};
    data_in = WMbusData{0};
    auto packet = std::make_unique<std::vector<uint8_t>>();
    packet->reserve(500);

    bool is_packet_complete = false;
    bool is_packet_failed = false;
    while (!is_packet_complete && !is_packet_failed) {
      uint8_t irq_flags = this->read_register_(REG_IRQFLAGS2);
      bool fifoLevelHit = (irq_flags & RF_IRQFLAGS2_FIFOLEVEL);
      bool fifoNotEmpty = ((irq_flags & RF_IRQFLAGS2_FIFOEMPTY) != RF_IRQFLAGS2_FIFOEMPTY);
      bool readingLastChunk = (loop_data.bytesLeft > 0 && loop_data.bytesLeft < FIFO_THRESHOLD_SIZE);

      // Check if FIFO indicates available data.
      if (fifoLevelHit || (fifoNotEmpty && readingLastChunk)) {
        std::vector<uint8_t> chunk = read_fifo_chunk(this, readingLastChunk);

        // Process preamble if not already processed.
        if (loop_data.lengthField == 0) {
          if (!process_preamble(chunk, loop_data, data_in)) {
            ESP_LOGV(TAG, "Undefined or failed preamble detection!");
            is_packet_failed = true;
            break;
          }
          loop_data.bytesRx = 0;
          loop_data.bytesLeft = loop_data.length;
        }

        // Process the received chunk.
        if (!chunk.empty()) {
          if (chunk.size() > loop_data.bytesLeft) {
            chunk.resize(loop_data.bytesLeft);
          }
          loop_data.bytesRx += chunk.size();
          loop_data.bytesLeft -= chunk.size();
          packet->insert(packet->end(), std::make_move_iterator(chunk.begin()), std::make_move_iterator(chunk.end()));
        }
      } // End FIFO condition

      if (!packet->empty() && loop_data.bytesLeft == 0) {
        is_packet_complete = true;
      }
    } // End inner loop

    if (is_packet_failed) {
      ESP_LOGV(TAG, "Failed packet!");
      continue;
    }

    // Verify and decode the received packet.
    if (!packet->empty()) {
      data_in.length = loop_data.bytesRx;
      WMbusFrame* frame = new WMbusFrame();
      frame->rssi = 0;
      frame->lqi = 0;

      if (loop_data.length != data_in.length) {
        ESP_LOGV(TAG, "Length problem: expected(%d) != received(%d)", loop_data.length, data_in.length);
        continue;
      }

      std::move(packet->begin(), packet->end(), data_in.data);

      // Decode the packet.
      if (mBusDecode(data_in, *frame)) {
        frame->mode = data_in.mode;
        frame->block = data_in.block;
        ESP_LOGV(TAG, "Sending packet to queue!");
        if (xQueueSend(frameQueue, &frame, pdMS_TO_TICKS(10)) != pdPASS) {
          ESP_LOGV(TAG, "xQueueSend failed!");
          delete frame;
        }
      } else {
        ESP_LOGV(TAG, "mBusDecode: Failed packet!");
        delete frame;
      }
    }
  } // End outer loop
}

void SX127x::run_image_cal() {
  uint32_t start = millis();
  this->write_register_(REG_IMAGE_CAL, AUTO_IMAGE_CAL_ON | IMAGE_CAL_START | TEMP_THRESHOLD_10C);
  while (this->read_register_(REG_IMAGE_CAL) & IMAGE_CAL_RUNNING) {
    if (millis() - start > 20) {
      ESP_LOGE(TAG, "Image cal failure");
      break;
    }
  }
}

void SX127x::set_mode_(SX127xOpMode mode) {
  uint32_t start = millis();
  this->write_register_(REG_OP_MODE, MOD_FSK | mode);
  while (true) {
    uint8_t curr = this->read_register_(REG_OP_MODE) & MODE_MASK;
    if ((curr == mode) || (mode == MODE_RX && curr == MODE_RX_FS)) {
      break;
    }
    if (millis() - start > 20) {
      ESP_LOGE(TAG, "Set mode failure");
      break;
    }
  }
}

void SX127x::dump_config() {
  ESP_LOGCONFIG(TAG, "SX127x:");
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  RST Pin: ", this->rst_pin_);
  LOG_PIN("  DIO0 Pin: ", this->dio0_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %" PRIu32 " Hz", this->frequency_);
  ESP_LOGCONFIG(TAG, "  Bandwidth: %" PRIu32 " Hz", BW_HZ[this->bandwidth_]);
  ESP_LOGCONFIG(TAG, "  Deviation: %" PRIu32 " Hz", this->deviation_);
  ESP_LOGCONFIG(TAG, "  Modulation: %s", "FSK");
  ESP_LOGCONFIG(TAG, "  Bitrate: %" PRIu32 "b/s", this->bitrate_);
  if (this->preamble_detect_ > 0) {
    ESP_LOGCONFIG(TAG, "  Preamble Detect: %" PRIu8, this->preamble_detect_);
    ESP_LOGCONFIG(TAG, "  Preamble Errors: %" PRIu8, this->preamble_errors_);
  }
  if (this->preamble_detect_ > 0) {
    ESP_LOGCONFIG(TAG, "  Preamble Polarity: 0x%X", this->preamble_polarity_);
  }
  if (!this->sync_value_.empty()) {
    ESP_LOGCONFIG(TAG, "  Sync Value: 0x%s", format_hex(this->sync_value_).c_str());
  }
}

}  // namespace wmbus
}  // namespace esphome

#include "rf_sx127x.h"

namespace esphome {
namespace wmbus {

  static const char *TAG = "rxLoop";

  bool RxLoop::init(esphome::wmbus::SX127x* sx127x_ptr) {
    sx127x = sx127x_ptr;
    sx127x->configure();
    xTaskCreatePinnedToCore(
      sx127xWrapper,  // Task function
      "SX127X",       // Task name
      4096,           // Stack size in bytes
      this->sx127x,   // Parameter (none)
      1,              // Task priority
      nullptr,        // Task handle
      1               // Core ID (ESP32: core 0 or 1)
    );
    return true;
  }

  bool RxLoop::task() {
    WMbusFrame* frame = nullptr;
    if(xQueueReceive(frameQueue, &frame, pdMS_TO_TICKS(10)) == pdPASS) {
      ESP_LOGV(TAG, "Frame received!");
      returnFrame = std::move(*frame);
      delete frame;
      return true;
    }
    return false;
  }

  WMbusFrame RxLoop::get_frame() {
    return this->returnFrame;
  }

  void RxLoop::sx127xWrapper(void* pvParameters) {
    SX127x* sx127x_instance = static_cast<SX127x*>(pvParameters);
    sx127x_instance->loop();
  }

}
}

#pragma once

#include "esphome/core/log.h"

#include "mbus.h"
#include "utils_my.h"
#include "decode3of6.h"
#include "m_bus_data.h"

#include <string>
#include <stdint.h>

#include "sx127x.h"

namespace esphome {
namespace wmbus {

  class RxLoop {
    public:
      bool init(esphome::wmbus::SX127x* sx127x);
      bool task();
      WMbusFrame get_frame();

    private:
      esphome::wmbus::SX127x* sx127x{nullptr};

      static void sx127xWrapper(void* pvParameters);

      WMbusFrame returnFrame;
  };

}
}

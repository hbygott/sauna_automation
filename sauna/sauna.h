#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esphome {
namespace sauna {

class Sauna : public Component {
 public:
  void set_current_temp_sensor(sensor::Sensor *s) { current_temp_sensor_ = s; }
  void set_setpoint_temp_sensor(sensor::Sensor *s) { setpoint_temp_sensor_ = s; }
  void set_timer_sensor(sensor::Sensor *s) { timer_sensor_ = s; }
  void set_running_binary_sensor(binary_sensor::BinarySensor *s) { running_binary_sensor_ = s; }

  void setup() override;
  void loop() override;

  QueueHandle_t byte_queue{nullptr};

 protected:
  sensor::Sensor *current_temp_sensor_{nullptr};
  sensor::Sensor *setpoint_temp_sensor_{nullptr};
  sensor::Sensor *timer_sensor_{nullptr};
  binary_sensor::BinarySensor *running_binary_sensor_{nullptr};


  void process_byte_(uint8_t b);


  int digits[256]{-1};

  bool synced{false};
  int cycles_since_sync{0};
  uint8_t frame[10]{};
  uint8_t frame_idx{0};

  int current_temp{-1};
  int setpoint_temp{-1};
  int last_temp{-1};
  uint8_t stable_counter{0};
  uint8_t blink_counter{0};
  uint32_t last_publish_ms{0};
};

}  // namespace sauna
}  // namespace esphome


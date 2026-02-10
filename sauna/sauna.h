#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esphome {
namespace sauna {

// If button is being sampled, D3 will be low, and D0-D2 address the button
//                                                      // signals  D2 D3 D1 D0
static const uint32_t BUTTON_MASK        = 0x0C006000;  //    pins  27 26 14 13
static const uint32_t BUTTON_TIMER_DOWN  = 0x00000000;  //           0  0  0  0      000
static const uint32_t BUTTON_TIMER_UP    = 0x00002000;  //           0  0  0  1      001
static const uint32_t BUTTON_TEMP_DOWN   = 0x00004000;  //           0  0  1  0      010
static const uint32_t BUTTON_TEMP_UP     = 0x00006000;  //           0  0  1  1      011
static const uint32_t BUTTON_COLOR_LIGHT = 0x08000000;  //           1  0  0  0      100
static const uint32_t BUTTON_RED_LIGHT   = 0x08002000;  //           1  0  0  1      101
//static const uint32_t BUTTON_UNK2      = 0x08004000;  //           1  0  1  0      110 Seems unused
static const uint32_t BUTTON_POWER       = 0x08006000;  //           1  0  1  1      111
static const uint32_t BUTTON_NO_PRESS    = 0xFFFFFFFFu; // sentinal value will never match


class Sauna : public Component {
 public:
  void set_current_temp_sensor(sensor::Sensor *s) { current_temp_sensor_ = s; }
  void set_setpoint_temp_sensor(sensor::Sensor *s) { setpoint_temp_sensor_ = s; }
  void set_timer_sensor(sensor::Sensor *s) { timer_sensor_ = s; }
  void set_running_binary_sensor(binary_sensor::BinarySensor *s) { running_binary_sensor_ = s; }
  void press_button(uint32_t button_code, uint32_t hold_ms);

  void setup() override;
  void loop() override;

  QueueHandle_t byte_queue{nullptr};

 protected:
  sensor::Sensor *current_temp_sensor_{nullptr};
  sensor::Sensor *setpoint_temp_sensor_{nullptr};
  sensor::Sensor *timer_sensor_{nullptr};
  binary_sensor::BinarySensor *running_binary_sensor_{nullptr};

  // Button press state shared with ISR
  volatile uint32_t button_selected{BUTTON_NO_PRESS};
  volatile uint32_t button_hold_until_ms{0};

  // ISR handling
  static void IRAM_ATTR d3_isr_trampoline(void *arg);
  void d3_isr();
  intr_handle_t d3_intr_handle{nullptr};

  void process_byte_(uint8_t b);


  int digits[256]{-1};

  bool synced{false};
  int cycles_since_sync{0};
  uint8_t frame[10]{};
  uint8_t frame_idx{0};

  int current_temp{-1};
  int current_timer{-1};
  int setpoint_temp{-1};
  int last_temp{-1};
  uint8_t stable_counter{0};
  uint8_t blink_counter{0};
  uint32_t last_publish_ms{0};
  uint32_t blink_start_ms{0};
  uint32_t stable_start_ms{0};
};

}  // namespace sauna
}  // namespace esphome


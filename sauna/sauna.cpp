#include "sauna.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"


extern "C" {
  void gpio_matrix_in(uint32_t gpio, uint32_t signal_idx, bool inv);
}

namespace esphome {
namespace sauna {

static const char *const TAG = "sauna";

// ----- Pin Mapping -----
static const int PIN_D0 = 13;   // sampled
static const int PIN_D1 = 14;   // MOSI (data into ESP32)
static const int PIN_D2 = 27;   // SCLK
static const int PIN_D3 = 26;   // "gate" -> use as CS (but inverted)
static const int PIN_D6 = 25;   // sampled

static constexpr uint8_t SYNC = 0xEE;
static constexpr uint8_t BIAS = 0xFF;

// ----- SPI -----
static const spi_host_device_t HOST = VSPI_HOST;
static constexpr int QUEUE_SIZE = 128;;

static spi_slave_transaction_t trans[QUEUE_SIZE];
static uint8_t rx_buf[QUEUE_SIZE];

static void queue_all_() {
  for (int i = 0; i < QUEUE_SIZE; i++) {
    memset(&trans[i], 0, sizeof(trans[i]));
    trans[i].length = 8;
    trans[i].rx_buffer = &rx_buf[i];
    trans[i].user = (void *) (uintptr_t) i;
    spi_slave_queue_trans(HOST, &trans[i], portMAX_DELAY);
  }
}

static void spi_task(void *param) {
  auto *self = static_cast<Sauna *>(param);
  spi_slave_transaction_t *rt = nullptr;
  static uint32_t drops = 0;
  static uint32_t last_log = 0;

  for (;;) {
    // Block a bit waiting for a transaction (prevents hot spinning)
    if (spi_slave_get_trans_result(HOST, &rt, pdMS_TO_TICKS(20)) == ESP_OK) {
      if (rt && rt->trans_len == 8) {
        uint8_t b = *(uint8_t *) rt->rx_buffer;
        // Drop if queue is full (non-blocking)
        if (xQueueSend(self->byte_queue, &b, 0) != pdTRUE) {
           drops++;
        }
        uint32_t now = millis();
        if ((drops > 0) && (now - last_log > 2000)) {
           last_log = now;
           ESP_LOGI(TAG, "SPI drops in last 2s: %u", (unsigned)drops);
           drops = 0;
        }
      }
      // Immediately re-queue the same transaction buffer
      spi_slave_queue_trans(HOST, rt, portMAX_DELAY);
    } else {
      // Nothing ready, yield to other tasks
      vTaskDelay(1);
    }
  }
}


// ----- Button Spoofing -----
void Sauna::press_button(uint32_t button_code, uint32_t hold_ms) {
  // called from HA context (not ISR)
  button_selected = button_code;
  button_hold_until_ms = millis() + hold_ms;
}

void IRAM_ATTR Sauna::d3_isr_trampoline(void *arg) {
  static_cast<Sauna *>(arg)->d3_isr();
}

void IRAM_ATTR Sauna::d3_isr() {
  if ((GPIO.in & BUTTON_MASK) == button_selected) {
    GPIO.out_w1ts = (1UL << PIN_D6);  // gate HIGH = press
  } else {
    GPIO.out_w1tc = (1UL << PIN_D6);  // gate LOW  = release
  }
}


void Sauna::setup() {

  // Set up our lookup table for segment display decode
  for (int i = 0; i < 256; i++) digits[i] = -1;
  digits[0x81] = 0;
  digits[0xE7] = 1;
  digits[0x49] = 2;
  digits[0x45] = 3;
  digits[0x27] = 4;
  digits[0x15] = 5;
  digits[0x11] = 6;
  digits[0xC7] = 7;
  digits[0x01] = 8;
  digits[0x05] = 9;
  digits[0x80] = 10;
  digits[0xE6] = 11;
  digits[0x48] = 12;
  digits[0x44] = 13;
  digits[0x26] = 14;
 
  gpio_config_t io = {};
  io.intr_type = GPIO_INTR_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.mode = GPIO_MODE_INPUT;

  io.pin_bit_mask =
      (1ULL << PIN_D0) |
      (1ULL << PIN_D1) |   // MOSI
      (1ULL << PIN_D2) |   // SCLK
      (1ULL << PIN_D3);    // CS gate

  gpio_config(&io);

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_D1;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = PIN_D2;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 1;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.spics_io_num = -1;     // manual routing/invert
  slvcfg.queue_size = QUEUE_SIZE;
  slvcfg.mode = 3;

  esp_err_t err = spi_slave_initialize(HOST, &buscfg, &slvcfg, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "spi_slave_initialize failed: %d", (int) err);
    return;
  }

  // invert CS so D3-high => CS asserted
  gpio_matrix_in(PIN_D3, VSPICS0_IN_IDX, true);

  queue_all_();

  byte_queue = xQueueCreate(1024, sizeof(uint8_t));  // 1KB buffer; tune later

  xTaskCreatePinnedToCore(spi_task,
                          "sauna_spi",
                          4096,   // stack
                          this,
                          5,      // priority (a bit above normal app code)
                          nullptr,
                          1       // core 1 keeps it off WiFi-heavy core 0
                         );

  ESP_LOGI(TAG, "SPI sniffer ready");

  // D6: MOSFET gating to ground
  gpio_set_direction((gpio_num_t) PIN_D6, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t) PIN_D6, 0);   // LOW -- button off

  gpio_set_intr_type((gpio_num_t)PIN_D3, GPIO_INTR_ANYEDGE);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);  // safe to call once globally; ESPHome often already does, but it's OK if returns ESP_ERR_INVALID_STATE
  gpio_isr_handler_add((gpio_num_t)PIN_D3, &Sauna::d3_isr_trampoline, this);
}

void Sauna::process_byte_(uint8_t byte) {
  //ESP_LOGI(TAG, "0x%2X", byte);
  const uint32_t now = millis();

  if (!synced) {
    if (byte == SYNC) {
      synced = true;
      frame_idx = 0;
      frame[frame_idx++] = byte;
      cycles_since_sync = 0;
    } else {
      cycles_since_sync++;
      if (cycles_since_sync > 1000 && (cycles_since_sync % 1000) == 0) {
        if (running_binary_sensor_) running_binary_sensor_->publish_state(false);
        if (current_temp_sensor_)   current_temp_sensor_->publish_state(NAN);
        if (setpoint_temp_sensor_)  setpoint_temp_sensor_->publish_state(NAN);
        if (timer_sensor_)          timer_sensor_->publish_state(NAN);
      }
    }
    return;
  }

  frame[frame_idx++] = byte;
  if (frame_idx < 10) return;
  frame_idx = 0;

  // If we've got a full frame, process it
  // Do some sanity checks:
  // - First byte should be sync
  // - Every other byte should be bias (or whatever it is)
  if (frame[0] != SYNC ||
      frame[1] != BIAS || 
      frame[3] != BIAS || 
      frame[5] != BIAS ||
      frame[7] != BIAS || 
      frame[9] != BIAS) {
    synced = false;
    ESP_LOGI(TAG, "...bad frame [%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]", 
                    frame[0], frame[1], frame[2], frame[3], frame[4], 
                    frame[5], frame[6], frame[7], frame[8], frame[9]);
    return;
  }

  // Check for a blink frame.  If we're blinked off, start a countdown and bail on this frame.
  if ((frame[2] == 0xFF) || (frame[4] == 0xFF)) {
    blink_counter++;
    if (blink_counter > 5) blink_start_ms = now;
    return;
  }
  blink_counter = 0;
  int temp = digits[frame[2]] * 10 + digits[frame[4]];

  // If time is blinking, just ignore it
  if ((frame[6] != 0xFF) || (frame[8] != 0xFF)) {
     current_timer = digits[frame[6]] * 10 + digits[frame[8]];
  }

  // Check temp reading stability
  if (temp != last_temp) {
    stable_start_ms = now;
  }
  last_temp = temp;

  // If we're stable, process the temperature.
  // If we are blinking, the temp displays setpoint.  Otherwise current temp.
  if (now - stable_start_ms  > 1000) {
    if (now - blink_start_ms <  500) {
      setpoint_temp = temp;
    }
    else {
      current_temp = temp;
    }
  }


  //ESP_LOGI(TAG,"Temp: %d, Setpoint: %d, Timer: %d, Raw %d, Blink: %d, Stable: %d", current_temp, setpoint_temp, current_timer, temp, now - blink_start_ms, now - stable_start_ms);
  if (now - last_publish_ms > 500) {
    last_publish_ms = now;
    //ESP_LOGI(TAG,"Temp: %d, Setpoint: %d, Timer: %d", current_temp, setpoint_temp, current_timer);
    if ( running_binary_sensor_) running_binary_sensor_->publish_state(true);
    if (current_temp_sensor_ && current_temp >= 0) current_temp_sensor_->publish_state(current_temp);
    if (setpoint_temp_sensor_ && setpoint_temp >= 0) setpoint_temp_sensor_->publish_state(setpoint_temp);
    if (timer_sensor_) timer_sensor_->publish_state(current_timer);
  }
}

void Sauna::loop() {
  uint32_t start = millis();

  // Check if its time to turn off a button
  if (start > button_hold_until_ms) {
     button_selected = BUTTON_NO_PRESS;
  }

  // Process up to ~5ms worth per ESPHome loop iteration
  if (byte_queue == nullptr) return;
  while ((millis() - start) < 5) {
    uint8_t b;
    if (xQueueReceive(byte_queue, &b, 0) != pdTRUE) break;
    process_byte_(b);
  }
}

}  // namespace sauna
}  // namespace esphome


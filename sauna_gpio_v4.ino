#include <Arduino.h>

#include "driver/spi_slave.h"
#include "soc/gpio_sig_map.h"   // VSPICS0_IN_IDX etc.
#include "driver/gpio.h"
#include "soc/gpio_periph.h"

extern "C" {
  void gpio_matrix_in(uint32_t gpio, uint32_t signal_idx, bool inv);
}

// ----- Your pin mapping -----
static const int PIN_D0 = 13;   // sampled
static const int PIN_D1 = 14;   // MOSI (data into ESP32)
static const int PIN_D2 = 27;   // SCLK
static const int PIN_D3 = 26;   // "gate" -> use as CS (but inverted)
static const int PIN_D6 = 25;   // sampled

// ----- SPI host -----
static const spi_host_device_t HOST = VSPI_HOST;   // VSPI = SPI3 on classic ESP32

// We’ll queue many 1-byte transactions.
static constexpr int QUEUE_SIZE = 32;

static uint8_t rx_items[QUEUE_SIZE];
static spi_slave_transaction_t trans[QUEUE_SIZE];
static uint8_t rx_buf[QUEUE_SIZE]; // 1 byte per transaction

// Ring buffer for completed bytes (so we print from loop, not ISR)
static constexpr uint32_t RB_SIZE = 256;
static volatile uint8_t rb[RB_SIZE];
static volatile uint8_t rb_w = 0;
static volatile uint8_t rb_r = 0;
static portMUX_TYPE rbMux = portMUX_INITIALIZER_UNLOCKED;

static inline bool rb_push_isr(const uint8_t &in) {
  bool ok = false;
  portENTER_CRITICAL_ISR(&rbMux);
  uint8_t next = uint8_t(rb_w + 1);
  if (next != rb_r) {
    rb[rb_w] = in;
    rb_w = next;
    ok = true;
  }
  portEXIT_CRITICAL_ISR(&rbMux);
  return ok;
}

static inline bool rb_pop(uint8_t &out) {
  bool ok = false;
  portENTER_CRITICAL(&rbMux);
  if (rb_r != rb_w) {
    out = rb[rb_r];
    rb_r = uint8_t(rb_r + 1);
    ok = true;
  }
  portEXIT_CRITICAL(&rbMux);
  return ok;
}

// Called in ISR context after each transaction completes
static void IRAM_ATTR post_trans_cb(spi_slave_transaction_t *t) {
  // t->user holds index 0..QUEUE_SIZE-1
  uint32_t idx = (uint32_t) t->user;
  rb_push_isr(rx_buf[idx]);
}

static void queue_all_transactions() {
  for (int i = 0; i < QUEUE_SIZE; i++) {
    memset(&trans[i], 0, sizeof(trans[i]));
    trans[i].length = 8;                 // bits
    trans[i].rx_buffer = &rx_buf[i];
    trans[i].tx_buffer = nullptr;
    trans[i].user = (void*) (uintptr_t)i;

    esp_err_t err = spi_slave_queue_trans(HOST, &trans[i], portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("spi_slave_queue_trans failed: %d\n", (int)err);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  pinMode(PIN_D2, INPUT);
  pinMode(PIN_D3, INPUT);
  pinMode(PIN_D6, INPUT);

  // --- SPI bus config ---
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_D1;
  buscfg.miso_io_num = -1;     // not used
  buscfg.sclk_io_num = PIN_D2;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 1;  // 1 byte

  // --- SPI slave config ---
  spi_slave_interface_config_t slvcfg = {};
  slvcfg.spics_io_num = PIN_D3;      // we will invert this input via GPIO matrix
  slvcfg.flags = 0;
  slvcfg.queue_size = QUEUE_SIZE;
  slvcfg.mode = 3;                  // SPI mode 0: CPOL=0, CPHA=0 (matches your sampling on rising)
  slvcfg.post_trans_cb = post_trans_cb;

  // Initialize SPI slave (DMA auto channel)
  esp_err_t err = spi_slave_initialize(HOST, &buscfg, &slvcfg, 0);
  if (err != ESP_OK) {
    Serial.printf("spi_slave_initialize failed: %d\n", (int)err);
    while (true) delay(1000);
  }

  // ---- CRITICAL: invert CS so "D3 high" becomes "CS asserted" ----
  //
  // For VSPI (SPI3), CS0 input index is VSPICS0_IN_IDX.
  // gpio_matrix_in(gpio, signal_idx, invert)
  gpio_matrix_in(PIN_D3, VSPICS0_IN_IDX, true);

  Serial.println("SPI sniffer up. Queuing transactions...");
  queue_all_transactions();
  Serial.println("Ready.");
}

void loop() {
  // Drain ring buffer and print results
  uint8_t item;
  while (rb_pop(item)) {
    Serial.printf("0x%02X\n", item);
  }

  // Keep the SPI queue full: as transactions complete, we must re-queue them.
  // We do this by pulling completed transactions from driver and re-queueing.
  spi_slave_transaction_t *rt = nullptr;
  while (spi_slave_get_trans_result(HOST, &rt, 0) == ESP_OK) {
    // Re-queue same transaction (its rx_buffer points to rx_buf[idx] already)
    esp_err_t err = spi_slave_queue_trans(HOST, rt, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("re-queue failed: %d\n", (int)err);
    }
  }

  delay(1);
}

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "driver/spi_slave.h"
#include "soc/gpio_sig_map.h"   // VSPICS0_IN_IDX etc.
#include "driver/gpio.h"
#include "soc/gpio_periph.h"

extern "C" {
  void gpio_matrix_in(uint32_t gpio, uint32_t signal_idx, bool inv);
}

static const uint16_t TELNET_PORT = 23;
static const char* WIFI_SSID = "Go Buffs!";
static const char* WIFI_PASS = "shouldertoshoulder";

// ----- Your pin mapping -----
static const int PIN_D0 = 13;   // sampled
static const int PIN_D1 = 14;   // MOSI (data into ESP32)
static const int PIN_D2 = 27;   // SCLK
static const int PIN_D3 = 26;   // "gate" -> use as CS (but inverted)
static const int PIN_D6 = 25;   // sampled

// Translate values to digits
static const int SYNC = 0xEE;
static const int BIAS = 0xFF;
static int digits[256]; // initialized in setup

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

// ---- Telnet server ----
WiFiServer server(TELNET_PORT);
WiFiClient client;

static void ensureClient() {
  if (client && client.connected()) return;

  if (client) client.stop();
  client = server.available();
  if (client) {
    client.setNoDelay(true);
    client.println();
    client.println("Wilkomen das sauna sniffer");
    client.println();
  }
}

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

static inline bool rb_peek(uint8_t &out) {
  uint8_t r = rb_r;
  if (r == rb_w) return false;
  out = rb[r];
  return true;
}

// Called in ISR context after each transaction completes
static void IRAM_ATTR post_trans_cb(spi_slave_transaction_t *t) {

  // Ignore bogus / partial transfers
  if (t->trans_len != 8) return;

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

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi OK. IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  server.setNoDelay(true);

  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // Initialize our digit lookup
  for (int i = 0; i < 256; i++) {
      digits[i] = -1;
  }
  
  // Value displayed by segment displays
  digits[0x81] = 0;
  digits[0xE7] = 1;
  digits[0x49] = 2;
  digits[0x45] = 3;
  digits[0x27] = 4;
  digits[0x15] = 5;
  digits[0x31] = 6;
  digits[0xC7] = 7;
  digits[0x01] = 8;
  digits[0x05] = 9;
  digits[0xE6] = 10;
  digits[0x80] = 11;
  digits[0x48] = 12;

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
  // slvcfg.spics_io_num = PIN_D3;      // Routed manually later so we can invert
  slvcfg.spics_io_num = -1;
  slvcfg.flags = 0;
  slvcfg.queue_size = QUEUE_SIZE;
  slvcfg.mode = 3;                  // SPI mode 0: CPOL=0, CPHA=0 (matches your sampling on rising)
  slvcfg.post_trans_cb = post_trans_cb;

  // Initialize SPI slave (DMA auto channel)
  esp_err_t err = spi_slave_initialize(HOST, &buscfg, &slvcfg, 0);
  if (err != ESP_OK) {
    if (client && client.connected()) {
      Serial.printf("spi_slave_initialize failed: %d\n", (int)err);
    }
    while (true) delay(1000);
  }

  // ---- CRITICAL: invert CS so "D3 high" becomes "CS asserted" ----
  //
  // For VSPI (SPI3), CS0 input index is VSPICS0_IN_IDX.
  // gpio_matrix_in(gpio, signal_idx, invert)
  gpio_matrix_in(PIN_D3, VSPICS0_IN_IDX, true);
  if (client && client.connected()) {
    Serial.println("SPI sniffer up. Queuing transactions...");
  }
  queue_all_transactions();
  Serial.println("Ready.");
}

void loop() {
  static bool synced = false;
  static int  cycles_since_sync = 0;
  uint8_t fill = rb_w - rb_r;
  uint8_t byte;
  uint8_t frame[10];

  ensureClient();
  ArduinoOTA.handle();

// Look for our sync marker.  Leave it in place when we find it.
  if (not synced) {
    while (rb_peek(byte)) {
      if (byte == SYNC) {
        synced = true;
        cycles_since_sync = 0;
        break;
      }
      else {
        rb_pop(byte);
        cycles_since_sync++;
          // If we haven't seen sync in 1000 cycles, assume we're powered off
        if (cycles_since_sync > 1000)
        {
          if ((cycles_since_sync % 1000) == 0) {
            if (client && client.connected()) {
              client.printf("powered off (queue @ %d)\n", fill);
            }
          }
        }
      }
    }
  }
  // We're synchronized. Look for a full frame of 10
  else {
    if (fill >=10) {
      rb_pop(frame[0]);
      rb_pop(frame[1]);
      rb_pop(frame[2]);
      rb_pop(frame[3]);
      rb_pop(frame[4]);
      rb_pop(frame[5]);
      rb_pop(frame[6]);
      rb_pop(frame[7]);
      rb_pop(frame[8]);
      rb_pop(frame[9]);


      // Do some sanity checks:
      // - First byte should be sync
      // - Every other byte should be bias (or whatever it is)
      if ((frame[0] == SYNC) &&
          (frame[1] == BIAS) &&
          (frame[3] == BIAS) &&
          (frame[5] == BIAS) &&
          (frame[7] == BIAS) &&
          (frame[9] == BIAS))
      {
        int temp = digits[frame[2]] * 10 + digits[frame[4]];
        int time = digits[frame[6]] * 10 + digits[frame[8]];
        if (client && client.connected()) {
          client.printf("Temp: %d, Time: %d (queue @ %d)\n", temp, time, fill);
          if ((temp < 0) || (time < 0)) {
            client.printf("[%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]\n", 
              frame[0], frame[1], frame[2], frame[3], frame[4], 
              frame[5], frame[6], frame[7], frame[8], frame[9]);
          }
        }
      }
      else {
        synced = false;
        if (client && client.connected()) {
          client.printf("...bad frame [%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X] (queue @ %d)\n", 
                        frame[0], frame[1], frame[2], frame[3], frame[4], 
                        frame[5], frame[6], frame[7], frame[8], frame[9], 
                        fill);
        }
      }
    } // if we have enough bytes. else: just wait
  } // if we're synchronized. else: just wait

  // Keep the SPI queue full: as transactions complete, we must re-queue them.
  // We do this by pulling completed transactions from driver and re-queueing.
  spi_slave_transaction_t *rt = nullptr;
  while (spi_slave_get_trans_result(HOST, &rt, 0) == ESP_OK) {
    // Re-queue same transaction
    esp_err_t err = spi_slave_queue_trans(HOST, rt, portMAX_DELAY);
    if (err != ESP_OK) {
      if (client && client.connected()) {
        client.printf("re-queue failed: %d\n", (int)err);
      }
    }
  }
   
  // If telnet client disconnected, clean up
  if (client && !client.connected()) {
    client.stop();
  }

  delay(1);
}

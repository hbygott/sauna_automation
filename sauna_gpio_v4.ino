#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include <WiFi.h>
#include <ArduinoOTA.h>


// Pins for signals, names per logic analyzer
static const int PIN_D0 = 13;
static const int PIN_D1 = 14;
static const int PIN_D2 = 27;
static const int PIN_D3 = 26;
static const int PIN_D6 = 25;

// Translate values to digits
static const int SYNC = 0xEE;
static const int BIAS = 0xFF;
static int digits[256]; // initialized in setup


static const uint16_t TELNET_PORT = 23;
static const char* WIFI_SSID = "Go Buffs!";
static const char* WIFI_PASS = "shouldertoshoulder";

// --- capture state (ISR) ---
static volatile bool in_burst = false;
static volatile uint8_t bit_count = 0;
static volatile uint8_t cur_byte = 0;

// ring buffer of bytes
constexpr uint32_t RB_SIZE = 256;
static volatile uint8_t rb[RB_SIZE];
static volatile uint8_t rb_w = 0;
static volatile uint8_t rb_r = 0;

// lock-free SPSC push (ISR producer, loop consumer)
static inline void rb_push_byte_isr(uint8_t v) {
  uint8_t w = rb_w;
  uint8_t next = uint8_t(w + 1);      // wraps naturally at 256
  if (next == rb_r) return;           // full -> drop
  rb[w] = v;
  rb_w = next;
}
static inline bool rb_pop_byte(uint8_t &out) {
  uint8_t r = rb_r;
  if (r == rb_w) return false;
  out = rb[r];
  rb_r = uint8_t(r + 1);
  return true;
}

static inline bool rb_peek_byte(uint8_t &out) {
  uint8_t r = rb_r;
  if (r == rb_w) return false;
  out = rb[r];
  return true;
}

static inline uint32_t fastReadPin(uint8_t pin) {
  // works for pins < 32
  return (GPIO.in >> pin) & 1U;
}

void IRAM_ATTR on_gate_rise() {
  // D3 rising: begin a burst, reset bit assembly
  in_burst = true;
  bit_count = 0;
  cur_byte = 0;
}

void IRAM_ATTR on_clk_fall() {
  if (!in_burst) return;

  // sample data on D2 falling edge
  uint32_t d1 = fastReadPin(PIN_D1);

  cur_byte = (uint8_t)((cur_byte << 1) | (d1 & 1U));
  bit_count++;

  if (bit_count >= 8) {
    rb_push_byte_isr(cur_byte);
    in_burst = false;   // done for this burst; next burst starts at next D3 rise
  }
}


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
    client.println("Wilkomen das sauna sniffer v4");
    client.println();
  }
}


void setup() {
  Serial.begin(115200);

  // Configure pins as inputs (no pullups unless you *know* you want them)
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
  Serial.print("Telnet server on port ");
  Serial.println(TELNET_PORT);

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

  attachInterrupt(digitalPinToInterrupt(PIN_D3), on_gate_rise, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_D2), on_clk_fall, FALLING);
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
    while (rb_peek_byte(byte)) {
      if (byte == SYNC) {
        synced = true;
        cycles_since_sync = 0;
        break;
      }
      else {
        rb_pop_byte(byte);
        cycles_since_sync++;
          // If we haven't seen sync in 1000 cycles, assume we're powered off
        if (cycles_since_sync > 1000)
        {
          if ((cycles_since_sync % 1000) == 0) {
            if (client && client.connected()) {
              client.printf("powered off\n");
            }
          }
        }
      }
    }
  }
  // We're synchronized. Look for a full frame of 10
  else {
    if (fill >=10) {
      rb_pop_byte(frame[0]);
      rb_pop_byte(frame[1]);
      rb_pop_byte(frame[2]);
      rb_pop_byte(frame[3]);
      rb_pop_byte(frame[4]);
      rb_pop_byte(frame[5]);
      rb_pop_byte(frame[6]);
      rb_pop_byte(frame[7]);
      rb_pop_byte(frame[8]);
      rb_pop_byte(frame[9]);


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


 // If telnet client disconnected, clean up
  if (client && !client.connected()) {
    client.stop();
  }
  delay(1); // yield to anybody else
}

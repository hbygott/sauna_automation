#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

static const uint16_t TELNET_PORT = 23;
static const char* WIFI_SSID = "Go Buffs!";
static const char* WIFI_PASS = "shouldertoshoulder";

// Pins for signals, names per logic analyzer
static const int PIN_D0 = 13;   // Button addr
static const int PIN_D1 = 14;   // Button addr
static const int PIN_D2 = 27;   // Button addr
static const int PIN_D3 = 26;   // gate
static const int PIN_D6 = 25;   // button press

// If button is being sampled, D3 will be low, and D0-D2 address the button
//                                                     // signals  D2 D3 D1 D0
static const uint32_t BUTTON_MASK       = 0x0C006000;  //    pins  27 26 14 13
static const uint32_t BUTTON_RED        = 0x08002000;  //           1  0  0  1
static const uint32_t BUTTON_TIMER_UP   = 0x00002000;  //           0  0  0  1    
static const uint32_t BUTTON_TIMER_DOWN = 0x00000000;  //           0  0  0  0
static const uint32_t BUTTON_TEMP_UP    = 0x00006000;  //           0  0  1  1
static const uint32_t BUTTON_TEMP_DOWN  = 0x00004000;  //           0  0  1  0
static const uint32_t BUTTON_NO_PRESS   = 0xFFFFFFFFu; // sentinal value

static volatile uint32_t button_selected = BUTTON_NO_PRESS;
static volatile uint32_t hold_until_ms = 0;   // 0 means not holding

static volatile uint32_t last_match = 0;
static volatile uint32_t last_d3_level = 0;

// If the board is sampling our button, drive D6 low
// Otherwise either sampling is ended (rising edge) or we are sampling a different button.  Put D6 into high-impedence.
void IRAM_ATTR button_edge() {
  // last_d3_level = GPIO.in & (1UL << PIN_D3);
  if ((GPIO.in & BUTTON_MASK) == button_selected) {
    // last_match = button_selected;
    // clamp low
    GPIO.out_w1tc    = (1UL << PIN_D6);   // ensure low
    GPIO.enable_w1ts = (1UL << PIN_D6);   // enable output driver (sink)
  } else {
    // release (high-Z)
    GPIO.enable_w1tc = (1UL << PIN_D6);   // disable output driver
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
    client.println("Button Presser...maybe");
    client.println();
  }
}

void setup() {
  Serial.begin(115200);

  // Configure pins as inputs
  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  pinMode(PIN_D2, INPUT);
  pinMode(PIN_D3, INPUT);

  pinMode(PIN_D6, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_D6, LOW);              // “assert” value when enabled
  GPIO.enable_w1tc = (1UL << PIN_D6);     // start released (high-Z)

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

  attachInterrupt(digitalPinToInterrupt(PIN_D3), button_edge, CHANGE);
}

void loop() {
  ensureClient();
  ArduinoOTA.handle();

  // if (client && client.connected()) {
  //   client.printf("0x%x 0x%x\n", last_d3_level, last_match);
  // }


  if (client && client.connected()) {
    while (client.available()) {
      char c = (char)client.read();
      if (c == '\r' || c == '\n') continue;
      
      // Optional: echo
      client.print("key: ");
      client.println(c);

      switch (c) {
        case 'r': button_selected = BUTTON_RED;         hold_until_ms = millis() + 300; break;
        case 'u': button_selected = BUTTON_TIMER_UP;    hold_until_ms = millis() + 300; break;
        case 'd': button_selected = BUTTON_TIMER_DOWN;  hold_until_ms = millis() + 300; break;
        case 'i': button_selected = BUTTON_TEMP_UP;     hold_until_ms = millis() + 300; break;
        case 'k': button_selected = BUTTON_TEMP_DOWN;   hold_until_ms = millis() + 300; break;

        case 't':
          GPIO.enable_w1ts = (1UL << PIN_D6); // drive low
          delay(500);
          GPIO.enable_w1tc = (1UL << PIN_D6); // float
          delay(500);
          break;
        case 'y':
          detachInterrupt(digitalPinToInterrupt(PIN_D3));  // stop ISR from fighting you
          delay(100);
          pinMode(PIN_D6, OUTPUT);
          digitalWrite(PIN_D6, LOW);
          delay(500);
          pinMode(PIN_D6, INPUT);
          delay(100);
          pinMode(PIN_D6, OUTPUT_OPEN_DRAIN);
          digitalWrite(PIN_D6, LOW);              // “assert” value when enabled
          GPIO.enable_w1tc = (1UL << PIN_D6);     // start released (high-Z)
          delay(100);
          attachInterrupt(digitalPinToInterrupt(PIN_D3), button_edge, CHANGE);
          break;

        default:
          client.println("keys: r,u,d,i,k");
          break;
      } // switch
    } // available
  } // connected
  
  // Release button if time
  if (millis() > hold_until_ms) {
    button_selected = BUTTON_NO_PRESS;
  }
  
  if (client && !client.connected()) {
    client.stop();
  }
  delay(1); // yield to anybody else
}

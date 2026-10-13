#include <Arduino.h>
#include "app.h"
#include "cli.h"
#include "config.h"
#include "web_ui.h"

static void poll_button() {
  static bool prev = true;
  static uint32_t last_ms = 0;
  bool now = digitalRead(PIN_BUTTON);
  if (prev && !now && millis() - last_ms > 300) {
    last_ms = millis();
    Command c;
    c.type = app_telemetry().running ? CmdType::Stop : CmdType::Start;
    app_send(c);
  }
  prev = now;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  app_begin();
  web_ui_begin();
  Serial.printf("pathpilot ready, wifi '%s', open http://192.168.4.1\n", WIFI_AP_SSID);
}

void loop() {
  web_ui_loop();
  cli_loop();
  poll_button();
  delay(2);
}

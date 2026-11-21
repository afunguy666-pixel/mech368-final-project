#include "ui.h"

namespace {
#if HAS_LEDS
  const uint8_t LED_PIN[UI::LED_COUNT] = {PIN_LED_LINK, PIN_LED_ESTOP, PIN_LED_LOW};
#endif
  UI::Pattern pattern[UI::LED_COUNT] = {UI::P_OFF, UI::P_OFF, UI::P_OFF};
}

void UI::begin() {
#if HAS_LEDS
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PIN[i], OUTPUT);
    digitalWrite(LED_PIN[i], LOW);
  }
#endif
#if HAS_BUTTON
  pinMode(PIN_BTN, INPUT_PULLUP);
#endif
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  rgb(false, false, false);
}

void UI::set(Led led, Pattern p) { pattern[led] = p; }

void UI::rgb(bool r, bool g, bool b) {
  // 板载 RGB 低电平点亮
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

void UI::update() {
#if HAS_LEDS
  uint32_t now = millis();
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    bool on = false;
    switch (pattern[i]) {
      case P_ON:   on = true; break;
      case P_SLOW: on = (now / 500) & 1; break;
      case P_FAST: on = (now / 100) & 1; break;
      default:     break;
    }
    digitalWrite(LED_PIN[i], on ? HIGH : LOW);
  }
#endif
}

UI::Press UI::button() {
#if HAS_BUTTON
  static bool stable = HIGH, last = HIGH, longFired = false;
  static uint32_t tChange = 0, tDown = 0;
  uint32_t now = millis();
  bool r = digitalRead(PIN_BTN);
  if (r != last) { last = r; tChange = now; }
  if (now - tChange > 30 && r != stable) {      // 30 ms 消抖
    stable = r;
    if (stable == LOW) { tDown = now; longFired = false; return PRESS_DOWN; }
  }
  if (stable == LOW && !longFired && now - tDown >= BTN_LONG_MS) {
    longFired = true;
    return PRESS_LONG;
  }
#endif
  return PRESS_NONE;
}

#pragma once
#include "config.h"

// 状态灯、板上按键和 Nano 板载 RGB 灯
namespace UI {
  enum Led : uint8_t { LED_LINK, LED_ESTOP, LED_LOW, LED_COUNT };
  enum Pattern : uint8_t { P_OFF, P_ON, P_SLOW, P_FAST };
  enum Press : uint8_t { PRESS_NONE, PRESS_DOWN, PRESS_LONG };

  void begin();
  void set(Led led, Pattern p);
  void rgb(bool r, bool g, bool b);   // Nano 板载 RGB，两种板都有
  void update();                      // 每次 loop 调用，刷新闪烁
  Press button();                     // 按下瞬间返回 PRESS_DOWN，按满 BTN_LONG_MS 返回 PRESS_LONG
}

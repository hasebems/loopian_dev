/* ========================================
 *
 *  loopian: white led
 *    description: white led
 *    for Arduino Leonardo / Sparkfun pro micro
 *
 *  Copyright(c)2023- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#ifndef WHITE_LED_H
#define WHITE_LED_H
 
#include <Arduino.h>
#include "configuration.h"

constexpr int MAX_EACH_LIGHT = 16;

class WhiteLed {

  long _total_time;   // *2msec(MINIMUM_RESOLUTION)
  int  _light_lvl[MAX_EACH_LIGHT*MAX_DEVICE_MBR3110];

public:
  WhiteLed(void): _total_time(0), _light_lvl() {}
  void clear_all(void);
  int gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV]);
  void one_kamaboco(int kamanum);
  void light_led_each(int num, uint16_t count);
};
#endif
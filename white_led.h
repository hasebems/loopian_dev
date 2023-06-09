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
constexpr int FADE_RATE = 2; // *MINIMUM_RESOLUTION[msec] で 3/4

class WhiteLed {

  long  _total_time;   // *10msec
  int   _fade_counter;
  int   _light_lvl[MAX_EACH_LIGHT*MAX_DEVICE_MBR3110];
  int   _light_lvl_itp[MAX_EACH_LIGHT*MAX_DEVICE_MBR3110];

public:
  WhiteLed(void): _total_time(0), _fade_counter(0), _light_lvl(), _light_lvl_itp() {}
  void clear_all(void);
  int gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV]);
  void one_kamaboco(int kamanum);
  void light_led_each(int num, uint16_t count);
};
#endif
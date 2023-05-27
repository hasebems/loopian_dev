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
#include "white_led.h"
#include "i2cdevice.h"

/*----------------------------------------------------------------------------*/
//     White LED Control
/*----------------------------------------------------------------------------*/
void WhiteLed::clear_all(void)
{
  for (int j=0; j<MAX_DEVICE_MBR3110; j++){
    pca9544_changeI2cBus(3,j);
    for (int i=0; i<MAX_EACH_LIGHT; i++){
      light_led_each(i, 0);
    }
    pca9544_changeI2cBus(1,j);
  }
}
int WhiteLed::gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV])
{
  _total_time += difftm;

  //for (int x=0; x<MAX_EACH_LIGHT*MAX_DEVICE_MBR3110; x++){_light_lvl[x]=0;}
  memset(&_light_lvl[0], 0, sizeof(int)*MAX_EACH_LIGHT*MAX_DEVICE_MBR3110);

  // tchev : 0-1599 + 1600*kamanum で絶対位置が表現され、イベントごとにその数値が入力される
  int max_ev = 0;
  for (int i=0; i<MAX_TOUCH_EV; i++){
    if (tchev[i] == -1){break;}
    int frac = tchev[i]%100;
    int pos = tchev[i]/100;
    for (int j=0; j<2; j++){
      //  触った箇所の前後二つのLEDが点灯する
      _light_lvl[pos+1+j] += (frac+100)>j*100? (frac+100)-j*100: 0;    // 199 - 0
      if (pos>=j){
        _light_lvl[pos-j] += (199-frac)>j*100? (199-frac)-j*100: 0;
      }
    }
    max_ev += 1;
  }

  for (int j=0; j<MAX_DEVICE_MBR3110; j++){one_kamaboco(j);}
  return max_ev;
}
void WhiteLed::one_kamaboco(int kamanum)
{
  uint16_t time = static_cast<uint16_t>(_total_time/5);
  const int offset_num = kamanum*MAX_EACH_LIGHT;
  pca9544_changeI2cBus(3,kamanum);

  for (int i=0; i<MAX_EACH_LIGHT; i++){
    if (_light_lvl[i+offset_num] > 0){
      int strength = 20*_light_lvl[i+offset_num];
      if (strength > 4000){strength = 4000;}
      light_led_each(i, strength);
    }
    else {
      // 背景で薄く光っている
      int ptn = (time+(4*i))%64;
      ptn = ptn<32? ptn:64-ptn;
      light_led_each(i, ptn);
    }
  }
  pca9544_changeI2cBus(1,kamanum); // 別のI2Cバスに変えないと、他のkamanumのときに上書きされてしまう
}
void WhiteLed::light_led_each(int num, uint16_t strength){ // strength=0-4095
  int err;
  uint8_t adrs = num * 4 + 0x06;
	err = PCA9685_write( 0, adrs, 0 );          // ONはtime=0
	err = PCA9685_write( 0, adrs+1, 0 );        // ONはtime=0
	err = PCA9685_write( 0, adrs+2, (uint8_t)(strength & 0x00ff) );// OFF 0-4095 (0-0x0fff) の下位8bit
	err = PCA9685_write( 0, adrs+3, (uint8_t)(strength>>8) );      // OFF 上位4bit
}
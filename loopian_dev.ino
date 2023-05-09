/* ========================================
 *
 *  loopian main
 *    description: Main Loop
 *    for Arduino Leonardo / Sparkfun pro micro
 *
 *  Copyright(c)2023- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#include  "configuration.h"

#include  <Arduino.h>
#include  <MsTimer2.h>

#include  "i2cdevice.h"
#include  "global_timer.h"
#include  "white_led.h"

#define LED_ERR       10   //  7: J3:3
#define LED1          A3   // 21: J3:1
#define LED2          A2   // 20: J3:2
#define JOYSTICK_X    A1   // 19: J10
#define JOYSTICK_Y    A0   // 18: J11
#define LED_ALL_ON    9    // 10: J4

/*----------------------------------------------------------------------------*/
//     Struct
/*----------------------------------------------------------------------------*/
struct TouchEvent {
  int _locate_current;  // -1, 0 - 9599 (16*6*100 - 1)
  int _locate_target;   // -1, 0 - 9599
  int _first_touch;     // -1, 0 - 7
  int _last_touch;      // -1, 0 - 7
  int _last_midi;
  int _time;
  TouchEvent(void): 
    _locate_current(-1),
    _locate_target(-1),
    _first_touch(-1),
    _last_touch(-1),
    _time(-1) {}
  TouchEvent& operator=(const TouchEvent& te){
    _locate_current = te._locate_current;
    _locate_target = te._locate_target;
    _first_touch = te._first_touch;
    _last_touch = te._last_touch;
    _time = te._time;
    return *this;
  }
};
/*----------------------------------------------------------------------------*/
//     Variables
/*----------------------------------------------------------------------------*/
constexpr int HOLD_TIME = 10;  // *10msec この間、一度でもonならonとする。離す時少し鈍感にする。最大16

int counter = 0;
unsigned long seconds_old = 0;
GlobalTimer gt;
WhiteLed wled;
TouchEvent ev[MAX_TOUCH_EV];

int maxCapSenseDevice;
bool availableEachDevice[MAX_DEVICE_MBR3110];
uint16_t sw[MAX_DEVICE_MBR3110][MAX_EACH_SENS] = {0};
int holdtime_cnt = 0;

/*----------------------------------------------------------------------------*/
//     setup/loop
/*----------------------------------------------------------------------------*/
void setup()
{
  wireBegin();   // Join I2C bus
  pca9544_changeI2cBus(3,0);
  PCA9685_init(0);

  ada88_init();
  ada88_writeNumber(0);

  pinMode(LED_ALL_ON,OUTPUT);
  digitalWrite(LED_ALL_ON, HIGH); // White LED All Off

  pinMode(LED1,OUTPUT);       // LED1: Touch Status
  pinMode(LED2,OUTPUT);       // LED2: Wait Status
  pinMode(LED_ERR,OUTPUT);
  for (int i=0; i<3; i++){    // means starting...
    delay(200);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED_ERR, LOW);
    delay(200);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED_ERR, HIGH);
  }
  digitalWrite(LED2, LOW);  // LED On: means during setup()

  for (int i=0; i<MAX_DEVICE_MBR3110; ++i){
    availableEachDevice[i] = true;
  }
  maxCapSenseDevice = MAX_DEVICE_MBR3110;

  uint16_t errNum = 0;
  int err;
#ifdef USE_CY8CMBR3110
#if SETUP_MODE // CapSense Setup Mode
  for (int i=0; i<maxCapSenseDevice; ++i){
    pca9544_changeI2cBus(0,i);
    err = MBR3110_setup(i);
    if (err){
      availableEachDevice[i] = false;
      digitalWrite(LED_ERR, HIGH);
      errNum += 0x0001<<i;
    }
  }
  ada88_writeBit(errNum);
  delay(2000);          // if something wrong, 2sec LED_ERR on
  for (int i=0; i<3; i++){  // when finished, flash 3times.
    digitalWrite(LED_ERR, LOW);
    delay(100);
    digitalWrite(LED_ERR, HIGH);
    delay(100);
  }
  digitalWrite(LED_ERR, LOW);
  while(1);
#else
  //  Normal Mode
  errNum = 0;
  for (int i=0; i<maxCapSenseDevice; ++i){
    pca9544_changeI2cBus(0,i);
    err = MBR3110_init(i);
    if (err){
      availableEachDevice[i] = false;
      errNum += 0x0001<<i;
    }
  }
  if (errNum){
    //  if err, stop 5sec.
    digitalWrite(LED_ERR, HIGH);
    ada88_writeBit(errNum);
    delay(5000);  //  5sec LED_ERR on
    digitalWrite(LED_ERR, LOW);
  }
#endif
#endif

  //  Set Interrupt
  MsTimer2::set(MINIMUM_RESOLUTION, flash);     // 10ms Interval Timer Interrupt
  MsTimer2::start();

  // the display is now on
  digitalWrite(LED_ALL_ON, LOW);  //  All White LED available
  digitalWrite(LED2, HIGH);
}
/*----------------------------------------------------------------------------*/
void loop() {
  //  Global Timer 
  long difftm = generateTimer();

  if ( gt.timer10msecEvent() ){
    // check active
    if (gt.timer100ms()%2){digitalWrite(LED2, LOW);}
    else {digitalWrite(LED2, HIGH);}
    holdtime_cnt += 1;
    if (holdtime_cnt>=HOLD_TIME){holdtime_cnt=0;}

 #ifdef USE_CY8CMBR3110
    //  Touch Sensor
    int errNum = 0;
    bool light_someone = false;
    for (int i=0; i<maxCapSenseDevice; ++i){
      if (availableEachDevice[i] == true){
        uint8_t swtmp[2] = {0};
        pca9544_changeI2cBus(0,i);
        int err = MBR3110_readTouchSw(swtmp,i);
        if (err){
          errNum += 0x01<<i;
        }
        uint16_t bptn = (((uint16_t)swtmp[1])<<8) + swtmp[0];
        for (int j=0; j<MAX_EACH_SENS; j++){
          if (bptn & (0x0001<<j)){
            sw[i][j] |= 0x0001<<holdtime_cnt;
          }
          else {
            sw[i][j] &= ~(0x0001<<holdtime_cnt);
          }
        }
      }
    }
    if (light_someone){digitalWrite(LED1, LOW);}
    else {digitalWrite(LED1, HIGH);}
    update_touch_target();
 #endif
  }

  //  update touch location
  interporate_location(difftm);
  int tchev[MAX_TOUCH_EV];
  for (int i=0; i<MAX_TOUCH_EV; i++){
    tchev[i] = ev[i]._locate_current;
  }

  // Light White LED
  int max_ev = wled.gen_lighting_in_loop(difftm, tchev);

  // for debug
  if (ev[0]._locate_current>=0){
    ada88_writeNumber(ev[0]._locate_current);
  }
  else {
    ada88_writeBit(0);
  }
}
/*----------------------------------------------------------------------------*/
//     Global Timer
/*----------------------------------------------------------------------------*/
void flash()
{
  gt.incGlobalTime();
}
long generateTimer( void )
{
  uint16_t  gTime = gt.globalTime();
  long diff = gTime - gt.gtOld();
  gt.setGtOld(gTime);
  if ( diff < 0 ){ diff += 0x10000; }

  gt.clearAllTimerEvent();
  gt.updateTimer(diff);
  return diff;
}
/*----------------------------------------------------------------------------*/
//     calcurate finger location
/*----------------------------------------------------------------------------*/
void update_touch_target(void){
  TouchEvent new_ev[MAX_TOUCH_EV];
  bool start=false;
  int start_i = 0;

  // new_ev の生成
  for (int e=0; e<MAX_TOUCH_EV; e++){
    for (int i=start_i; i<MAX_DEVICE_MBR3110*MAX_EACH_SENS; i++){
      int which_dev=i/MAX_EACH_SENS;
      int each_sw=i%MAX_EACH_SENS;
      if (sw[which_dev][each_sw] != 0){
        if (!start){
          start = true;
          new_ev[e]._first_touch = which_dev*MAX_EACH_SENS + each_sw;
         }
      }
      else {
        if (start){
          start = false;
          new_ev[e]._last_touch = which_dev*MAX_EACH_SENS + each_sw - 1;
          new_ev[e]._locate_target = (new_ev[e]._first_touch + new_ev[e]._last_touch)*100; // *200/2
          start_i = i+1;
          break;
        }
      }
    }
  }
  // ev[]とnew_ev[]を照合して、current/time をコピー
  constexpr int SAME_LOCATE = 160;  // 100 means next
  for (int x=0; x<MAX_TOUCH_EV; x++){
    int new_target = new_ev[x]._locate_target;
    if (new_target == -1){break;}
    bool found = false;
    for (int y=0; y<MAX_TOUCH_EV; y++){
      int crnt_target = ev[y]._locate_target;
      if (crnt_target == -1){break;}
      if ((crnt_target-SAME_LOCATE < new_target) && (new_target < crnt_target+SAME_LOCATE)){
        new_ev[x]._locate_current = ev[y]._locate_current;
        new_ev[x]._time = ev[y]._time;
        found = true;
        break;
      }
    }
    if (!found){
      new_ev[x]._locate_current = new_ev[x]._locate_target;
      new_ev[x]._time = 0;
      new_ev[x]._last_midi = new_ev[x]._locate_current;
      generate_midi(new_ev[x]._last_midi);
    }
  }
  // copy
  memcpy(ev,new_ev,sizeof(TouchEvent)*MAX_TOUCH_EV);
}
//current を target に近づける
void interporate_location(long difftm)
{
  constexpr int RATE = 5; // locate diff / 2msec
  for (int i=0; i<MAX_TOUCH_EV; i++){
    int target = ev[i]._locate_target;
    if (target==-1){break;}
    int diff = target - ev[i]._locate_current;
    if (diff>0){
      diff = difftm*RATE>diff? diff:difftm*RATE;
    }
    else if (diff<0){
      diff = difftm*RATE>(-diff)? diff:-difftm*RATE;
    }
    ev[i]._locate_current += diff;
    ev[i]._time += difftm;
  }
}
/*----------------------------------------------------------------------------*/
//     generate midi event
/*----------------------------------------------------------------------------*/
void generate_midi(int locate){}


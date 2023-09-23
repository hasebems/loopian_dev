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
#include  <MIDI.h>
#include  <MIDIUSB.h>

#include  "i2cdevice.h"
#include  "global_timer.h"
#include  "white_led.h"

#define LED_ERR       10   //  7: J3:3
#define LED1          A3   // 21: J3:1
#define LED2          A2   // 20: J3:2
#define JOYSTICK_X    A1   // 19: J10
#define JOYSTICK_Y    A0   // 18: J11
#define JOYSTICK_SW   8    // 11:
#define LED_ALL_ON    9    // 12: J4
#define SETUP_MODE    6    // 9:  Displayの端の端子

constexpr int NOTHING = -1;
constexpr int COLLATED = -2;  // 照合済

/*----------------------------------------------------------------------------*/
//     Parameter
/*----------------------------------------------------------------------------*/
constexpr int LED_CHASE_SPEED = 10; // locate diff / 2msec
constexpr int SAME_FINGER = 210;  // 100 means next value (200means next sensor)/10msec
                                  // 同じ指とみなす速さ

/*----------------------------------------------------------------------------*/
//     Struct
/*----------------------------------------------------------------------------*/
struct TouchEvent {
  int _locate_current;  // -1, 0 - 9599 (16*6*100 - 1)
  int _locate_target;   // -1, 0 - 9599
  int _mintch_locate;   // -1, 0 - 47 (8*6 - 1)
  int _maxtch_locate;   // -1, 0 - 47
  int _last_midi;       // 0 - 95 (locate/100)
  int _time;
  TouchEvent(void): 
    _locate_current(NOTHING),
    _locate_target(NOTHING),
    _mintch_locate(NOTHING),
    _maxtch_locate(NOTHING),
    _last_midi(NOTHING),
    _time(NOTHING) {}
  TouchEvent& operator=(const TouchEvent& te){
    _locate_current = te._locate_current;
    _locate_target = te._locate_target;
    _mintch_locate = te._mintch_locate;
    _maxtch_locate = te._maxtch_locate;
    _last_midi = te._last_midi;
    _time = te._time;
    return *this;
  }
};
/*----------------------------------------------------------------------------*/
//     Variables
/*----------------------------------------------------------------------------*/
MIDI_CREATE_DEFAULT_INSTANCE();

bool setup_mode = false;
constexpr int HOLD_TIME = 10;  // *10msec この間、一度でもonならonとする。離す時少し鈍感にする。最大16

int counter = 0;
unsigned long seconds_old = 0;
GlobalTimer gt;
WhiteLed wled;
TouchEvent ev[MAX_TOUCH_EV];

bool availableEachDevice[MAX_DEVICE_MBR3110];
uint16_t sw[MAX_DEVICE_MBR3110][MAX_EACH_SENS] = {0};
int holdtime_cnt = 0; // 指を離したときの感度を弱めに（反応を遅めに）にするためのカウンタ

/*----------------------------------------------------------------------------*/
//     CY8CMBR3110 setup mode / check White LED
/*----------------------------------------------------------------------------*/
void check_and_setup_board(void)
{
  int err;
  int j1_7_sw = digitalRead(SETUP_MODE);

  //  Check White LED
  if (!j1_7_sw){
    ada88_write(2);//"B"
    delay(200);
    for(int f=0; f<MAX_EACH_LIGHT; f++){wled.light_led_each(f,0);}
    digitalWrite(LED_ALL_ON, LOW);  //  All White LED available
    while(1){
      for(int l=0; l<2; l++){
        for(int e=0; e<MAX_EACH_LIGHT; e++){
          uint16_t bright = (e%2)==0?l:(l+1)%2;
          wled.light_led_each(e,bright*200);
        }
        delay(200);
      }
    } //  無限ループ
  }

  // CapSense Setup Mode
  bool sup_ok = false;
  ada88_write(21);//"SU"
  for (int i=0; i<MAX_DEVICE_MBR3110; ++i){
    pca9544_changeI2cBus(0,i);
    err = MBR3110_setup(i);
    if (err){
      digitalWrite(LED_ERR, LOW); // turn on
    }
    else{
      ada88_write(22); // "Ok"
      sup_ok = true;
      break;
    }
  }

  if (!sup_ok){ada88_write(23);} // "Er"
  delay(2000);          // if something wrong, 2sec LED_ERR on

  for (int i=0; i<3; i++){  // when finished, flash 3times.
    digitalWrite(LED_ERR, LOW);
    delay(100);
    digitalWrite(LED_ERR, HIGH);
    delay(100);
  }
  if (!sup_ok){digitalWrite(LED_ERR, LOW);}

  while(1);
}
/*----------------------------------------------------------------------------*/
//     setup
/*----------------------------------------------------------------------------*/
void setup()
{
  //+++++++++++++++++++++++++++++++++
  //  MIDI settings
  MIDI.setHandleNoteOff(handlerNoteOff);
  MIDI.setHandleNoteOn(handlerNoteOn);
  MIDI.setHandleControlChange(handlerCC);
  MIDI.begin();
  MIDI.turnThruOff();

  //+++++++++++++++++++++++++++++++++
  //  I2C/device settings
  wireBegin();   // Join I2C bus
  for (int k=0; k<MAX_DEVICE_MBR3110; k++){
    pca9544_changeI2cBus(3,k);
    PCA9685_init(0);
  }
  ada88_init();
  ada88_write(0); // nothing

  //+++++++++++++++++++++++++++++++++
  //  GPIO settings
  pinMode(LED_ALL_ON,OUTPUT);
  digitalWrite(LED_ALL_ON, HIGH); // White LED All Off
  pinMode(SETUP_MODE,INPUT);
  pinMode(JOYSTICK_SW,INPUT);
  int joystick_sw = digitalRead(JOYSTICK_SW);
  if (joystick_sw == 0){setup_mode = true;}

  pinMode(LED1,OUTPUT);       // LED1: Touch Status
  pinMode(LED2,OUTPUT);       // LED2: Wait Status
  pinMode(LED_ERR,OUTPUT);
  for (int i=0; i<3; i++){    // 3times lighting means starting...
    delay(100);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED_ERR, LOW);
    delay(100);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED_ERR, HIGH);
  }
  digitalWrite(LED2, LOW);  // LED On: means during setup()

  //+++++++++++++++++++++++++++++++++
  //  Init global variables
  for (int i=0; i<MAX_DEVICE_MBR3110; ++i){
    availableEachDevice[i] = true;
  }

  //+++++++++++++++++++++++++++++++++
  //  local variables
  uint16_t errBit = 0;
  int err;

  //+++++++++++++++++++++++++++++++++
  //  setup mode
  if (setup_mode){check_and_setup_board();}
  //  戻ってこない

  //+++++++++++++++++++++++++++++++++
  //  normal mode
  else {
    for (int i=0; i<MAX_DEVICE_MBR3110; ++i){
      pca9544_changeI2cBus(0,i);
      err = MBR3110_init(i);
      if (err){
        availableEachDevice[i] = false;
        errBit += 0x0001<<i;
      }
    }
    //  if err, stop 3sec.
    digitalWrite(LED_ERR, HIGH);
    ada88_writeBit(~errBit);
    delay(3000);  //  3sec LED_ERR on
    digitalWrite(LED_ERR, LOW);
  }

  //+++++++++++++++++++++++++++++++++
  //  Set Interrupt
  MsTimer2::set(MINIMUM_RESOLUTION, flash);     // 10ms Interval Timer Interrupt
  MsTimer2::start();

  //+++++++++++++++++++++++++++++++++
  //  turn LEDs on
  wled.clear_all();
  digitalWrite(LED_ALL_ON, LOW);  //  All White LED available
  digitalWrite(LED2, HIGH);
}
/*----------------------------------------------------------------------------*/
//     loop
/*----------------------------------------------------------------------------*/
void loop() {
  //  Global Timer 
  long difftm = generateTimer();

  //  MIDI Receive
  receiveMidi();

  if ( gt.timer10msecEvent() ){
    // check active
    if (gt.timer100ms()%2){digitalWrite(LED2, LOW);}
    else {digitalWrite(LED2, HIGH);}
    holdtime_cnt += 1;
    if (holdtime_cnt>=HOLD_TIME){holdtime_cnt=0;}

    //  Touch Sensor
    int errNum = 0;
    bool light_someone = false;
    for (int i=0; i<MAX_DEVICE_MBR3110; ++i){
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
            light_someone= true;
          }
          else {  // 立てたビットを全部お掃除しないと Off にならない
            sw[i][j] &= ~(0x0001<<holdtime_cnt);
          }
        }
      }
    }
    if (light_someone){digitalWrite(LED1, LOW);}
    else {digitalWrite(LED1, HIGH);}
    update_touch_target();
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
    ada88_writeNumber(ev[0]._locate_current/10);  // 0-959
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
// 連続するタッチオンの検出
void extract_finger(TouchEvent (&new_ev)[MAX_TOUCH_EV])
{
  bool start=false;
  int start_i = 0;

  // new_ev の生成
  for (int e=0; e<MAX_TOUCH_EV; e++){
    //  下から上まで端子の状態を走査しながら、タッチされた端子が連続している箇所を探す
    int i=start_i;
    while (i<MAX_DEVICE_MBR3110*MAX_EACH_SENS) {
      int which_dev=i/MAX_EACH_SENS;
      int each_sw=i%MAX_EACH_SENS;
      if (sw[which_dev][each_sw] != 0){
        if (!start){
          start = true;
          new_ev[e]._mintch_locate = which_dev*MAX_EACH_SENS + each_sw;
         }
      }
      else {
        if (start){
          start = false;
          new_ev[e]._maxtch_locate = which_dev*MAX_EACH_SENS + each_sw - 1;
          new_ev[e]._locate_target = (new_ev[e]._mintch_locate + new_ev[e]._maxtch_locate)*100; // *200/2
          start_i = i+1;
          break;
        }
      }
      i+=1;
    }
    if (start){ // 最後のセンサ
      new_ev[e]._maxtch_locate = MAX_DEVICE_MBR3110*MAX_EACH_SENS - 1;
      new_ev[e]._locate_target = (new_ev[e]._mintch_locate + new_ev[e]._maxtch_locate)*100; // *200/2
    }
  }
}
int update_touch_target(void)
{
  int target_num = 0;
  TouchEvent new_ev[MAX_TOUCH_EV];

  // 指と判断できるイベント抽出
  extract_finger(new_ev);

  // ev[]とnew_ev[]を照合して、Note Event を生成
  for (int x=0; x<MAX_TOUCH_EV; x++){
    int new_target = new_ev[x]._locate_target;
    if (new_target == NOTHING){break;}
    bool found = false;
    for (int y=0; y<MAX_TOUCH_EV; y++){
      int crnt_target = ev[y]._locate_target;
      if (crnt_target == NOTHING){break;}
      if (crnt_target == COLLATED){continue;}
      if ((crnt_target-SAME_FINGER < new_target) && (new_target < crnt_target+SAME_FINGER)){
        new_ev[x]._locate_current = ev[y]._locate_current;
        new_ev[x]._time = ev[y]._time;
        ev[y]._locate_target = COLLATED;
        found = true;
        new_ev[x]._last_midi = new_ev[x]._locate_target/100;
        if (new_ev[x]._last_midi != ev[y]._last_midi){
          generate_midi(1, new_ev[x]._last_midi, ev[y]._last_midi);
        }
        target_num = x;
        break;
      }
    }
    if (!found){ // on:new, off:old -> note on
      new_ev[x]._locate_current = new_target;
      new_ev[x]._time = 0;
      new_ev[x]._last_midi = new_ev[x]._locate_current/100;
      generate_midi(0, new_ev[x]._last_midi, NOTHING);
    }
  }
  for (int z=0; z<MAX_TOUCH_EV; z++){ // off:new, on:old -> note off
    int crnt_target = ev[z]._locate_target;
    if (crnt_target == NOTHING){break;}
    if (crnt_target == COLLATED){continue;}
    else {generate_midi(2, ev[z]._last_midi, NOTHING);}
  }
  // copy
  memcpy(ev,new_ev,sizeof(TouchEvent)*MAX_TOUCH_EV);
  //for (int c=0; c<MAX_TOUCH_EV; c++){ev[c] = new_ev[c];}
  return target_num;
}
//current を target に近づける
void interporate_location(long difftm)
{
  for (int i=0; i<MAX_TOUCH_EV; i++){
    int target = ev[i]._locate_target;
    if (target==-1){break;}
    int diff = target - ev[i]._locate_current;
    if (diff>0){
      diff = difftm*LED_CHASE_SPEED>diff? diff:difftm*LED_CHASE_SPEED;
    }
    else if (diff<0){
      diff = difftm*LED_CHASE_SPEED>(-diff)? diff:-difftm*LED_CHASE_SPEED;
    }
    ev[i]._locate_current += diff;
    ev[i]._time += difftm;
  }
}
/*----------------------------------------------------------------------------*/
//     MIDI Out
/*----------------------------------------------------------------------------*/
void setMidiNoteOn(uint8_t note, uint8_t vel)
{
  //MIDI.sendNoteOn(note, vel, 1);
  midiEventPacket_t event = {0x09, 0x90, note, vel};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}
/*----------------------------------------------------------------------------*/
void setMidiNoteOff(uint8_t note)
{
  //MIDI.sendNoteOff(note, vel, 1);
  midiEventPacket_t event = {0x09, 0x90, note, 0};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}
/*----------------------------------------------------------------------------*/
void setMidiControlChange(uint8_t controller, uint8_t value)
{
  //MIDI.sendControlChange(controller, value, 1);
  midiEventPacket_t event = {0x0b, 0xb0, controller, value};
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}
/*----------------------------------------------------------------------------*/
//     generate midi event
/*----------------------------------------------------------------------------*/
void generate_midi(int type, int locate, int last_locate){
  constexpr int CHECK_NOTE = 0;
  switch(type){
    case 0:{
      setMidiNoteOn(locate+CHECK_NOTE, 100);
      break;
    }
    case 1:{
      setMidiNoteOn(locate+CHECK_NOTE, 100);
      setMidiNoteOff(last_locate+CHECK_NOTE);
      break;
    }
    case 2:{
      setMidiNoteOff(locate+CHECK_NOTE);
      break;
    }
  }
}

/*----------------------------------------------------------------------------*/
//      Serial MIDI In
/*----------------------------------------------------------------------------*/
void receiveMidi(void){
  //MIDI.read();
  midiEventPacket_t rx = MidiUSB.read();
}
/*----------------------------------------------------------------------------*/
void handlerNoteOn(byte channel , byte note , byte vel)
{
  if (channel == 1){
    //tchkbd.makeKeySwEvent(note, true, vel);
  }
}
/*----------------------------------------------------------------------------*/
void handlerNoteOff(byte channel , byte note , byte vel)
{
  if (channel == 1){
    //tchkbd.makeKeySwEvent(note, false, vel);
  }
}
/*----------------------------------------------------------------------------*/
void handlerCC(byte channel , byte number , byte value)
{
  if (channel == 1){
    //setMidiControlChange(number, value);
  }
}
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

#include <Arduino.h>

#include  <MsTimer2.h>
#include  "i2cdevice.h"
#include  "loopian_main.h"

#define LED_ERR       10   //  7: J3:3
#define LED1          A3   // 21: J3:1
#define LED2          A2   // 20: J3:2
#define JOYSTICK_X    A1   // 19: J10
#define JOYSTICK_Y    A0   // 18: J11
#define LED_ALL_ON    9    // 10: J4

/*----------------------------------------------------------------------------*/
//     Variables
/*----------------------------------------------------------------------------*/
// assume the display is off until configured in setup()
bool            isDisplayVisible        = false;

// declare size of working string buffers. Basic strlen("d hh:mm:ss") = 10
const size_t    MaxString               = 16;

// the string being displayed on the SSD1331 (initially empty)
char oldTimeString[MaxString]           = { 0 };

// variables
int counter = 0;
unsigned long seconds_old = 0;
GlobalTimer gt;

int maxCapSenseDevice;
bool availableEachDevice[MAX_DEVICE_MBR3110];

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
  isDisplayVisible = true;
  digitalWrite(LED_ALL_ON, LOW);  //  All White LED available
  digitalWrite(LED2, HIGH);
}
/*----------------------------------------------------------------------------*/
void loop() {
  //  Global Timer 
  generateTimer();

  if ( gt.timer10msecEvent() ){
    // check active
    if (gt.timer100ms()%2){digitalWrite(LED2, LOW);}
    else {digitalWrite(LED2, HIGH);}    

 #ifdef USE_CY8CMBR3110
    //  Touch Sensor
    uint16_t sw[MAX_DEVICE_MBR3110] = {0};
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
        sw[i] = (((uint16_t)swtmp[1])<<8) + swtmp[0];
        //setAda88_Number(sw[i]*10); //$$$
        light_someone = true;
      }
    }
    if (light_someone){digitalWrite(LED1, LOW);}
 #endif
  }

  // Light White LED
  white_led_cntrl(100);
}
/*----------------------------------------------------------------------------*/
//     Global Timer
/*----------------------------------------------------------------------------*/
void flash()
{
  gt.incGlobalTime();
}
void generateTimer( void )
{
  uint16_t  gTime = gt.globalTime();
  long diff = gTime - gt.gtOld();
  gt.setGtOld(gTime);
  if ( diff < 0 ){ diff += 0x10000; }

  gt.clearAllTimerEvent();
  gt.updateTimer(diff);
}
/*----------------------------------------------------------------------------*/
//     White LED Control
/*----------------------------------------------------------------------------*/
void white_led_cntrl(int lednum)
{
  uint16_t time = gt.timer10ms();
  for (int i=0; i<16; i++){
    if (lednum==i){
      light_led(i, 0, 0x0ff0);
    }
    else if ((time%64)/4 == i){
      light_led(i, 0, 0x0200);
    }
    else if (((time-4)%64)/4 == i){
      light_led(i, 0, 0x0100);
    }
    else if (((time-8)%64)/4 == i){
      light_led(i, 0, 0);
    }
    else if (((time-12)%64)/4 == i){
      light_led(i, 0, 0);
    }
    else {
      light_led(i, 0, 0);
    }
  }
}
void light_led(int num, int which, uint16_t count){ // count=0-4095
  int err;
  uint8_t adrs = num * 4 + 0x06;
  pca9544_changeI2cBus(3,which);
	err = PCA9685_write( 0, adrs, 0 );          // ONはtime=0
	err = PCA9685_write( 0, adrs+1, 0 );        // ONはtime=0
	err = PCA9685_write( 0, adrs+2, (uint8_t)(count & 0x00ff) );// OFF 0-4095 (0-0x0fff) の下位8bit
	err = PCA9685_write( 0, adrs+3, (uint8_t)(count>>8) );      // OFF 上位4bit
}

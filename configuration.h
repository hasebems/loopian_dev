/* ========================================
 *
 *	configuration.h
 *		description: TouchMidi Configuration
 *
 *	Copyright(c)2017- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt.
 *
 * ========================================
*/
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//---------------------------------------------------------
//    Touch Sensor Setup Mode
//---------------------------------------------------------
#define   SETUP_MODE    0   //  1: Setup Mode, 0: Normal Mode

#define   MAX_DEVICE_MBR3110    6
#define   MAX_TOUCH_EV          8
#define   MAX_EACH_SENS         8
//---------------------------------------------------------
//		I2C Device Configuration
//---------------------------------------------------------
#define		USE_CY8CMBR3110  // とりあえずまだ off
#define		USE_ADA88
//#define		USE_LPS22HB
//#define		USE_LPS25H
//#define		USE_AQM1602XA
//#define		USE_ADXL345
#define		USE_PCA9685
//#define		USE_ATTINY
#define		USE_PCA9544A
#endif

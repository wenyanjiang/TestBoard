#include "delay.h"
#include "stdio.h"
#include "bsp.h"
#include "w25qxx.h"

//define simplified lux calculation
#define LUX_SCALE 14 // scale by 2^14
#define RATIO_SCALE 9 // scale ratio by 2^9
//---------------------------------------------------
// Integration time scaling factors
//---------------------------------------------------
#define CH_SCALE 10 // scale channel values by 2^10
#define CHSCALE_TINT0 0x7517 // 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1 0x0fe7 // 322/81 * 2^CH_SCALE

#define K1T 0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T 0x01be // 0.0272 * 2^LUX_SCALE
#define K2T 0x0080 // 0.250 * 2^RATIO_SCALE
#define B2T 0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1 // 0.0440 * 2^LUX_SCALE
#define K3T 0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T 0x023f // 0.0351 * 2^LUX_SCALE
#define M3T 0x037b // 0.0544 * 2^LUX_SCALE
#define K4T 0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe // 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T 0x016f // 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc // 0.0310 * 2^LUX_SCALE
#define K6T 0x019a // 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb // 0.0153 * 2^LUX_SCALE
#define K7T 0x029a // 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 // 0.00112 * 2^LUX_SCALE
#define K8T 0x029a // 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 // 0.000 * 2^LUX_SCALE
#define M8T 0x0000 // 0.000 * 2^LUX_SCALE

//---------------------------------------------------
// CS package coefficients
//---------------------------------------------------
#define K1C 0x0043 // 0.130 * 2^RATIO_SCALE
#define B1C 0x0204 // 0.0315 * 2^LUX_SCALE
#define M1C 0x01ad // 0.0262 * 2^LUX_SCALE
#define K2C 0x0085 // 0.260 * 2^RATIO_SCALE
#define B2C 0x0228 // 0.0337 * 2^LUX_SCALE
#define M2C 0x02c1 // 0.0430 * 2^LUX_SCALE
#define K3C 0x00c8 // 0.390 * 2^RATIO_SCALE
#define B3C 0x0253 // 0.0363 * 2^LUX_SCALE
#define M3C 0x0363 // 0.0529 * 2^LUX_SCALE
#define K4C 0x010a // 0.520 * 2^RATIO_SCALE
#define B4C 0x0282 // 0.0392 * 2^LUX_SCALE
#define M4C 0x03df // 0.0605 * 2^LUX_SCALE
#define K5C 0x014d // 0.65 * 2^RATIO_SCALE
#define B5C 0x0177 // 0.0229 * 2^LUX_SCALE
#define M5C 0x01dd // 0.0291 * 2^LUX_SCALE
#define K6C 0x019a // 0.80 * 2^RATIO_SCALE
#define B6C 0x0101 // 0.0157 * 2^LUX_SCALE
#define M6C 0x0127 // 0.0180 * 2^LUX_SCALE
#define K7C 0x029a // 1.3 * 2^RATIO_SCALE
#define B7C 0x0037 // 0.00338 * 2^LUX_SCALE
#define M7C 0x002b // 0.00260 * 2^LUX_SCALE
#define K8C 0x029a // 1.3 * 2^RATIO_SCALE
#define B8C 0x0000 // 0.000 * 2^LUX_SCALE
#define M8C 0x0000 // 0.000 * 2^LUX_SCALE


//*********************“‘…œ «calculate ≈‰÷√********************************

//#define tsl2561_add0 0x29 // '0'
//#define tsl2561_add  0x39 // 'float'0x39
//#define tsl2561_add1 0x49 // '1'


//#define tsl2561_add0 0x52 // '0' 0101001_0
//#define tsl2561_add  	0x72 // 'float' 0111001_0 0x72
//#define tsl2561_add1 0x92 // '1' 1001001_0

////tsl2561 reg
//#define tsl2561_cmd 		0x80
//#define tsl2561_timing 		0x81
//#define tsl2561_reg_id		0x8a
//#define tsl2561_data0_low 	0x8c
//#define tsl2561_data0_high 	0x8d
//#define tsl2561_data1_low 	0x8e
//#define tsl2561_data1_high 	0x8f	

////power_on power off
//#define tsl2561_power_on 0x03
//#define tsl2561_power_off 0x00

////timing reg value
//#define timing_13ms 	0x00 //13.7ms
//#define timing_101ms	0x01 //101ms
//#define timing_402ms 	0x02 //402ms
//#define timing_gain_1x  0x10 //gain_pux_time
//#define timing_gain_16x 0x00 //

//tsl_2561_start_integration
void tsl_start(void);
	
void tsl_stop(void);

//write code and data
//high 4bits control command and low 4bits is reg addr 
void tsl2561_write(uint8_t command,uint8_t data);

//read one byte data
//command high 4bits is cmd,low 4bits is data addr
uint16_t tsl2561_read(uint8_t command);

//function:start tsl module
void tsl2561_poweron(void);

//close tsl
void tsl2561_poweroff(void);

//time
void tsl2561_timeset(uint8_t time);

//init tsl2561
void tsl2561_init(uint8_t time_mode1);

//read 16bits chanel0
 uint16_t tsl_chl0_read(void);

//read 16bits chanel1
 uint16_t tsl_chl1_read(void);

 //caculate_float
 double tsl_adj(void);
 
 //calculate_no_float
 unsigned int CalculateLux(unsigned int iGain, unsigned int tInt, unsigned int ch0,unsigned int ch1, uint8_t iType); 



/*****************END*********************/


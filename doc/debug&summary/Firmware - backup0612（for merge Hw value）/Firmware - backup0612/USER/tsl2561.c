#include "stm32f4xx.h"
//#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "stdio.h"
#include "math.h"
#include "bsp.h"
#include "tsl2561.h"

#define tsl2561_add0 0x52 // '0' 0101001_0
#define tsl2561_add  0x72//0x72 // 'float' 0111001_0
#define tsl2561_add1 0x92 // '1' 1001001_0

//tsl2561 reg
#define tsl2561_cmd 	    0x80
#define tsl2561_timing 		0x81
#define tsl2561_reg_id		0x8a
#define tsl2561_data0_low 	0x8c
#define tsl2561_data0_high 	0x8d
#define tsl2561_data1_low 	0x8e
#define tsl2561_data1_high 	0x8f

//power_on power off
#define tsl2561_power_on  0x03
#define tsl2561_power_off 0x00

//timing reg value
#define timing_13ms 	0x00 //13.7ms
#define timing_101ms	0x01 //101ms
#define timing_402ms 	0x02 //402ms
#define timing_gain_1x  0x10 //gain_pux_time
#define timing_gain_16x 0x00
#define tsl_start_inte 	0x0b	//manual start
#define tsl_stop_inte 	0x03  //manual stop

//sampling_value
uint16_t ch0=0;
uint16_t ch1=0;

//write code and data
//high 4bits control command and low 4bits is reg addr 
void tsl2561_write(uint8_t command,uint8_t data)
{
	IIC_WriteByte(Open_I2Cx,tsl2561_add,command,data,1);
//	iic_start();
//	iic_byte_write(tsl2561_add<<1);
//	iic_wait_ack();
//	iic_byte_write(command);
//	iic_wait_ack();
//	iic_byte_write(data);
//	iic_wait_ack();
//	iic_stop();

}

//tsl_2561_start_integration
void tsl_start()
{
	tsl2561_write(0x81,0x12);
}

//tsl_2561_start_integration
void tsl_stop()
{
	tsl2561_write(0x81,0x03);
}


//read one byte data
//command high 4bits is cmd,low 4bits is data addr
uint16_t tsl2561_read_two_byte(uint8_t command,uint8_t addr)
{
        uint16_t data;
	 uint16_t tmr;
	 uint8_t I2C_Err=0;
	
    	tmr = 1000;
    while((--tmr)&&I2C_GetFlagStatus(Open_I2Cx, I2C_FLAG_BUSY));
    if(tmr==0) {I2C_Err = 1;}

	 I2C_GenerateSTART(Open_I2Cx, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(Open_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
    if(tmr==0) {I2C_Err = 1;}

    I2C_Send7bitAddress(Open_I2Cx, addr, I2C_Direction_Transmitter);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(Open_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
    if(tmr==0) {I2C_Err = 1;}
	
	 I2C_SendData(Open_I2Cx, command);
	 tmr = 1000;
	 while((--tmr)&&(!I2C_CheckEvent(Open_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	 if(tmr==0) {I2C_Err = 1;}
	
    I2C_GenerateSTART(Open_I2Cx, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(Open_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
    if(tmr==0) {I2C_Err = 1;}

    I2C_Send7bitAddress(Open_I2Cx, addr, I2C_Direction_Receiver);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(Open_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
    if(tmr==0) {I2C_Err = 1;} 

	 tmr = 1000;
    while((--tmr)&&(!(I2C_CheckEvent(Open_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)))); 
    if(tmr==0) {I2C_Err = 1;}
	 data= I2C_ReceiveData(Open_I2Cx);
	 
    I2C_AcknowledgeConfig(Open_I2Cx, DISABLE);
    I2C_GenerateSTOP(Open_I2Cx, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!(I2C_CheckEvent(Open_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)))); 
    if(tmr==0) {I2C_Err = 1;}

    data = data | (I2C_ReceiveData(Open_I2Cx)<<8);

    I2C_AcknowledgeConfig(Open_I2Cx, ENABLE);
	return data;
//	iic_start();
//	iic_byte_write(tsl2561_add<<1);
//	iic_wait_ack();
//	iic_byte_write(command);
//	iic_wait_ack();

//	iic_start();
//	iic_byte_write((tsl2561_add<<1)+1);
//	iic_wait_ack();
//	data=iic_byte_read(1);
//	data+=((iic_byte_read(1))<<8);
//	iic_stop();
}

//function:start tsl module
void tsl2561_poweron(void)
{
	tsl2561_write(tsl2561_cmd,tsl2561_power_on);	
}

//close tsl
void tsl2561_poweroff(void)
{
	tsl2561_write(tsl2561_cmd,tsl2561_power_off);
}

//time
void tsl2561_timeset(uint8_t time)
{
		tsl2561_write(tsl2561_timing,time);
//	iic_start();
//	iic_byte_write(tsl2561_add<<1);
//	iic_wait_ack();
//	iic_byte_write(tsl2561_timing);
//	iic_wait_ack();
//	iic_byte_write(time);
//	iic_wait_ack();
//	iic_stop();
}

//init tsl2561
void tsl2561_init(uint8_t time_mode1)
{
	tsl2561_poweron();
	tsl_stop();
}

//read 16bits chanel0
 uint16_t tsl_chl0_read(void)
 {
 	uint16_t buff;
 	buff=tsl2561_read_two_byte(tsl2561_data0_low,tsl2561_add);
 	return buff;
 }

 //read 16bits chanel1
 uint16_t tsl_chl1_read(void)
 {
 	uint16_t buff;
 	buff=tsl2561_read_two_byte(tsl2561_data1_low,tsl2561_add);
 	return buff;
 }

 //adj
 double tsl_adj()
 {
 	double lux;
 	if(((double)(ch1/ch0)<=0.50)&&((double)(ch1/ch0)>0))
 	{
 		lux=0.0304*ch0-0x062*ch0*pow((ch1/ch0),1.4);
 	}
 	else if(((double)(ch1/ch0)<=0.61)&&((double)(ch1/ch0)>0.50))
 	{
 		lux=0.0224*ch0-0.031*ch1;
 	}
 	else if(((double)(ch1/ch0)<=0.8)&&((double)(ch1/ch0)>0.61))
 	{
 		lux=0.0128*ch0-0.0153*ch1;
 	}
 	else if(((double)(ch1/ch0)<=1.30)&&((double)(ch1/ch0)>0.80))
 	{
 		lux=0.00146*ch0-0.00112*ch1;
 	}
 	else if((double)(ch1/ch0)>1.30)
 	{
 		lux = 0;
 	}
 	return lux;
 }
 //calculate
 
unsigned int CalculateLux(unsigned int iGain, unsigned int tInt, unsigned int ch0,unsigned int ch1, uint8_t iType)
{
//------------------------------------------------------------------------
// first, scale the channel values depending on the gain and integration time
// 16X, 402mS is nominal.
// scale if integration time is NOT 402 msec
	unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;
	unsigned long ratio1 = 0;
	unsigned long ratio=0;
	unsigned long temp;	
	unsigned int b, m;
	unsigned long lux=0;	
	switch (tInt)
	{
		case 0: // 13.7 msec
		chScale = CHSCALE_TINT0;
		break;
		case 1: // 101 msec
		chScale = CHSCALE_TINT1;
		break;
		default: // assume no scaling
		chScale = (1 << CH_SCALE); 
		break;
	}
// scale if gain is NOT 16X
		if (!iGain) chScale = chScale << 4; // scale 1X to 16X
		// scale the channel values
		channel0 = (ch0 * chScale) >> CH_SCALE;
		channel1 = (ch1 * chScale) >> CH_SCALE;
//------------------------------------------------------------------------
// find the ratio of the channel values (Channel1/Channel0)
// protect against divide by zero

		if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;
		// round the ratio value
		ratio = (ratio1 + 1) >> 1;
// is ratio <= eachBreak ?

		switch (iType)
		{
		case 0: // T, FN and CL package
			/*if ((ratio >= 0) && */if(ratio <= K1T)
			{b=B1T; m=M1T;}
			else if (ratio <= K2T)
			{b=B2T; m=M2T;}
			else if (ratio <= K3T)
			{b=B3T; m=M3T;}
			else if (ratio <= K4T)
			{b=B4T; m=M4T;}
			else if (ratio <= K5T)
			{b=B5T; m=M5T;}
			else if (ratio <= K6T)
			{b=B6T; m=M6T;}
			else if (ratio <= K7T)
			{b=B7T; m=M7T;}
			else if (ratio > K8T)
			{b=B8T; m=M8T;}
		break;
		case 1:// CS package
		/*	if ((ratio >= 0) && */if(ratio <= K1C)
			{b=B1C; m=M1C;}
			else if (ratio <= K2C)
			{b=B2C; m=M2C;}
			else if (ratio <= K3C)
			{b=B3C; m=M3C;}
			else if (ratio <= K4C)
			{b=B4C; m=M4C;}
			else if (ratio <= K5C)
			{b=B5C; m=M5C;}
			else if (ratio <= K6C)
			{b=B6C; m=M6C;}
			else if (ratio <= K7C)
			{b=B7C; m=M7C;}
			else if (ratio > K8C)
			{b=B8C; m=M8C;}
		break;
		}

			temp = ((channel0 * b) - (channel1 * m));
			// do not allow negative lux value
			//if (temp < 0) temp = 0;
			// round lsb (2^(LUX_SCALE-1))
			temp += (1 << (LUX_SCALE-1));
			// strip off fractional portion
			lux = temp >> LUX_SCALE;
			return lux;
}

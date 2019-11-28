#ifndef __BSP_H
#define __BSP_H

#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "protocol.h"


#define R_SHUNT					250 /* unit: mO */ 
#define I_LIMIT					300000 /* unit: uA */
#define MAX_VAL_INA230			(1<<15) 


#define PLATFORM_4940				1
#define PLATFORM_6500				2
#define PLATFORM_TYPE				PLATFORM_4940

#define RT9367_ADDR 				0xa8	//101_0100_0
#define PMOD2_DET_ADDR 				0x80	//1000000_0
#define PMOD1_DET_ADDR 				0x8a	//1000101_0

#define RT9367_LDO1_ON 				0x21
#define RT9367_LDO1_OFF 			0x20
#define RT9367_LDO2_ON				0x25
#define RT9367_LDO2_OFF				0x24



/*It is the 1.8V enable pin in board 6500*/
#define RT9367_EN()					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET)
#define RT9367_DISEN()				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET)

#define VMOD1_EN()					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)
#define VMOD1_DISEN()				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)

#if(PLATFORM_TYPE == PLATFORM_6500)
	#define CALI_PIN_EN()			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)
	#define CALI_PIN_DIS()			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)			
#endif

typedef enum
{
  	CONFIG_ADDR = 0,
 	VSHUNT_ADDR,
  	VOLTAGE_ADDR,
  	POWER_ADDR,
    CURRENT_ADDR,
  	CAL_ADDR,
  	MASK_ENABLE_ADDR,
 	ALERT_ADDR,
}INA230_REG_ADDR;




extern 	DAC_HandleTypeDef hdac;
                                                                                                                                                                   extern 	I2C_HandleTypeDef hi2c1;
extern  SPI_HandleTypeDef hspi1;
extern  SPI_HandleTypeDef hspi3;
//extern  TIM_HandleTypeDef htim4;

extern  volatile uint8_t g_u8Buf;
extern  volatile uint8_t g_u8Vmod2;
extern  volatile uint16_t g_u16Vmod1;
extern  volatile uint8_t g_u8SpiSped;
extern  volatile uint16_t g_u16Mod1CaliVal;
extern  volatile uint16_t g_u16Mod2CaliVal;
extern  volatile int g_readCurrent;

void IwdgFeed(void);
void BSP_VoltageInit(uint16_t mode1,uint8_t buf,uint8_t mode2);	
void BSP_VoltAndSpiSetting(uint16_t mode1,uint8_t mode2,uint8_t buf,uint8_t spiSpeed);	
void BSP_DacVolSetting(uint16_t vol);
void BSP_INA230Init(uint16_t mod1CaliVal,uint16_t mod2Calival);
int BSP_ReadCurrent(uint8_t chanel);
uint32_t BSP_ReadVoltage(uint8_t chanel);
EN_RESP_TYPE BSP_SetBacklight(uint8_t curLed,uint8_t PWMled1,uint8_t PWMled2);
short BSP_VMOD1Calibration(uint32_t *databuf);
void BSP_VMOD2Calibration(uint32_t *databuf);
void BSP_MOD1MOD2Calibration(void);
EN_RESP_TYPE BSP_PowerControl(uint8_t control,uint16_t delay);
EN_RESP_TYPE BSP_SpiSpeedSetting(uint8_t prescal);
void BSP_Sector8Prasing(void);

int Dev_SPI_SendData(SPI_HandleTypeDef *hspi , uint8_t * SendBuf , uint32_t size , uint32_t timeout);
int Dev_SPI_ReceiveData(SPI_HandleTypeDef *hspi , uint8_t * receBuf , uint32_t size , uint32_t timeout);
int Dev_SPI_ReceiveDataDMA(SPI_HandleTypeDef *hspi , uint8_t * receBuf , uint32_t size);
int Dev_SPI_SendDataContinus(SPI_HandleTypeDef *hspi , uint8_t * sendBuf , uint32_t size);
void Dev_Flash_SPISendData(uint8_t * SendBuf , uint32_t size , uint32_t timeout);
void Dev_Flash_SPIReceiveData(uint8_t * receBuf , uint32_t size , uint32_t timeout);
void Dev_Flash_SPIReceiveDataDMA(uint8_t * receBuf , uint32_t size);

#endif

#ifndef __BSP_H
#define __BSP_H 


#include "stm32f4xx.h"
//#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "stdio.h"

#define Burn_off GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define Burn_on	 GPIO_ResetBits(GPIOB,GPIO_Pin_6)
//#define DEBUG
//#define Image_Size		64*800
#define PackSize 			64

#define APP_DEFAULT_ADD                 0x08040000

//FLASH起始地址
#define STM32_FLASH_BASE 		0x08000000 	//STM32 FLASH的起始地址

#define FLASH_SAVE_ADDR  		0x0800C000 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
														//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
#define GAIN_INFOR_SAVE_ADDR  0x0800C004		//4Byte length + 4Byte CRC16 + 2Byte Gain infor use 10 Byte,/*2Byte reserve*/.2Byte used
										//0x0800C008
										//0x0800C00C

/*******CANCEL USER_INFOR_SAVE_ADDR***********/
#define USER_INFOR_SAVE_ADDR  FLASH_SAVE_ADDR+16	//64Byte user name + 64Byte password use 128 Byte
/*******CANCEL USER_INFOR_SAVE_ADDR***********/


#define VMOD1_ADDR  									0x0800C010
#define VMOD2_ADDR  									0x0800C014
#define BUFFER_ADDR  								0x0800C018

#define VMOD1_RX_CALIBRATION_ADDR  				0x0800C01C
#define VMOD1_CALIBRATION_BACKUP_ADDR1  		0x0800C020
#define VMOD1_CALIBRATION_BACKUP_ADDR2  		0x0800C024
#define VMOD2_RX_CALIBRATION_ADDR  				0x0800C028
#define VMOD2_CALIBRATION_BACKUP_ADDR1  		0x0800C02C
#define VMOD2_CALIBRATION_BACKUP_ADDR2  		0x0800C030


//12 Byte reserve
#define PARAMETER_SAVE_ADDR   FLASH_SAVE_ADDR+156	//4Byte image_Size + 12Byte voltage + 200Byte RegParameter use 216 Byte
//以上从0地址开始总共用了372字节的数据
#define GAIN_PARAMETER_SAVE_ADDR   FLASH_SAVE_ADDR+372//122PMU+4Byte use 126Byte,2Byte reserve.
//以上从0地址开始总共用了500字节的数据

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  

#define FIRE 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7) 	//PE7
#define KEY2 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11) 	//PE11
#define KEY3 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10)	//PE10 
#define KEY2_PRESS	1
#define KEY3_PRESS	2
#define BOTH_PRESS	3

#define LED1_Toggle		GPIO_ToggleBits(GPIOE,GPIO_Pin_12)//LED1
#define LED2_Toggle		GPIO_ToggleBits(GPIOE,GPIO_Pin_13)//LED2
#define LED3_Toggle		GPIO_ToggleBits(GPIOE,GPIO_Pin_14)//LED3
#define LED4_Toggle		GPIO_ToggleBits(GPIOE,GPIO_Pin_15)//LED4

#define LED1_OFF 		GPIO_ResetBits(GPIOE,GPIO_Pin_12)
#define LED1_ON 		GPIO_SetBits(GPIOE,GPIO_Pin_12)
#define LED2_OFF 		GPIO_ResetBits(GPIOE,GPIO_Pin_13)
#define LED2_ON 		GPIO_SetBits(GPIOE,GPIO_Pin_13)
#define LED3_OFF 		GPIO_ResetBits(GPIOE,GPIO_Pin_14)
#define LED3_ON 		GPIO_SetBits(GPIOE,GPIO_Pin_14)
#define LED4_OFF 		GPIO_ResetBits(GPIOE,GPIO_Pin_15)
#define LED4_ON 		GPIO_SetBits(GPIOE,GPIO_Pin_15)

#define CS0_High			GPIO_SetBits(GPIOA,GPIO_Pin_0)
#define CS0_Low			GPIO_ResetBits(GPIOA,GPIO_Pin_0)
#define CS1_High			GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define CS1_Low			GPIO_ResetBits(GPIOC,GPIO_Pin_13)

#define G0361_BLED_ON			GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define G0361_BLED_OFF			GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define G0361_RLED_ON			GPIO_SetBits(GPIOE,GPIO_Pin_3)
#define G0361_RLED_OFF			GPIO_ResetBits(GPIOE,GPIO_Pin_3)

#define G0361_POWERON			GPIO_SetBits(GPIOE,GPIO_Pin_5)
#define G0361_POWEROFF			GPIO_ResetBits(GPIOE,GPIO_Pin_5)

#define G0360_POWERON			GPIO_ResetBits(GPIOE,GPIO_Pin_6)
#define G0360_POWEROFF			GPIO_SetBits(GPIOE,GPIO_Pin_6)

#define CS_FLASH_High	GPIO_SetBits(GPIOE,GPIO_Pin_4);\
								CS0_Low;\
								delay_us(7)
								
#define CS_FLASH_Low		GPIO_ResetBits(GPIOE,GPIO_Pin_4);\
								CS0_High;\
								delay_us(7)
								
#define CS_FLASH_High_MK510	GPIO_SetBits(GPIOC,GPIO_Pin_4);\
										CS0_Low;\
										delay_us(7)
								
#define CS_FLASH_Low_MK510		GPIO_ResetBits(GPIOC,GPIO_Pin_4);\
										CS0_High;\
										delay_us(7)								

#define RT9367_addr 			0xa8	//101_0100_0
#define PMOD2_DET_addr 		0x80	//1000000_0
#define PMOD1_DET_addr 		0x8a	//1000101_0

#define PWRON_1117 			GPIO_SetBits(GPIOB,GPIO_Pin_14);\
									G0361_POWERON;\
									G0360_POWERON;\
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,VMOD2,1)//MOD2 on
									
#define PWRON_60ms_delay 				PWRON_1117;\
																IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,VMOD2,1);\
																delay_ms(60)//pwron_60ms

#define PWROFF_1117 			GPIO_ResetBits(GPIOB,GPIO_Pin_14);\
									G0361_POWEROFF;\
									G0360_POWEROFF;\
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VMOD2,1)//MOD2 off

#define Open_I2Cx                        	I2C1
//#define Open_I2Cx                        	I2C2
#define Open_I2Cx_CLK                    	RCC_APB1Periph_I2C2

#define Open_I2Cx_SDA_PIN                 	GPIO_Pin_0
#define Open_I2Cx_SDA_GPIO_PORT           	GPIOF
#define Open_I2Cx_SDA_GPIO_CLK            	RCC_AHB1Periph_GPIOF
#define Open_I2Cx_SDA_SOURCE              	GPIO_PinSource0
#define Open_I2Cx_SDA_AF                  	GPIO_AF_I2C2

#define Open_I2Cx_SCL_PIN                 	GPIO_Pin_1
#define Open_I2Cx_SCL_GPIO_PORT           	GPIOF
#define Open_I2Cx_SCL_GPIO_CLK            	RCC_AHB1Periph_GPIOF
#define Open_I2Cx_SCL_SOURCE              	GPIO_PinSource1
#define Open_I2Cx_SCL_AF                  	GPIO_AF_I2C2

#define I2Cx_SPEED               100000
#define I2Cx_SLAVE_ADDRESS7      0x30



#define Open_RCC_APB2Periph_SPIx   	       RCC_APB2Periph_SPI1

#define Open_SPIx                           SPI1
#define Open_SPIx_CLK                       RCC_APB2Periph_SPI1
#define Open_SPIx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define Open_SPIx_IRQn                      SPI1_IRQn
#define Open_SPIx_IRQHANDLER                SPI1_IRQHandler

#define Open_SPIx_SCK_PIN                   GPIO_Pin_3
#define Open_SPIx_SCK_GPIO_PORT             GPIOB
#define Open_SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define Open_SPIx_SCK_SOURCE                GPIO_PinSource3
#define Open_SPIx_SCK_AF                    GPIO_AF_SPI1

#define Open_SPIx_MISO_PIN                  GPIO_Pin_6
#define Open_SPIx_MISO_GPIO_PORT            GPIOA
#define Open_SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define Open_SPIx_MISO_SOURCE               GPIO_PinSource6
#define Open_SPIx_MISO_AF                   GPIO_AF_SPI1

#define Open_SPIx_MOSI_PIN                  GPIO_Pin_7
#define Open_SPIx_MOSI_GPIO_PORT            GPIOA
#define Open_SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define Open_SPIx_MOSI_SOURCE               GPIO_PinSource7
#define Open_SPIx_MOSI_AF                   GPIO_AF_SPI1


#define RCC_GPIO_CS								RCC_AHB1Periph_GPIOA
#define RCC_AHBxPeriphClockCmd				RCC_AHB1PeriphClockCmd
#define GPIO_PIN_CS0								GPIO_Pin_0
#define GPIO_PIN_CS1								GPIO_Pin_13
#define RST											GPIO_Pin_1
#define GPIO_CS_PORT								GPIOA


#define Open_USART                        USART1
#define Open_USART_CLK                    RCC_APB2Periph_USART1

#define Open_USART_TX_PIN                 GPIO_Pin_9
#define Open_USART_TX_GPIO_PORT           GPIOA
#define Open_USART_TX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define Open_USART_TX_SOURCE              GPIO_PinSource9
#define Open_USART_TX_AF                  GPIO_AF_USART1


#define Open_USART_RX_PIN                 GPIO_Pin_10
#define Open_USART_RX_GPIO_PORT           GPIOA
#define Open_USART_RX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define Open_USART_RX_SOURCE              GPIO_PinSource10
#define Open_USART_RX_AF                  GPIO_AF_USART1

#define Open_USART_IRQn                   USART1_IRQn	 


typedef union _CMD
{
	uint8_t Byte[2];
	uint16_t TwoByte;
}TWO_BYTE_CMD;

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

void HANDSHAKE(void);

void TIM4_PWM_Config(uint16_t arr,uint16_t psc,uint16_t PWM_Val);

void delay(uint16_t i);

void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);
void TXDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u16 ndtr);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr,uint32_t rx_buf);
uint8_t SPI1_ReadWriteByte(uint8_t TxData);
void Write_PMU(uint8_t addr, uint8_t data);
void Read_data(uint8_t *data);
uint16_t Read_PMU(uint8_t cmd);
uint8_t Read_Byte(void);

u32 STMFLASH_ReadWord(u32 faddr);
uint8_t STMFLASH_ReadByte(u32 faddr);
void STMFlash_EraseSect(u32 WriteAddr, u32 NumToWrite);
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);
void MY_STMFLASH_Write(u32 WriteAddr,u8 *pBuffer,u32 NumToWrite);
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);
void STMFLASH_ReadforByte(u32 ReadAddr,uint8_t *pBuffer,u32 NumToRead);
uint16_t STMFLASH_GetFlashSector(u32 addr);
void Start_Process(void);
void LED_Config(void);
void GPIO_Config(void);
void MX_GPIO_TEST_Init(void);
void DAC1_init(void);
void Dac1_Dec_Set(uint16_t Dec);
void Dac1_Set_Vol(uint16_t vol);
uint8_t VMOD1_Voltage_Set(uint16_t Voltage,uint16_t *DAC_Dec);
void SPI_Configuration(void);
void SPI3_Configuration(void);
void SPI1_ReConfig(void);
void SPI1_SetDirection(uint32_t SPI_Direction);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
void I2C_Config(void);
void I2C1_Config(void);
void USART_Configuration(void);
void Command(uint8_t *parameter);
void SendOK(void);
void SendError(void);
void SendErrorCode(uint8_t Code,uint16_t Result);
void Send_FWID(void);
uint8_t WriteRegToCMOS(void);
uint8_t WriteRegToPMU(void);
//void WriteSizeRegToPMU(void);
void SendCMOSImageData(void);
void SendImageData(void);
void SendSingleFrameImage(uint8_t cmd);
uint8_t SendContinueImage(uint8_t cmd1,uint8_t cmd2,uint16_t value);
void ProcessNewCmd(TWO_BYTE_CMD *CMD);
//void ProcessOldCmd(TWO_BYTE_CMD *CMD);

void RST_PMU(void);
void BUF_Config(void);
void KEY_Config(void);

void BreathingLightProcess(void);
void KeyProcess(void);

uint8_t WakeUpProcess(void);
uint8_t SleepProcess(void);
void TimeOutProcess(void);
uint8_t CodeDataCopy(uint8_t *data,uint32_t length);
uint16_t CRC16(uint8_t *Buf, uint32_t BufLen, uint16_t CRCCode);
void swap_u16_data(uint16_t *ptr,uint32_t size);
uint8_t KEY_Scan(uint8_t mode);
uint8_t Touch_Scan(uint8_t mode);
void UserNamePassWordSave(void);
void SendUserNamePassWord(void);

void RT9367C_Init(void);
void Voltage_Init(uint8_t *VolSetBuf);
//void Voltage_Init(uint32_t VT1,uint32_t VT2,uint32_t BUF);
void Both_INA230_Init(uint8_t *INA230Buf);
int16_t Voltage_MOD1_DET(void);
int16_t Voltage_MOD2_DET(void);
int16_t Current_MOD1_DET(void);
int16_t Current_MOD2_DET(void);
void VMOD1_Calibration(uint8_t *R_CalibrationRegister);
void VMOD2_Calibration(uint8_t *R_CalibrationRegister);
void IIC_WriteByte(I2C_TypeDef *I2Cx,u8 addr,u8 reg,u32 data,u8 num);
void IIC_WriteByte1(I2C_TypeDef *I2Cx,u8 addr,u8 reg,u32 data,u8 num);
int16_t I2C_ReadTwoByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t Reg_addr);
void IWDG_Init(u8 prer,u16 rlr);
uint8_t Handshake(void);
void TIM3_Config(uint16_t arr, uint16_t psc);


uint8_t ReceHead(void);
uint8_t ReceEnd(void);
uint8_t ReadyForCMD(void);
uint8_t ReadyFor_module_sn_cmd(void);
uint8_t ReadyForMK_VOL(void);
void SPIReadData(uint8_t *data);

#endif




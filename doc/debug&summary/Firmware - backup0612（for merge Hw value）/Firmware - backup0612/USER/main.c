/**
  ******************************************************************************
  * @file    app.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/ 
//#include "usbd_cdc_core.h"
//#include "usbd_usr.h"
//#include "usbd_desc.h"
#include "delay.h"
#include "stdio.h"
#include "bsp.h"
#include "w25qxx.h"
#include "tsl2561.h"



/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup APP_HID 
  * @brief Mass storage application module
  * @{
  */ 

/** @defgroup APP_HID_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup APP_HID_Private_Defines
  * @{
  */ 


/**
  * @} 
  */ 


/** @defgroup APP_HID_Private_Macros
  * @{
  */ 
  
uint8_t DaT[8]; 
	
extern uint32_t Image_Size;
extern uint8_t Sdata[];

extern uint8_t Line_H,Line_L;
//uint8_t CodeBuffer[65536];
//extern uint8_t SendCmdBuff[64];
extern uint8_t RegParameter[200];
extern const uint8_t FWID[4]; 
//extern uint8_t DFUID[4];
//uint16_t ii=1000;
//uint8_t ContinueFlag =0;
uint8_t DFUDataFlag=0,DataFlag=0,GainDataFlag=0,FlashDataFlag=0;
volatile uint8_t TouchFire = 0,TouchCMOS = 0,TouchOPPO_all=0,TouchOPPO_win=0,TouchMK210_win=0,Touchmk310_win=0,Touchmk510_win=0;
volatile uint8_t Touchmk610_win=0,Touchmk710_win=0,Touchmk710_win_B=0;
uint8_t TouchMod = 0;

uint8_t touch_4=0;//touch 采图4张，上传1张
uint8_t xxh,xxl,yyh,yyl,low_power_flag=0;

__IO uint16_t VMOD1=3300,VMOD2=0x1f,BUF=0x0E;

extern uint8_t WakeUpReg;
extern uint8_t Reg9d1,Reg9d2;
extern uint8_t Reg9e1,Reg9e2;

extern  __IO uint8_t USB_StatusDataSended;
extern  uint32_t USB_ReceivedCount;
//extern  uint8_t USB_Tx_Buffer[];
extern  uint8_t USB_Rx_Buffer[];
extern __IO uint8_t DeviceConfigured;
extern __ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

/**
  * @}
  */ 


/** @defgroup APP_HID_Private_Variables
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup APP_HID_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */ 


/** @defgroup APP_HID_Private_Functions
  * @{
  */ 

/**
  * @brief  Program entry point
  * @param  None
  * @retval None
  */

void TaskIwdg(void);
void TaskLED(void);
void TaskKeyScan(void);
void TaskUSBReceiveDataProcess(void);
void TaskSendContinueImage(void);

void TaskProcess(void);
void TaskRemarks(void);

void PVD_Config(uint8_t pvd);

typedef struct _TASK_COMPONENTS
{
	uint8_t Run;
	uint16_t Timer;
	uint16_t ItvTimer;
	void (*TaskHook)(void);
}TASK_COMPONENTS;

typedef enum _TASK_LIST
{
	TASK_SEND_CONTINUE_IMAGE,
	TASK_USB_DATA,
	TASK_KEY_SCAN,
	TASK_LED,
	TASK_IWDG,
	TASKS_MAX
}TASK_LIST;


static TASK_COMPONENTS TaskComps[]=
{
	{0,1,1,TaskSendContinueImage},
	{0,10,10,TaskUSBReceiveDataProcess},
	{0,100,100,TaskKeyScan},
	{0,1000,1000,TaskLED},
	{0,3000,3000,TaskIwdg}
};

void set_bor(uint8_t level)
{
	      FLASH_OB_Unlock();
       /* Select the desired V(BOR) Level */
			FLASH_OB_BORConfig(level); 
      /* Launch the option byte loading */
			FLASH_OB_Launch(); 
       /* Locks the option bytes block access */
			FLASH_OB_Lock();
}
int main(void)
{
//	uint16_t i,j=0;
	uint16_t i;
//	uint16_t Timer = 500;
//	uint8_t dd[4] = {0x12,0x34,0x56,0x78};
//	uint32_t *dtt = (uint32_t*)dd;
  /*!< At this stage the microcontroller clock setting is already configured, 
  this is done through SystemInit() function which is called from startup
  file (startup_stm32fxxx_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32fxxx.c file
  */
	delay_init(168);
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)Sdata,1024);// Read 4K
	USART_Configuration();	
   GPIO_Config();
	LED_Config();
			
	TIM3_Config(1000-1,84-1);//1ms定时器
//	TIM3_Config(1000-1,60-1);//1ms定时器
//	TIM3_Config(1000-1,48-1);//1ms定时器
	TIM4_PWM_Config(400-1,21-1,0);//led_pwm
	
	BUF_Config();
	KEY_Config();
	Start_Process();
//	for(i=0;i<3;i++)
//	{
//		DFUID[i] = STMFLASH_ReadByte(FLASH_SAVE_ADDR+i+1);//ADDR-1-2-3 DFU Number
//	}
	//printf("\r\nDFU Version is: %d.%d.%d\r\n",DFUID[0],DFUID[1],DFUID[2]);
	printf("\r\nDFU Version is: %d.%d.%d\r\n",Sdata[1],Sdata[2],Sdata[3]);
	printf("Firmware Version is: %d.%d.%d\r\n",FWID[0],FWID[1],FWID[2]);
	printf("Device start!!!\r\n");
	
	//delay_init(SystemCoreClock/1000000);
   DAC1_init();
	SPI_Configuration();
//	SPI3_Configuration();

	MYDMA_Config(DMA2_Stream2,DMA_Channel_3,(uint32_t)&SPI1->DR,(uint32_t)Sdata,2048);
	//I2C_Config();
	I2C1_Config();
	
//	if((STMFLASH_ReadWord(PARAMETER_SAVE_ADDR)    == 0xffffffff)||
//		(STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+4)  == 0xffffffff)||
//		(STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+8)  == 0xffffffff)||
//		(STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+12) == 0xffffffff))
//	{}
//	else
//	{
//		Image_Size = STMFLASH_ReadWord(PARAMETER_SAVE_ADDR);
//		VMOD1 = STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+4);
//		VMOD2 = STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+8);
//		BUF = STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+12);
//		//printf(" %x %x\r\n",Image_Size,VMOD1);
//		//STMFlash_EraseSect(FLASH_SAVE_ADDR+12, 256);
//	}
	
	for(i=0;i<=197;i=i+4)
	{  
		if(STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+16+i)==0xffffffff)
		{break;}
		if(i == 197)
		{
			for(i=0;i<200;i++) 
			{
				RegParameter[i] = *((uint8_t*)PARAMETER_SAVE_ADDR+16+i);
				//printf("%x ",RegParameter[i]);
			}
			break;
		}
	}
	//printf("\r\n%d %d 0x%x 0x%x \r\n ",Image_Size,VMOD1,VMOD2,BUF);	
	//Voltage_Init(VMOD1,VMOD2,BUF);
	Voltage_Init(Sdata);
	//RT9367C_Init();
	Both_INA230_Init(Sdata);	
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)Sdata,1024);// Write 4K
	
   USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
   while(DeviceConfigured==0)
	 {
		LED1_OFF; delay_ms(500);
		LED1_ON;  delay_ms(500);
    }
	LED1_ON;

  Burn_off;//close burn device   

		tsl2561_poweron();
		tsl_start();
		delay_ms(300);
		LED3_OFF;
		IWDG_Init(5,2000);		//10s watchdog//2500// 6,1875

//PVD_Config(PWR_PVDLevel_7);
//set_bor(OB_BOR_LEVEL2); //set low power reset

//	VMOD1_Voltage_Set(3300,&i);
//	VMOD1_Calibration(Sdata);
//	 
//	 GPIO_SetBits(GPIOB,GPIO_Pin_14);
//	VMOD2_Calibration(Sdata);
//	 IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,0x1f,1);
	 
  while(1)
  {
	  TaskProcess();
  } 
}

 void PVD_Config(uint8_t pvd)
 {
   NVIC_InitTypeDef NVIC_InitStructure;
   EXTI_InitTypeDef EXTI_InitStructure;
 
   /* Enable PWR clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
 
   /* Configure one bit for preemption priority */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
   
   /* Enable the PVD Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
       
   /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
      falling edges */
   EXTI_ClearITPendingBit(EXTI_Line16);
   EXTI_InitStructure.EXTI_Line = EXTI_Line16;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
 
   /* Configure the PVD Level to 3 (PVD detection level set to 2.5V, refer to the
       electrical characteristics of you device datasheet for more details) */
   PWR_PVDLevelConfig(pvd);
 
   /* Enable the PVD Output */
   PWR_PVDCmd(ENABLE);
 }

void TaskIwdg(void)
{
	IWDG_ReloadCounter();//reload
//	if(low_power_flag==1)
//	{
//		low_power_flag=0;
//		   USBD_Init(&USB_OTG_dev,
//#ifdef USE_USB_OTG_HS 
//            USB_OTG_HS_CORE_ID,
//#else            
//            USB_OTG_FS_CORE_ID,
//#endif
//            &USR_desc, 
//            &USBD_CDC_cb, 
//            &USR_cb);
//   while(DeviceConfigured==0)
//	 {
//		LED1_OFF; delay_ms(500);
//		LED1_ON;  delay_ms(500);
//    }
//	LED1_ON;
//	}
}

void TaskLED(void)
{
	LED2_Toggle;
//	printf("current = %duA\r\n",I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,CURRENT_ADDR)*10);
//	printf("current = %duA\r\n",I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,CURRENT_ADDR)*10);
//	LED3_Toggle;
//	LED4_Toggle;
}

void TaskKeyScan(void)
{
//	BreathingLightProcess();
	KeyProcess();
}

void TaskSendContinueImage(void)
{
	TWO_BYTE_CMD Cmd;

	if(USB_ReceivedCount > 0)
	{//判断是否接收到了USB数据
      USB_ReceivedCount = 0;
		if((DFUDataFlag == 0)&&(DataFlag == 0)&&(GainDataFlag == 0)&&(FlashDataFlag == 0))
		{
			Cmd.Byte[1] = USB_Rx_Buffer[0];
			Cmd.Byte[0] = USB_Rx_Buffer[1];
		}
		
		ProcessNewCmd(&Cmd);
	}
}

void TaskUSBReceiveDataProcess(void)
{
	uint16_t Timer=500;

	if(TouchFire == 1)
	{
		if(Touch_Scan(TouchMod))
		{
			//printf(" %x %x\r\n",Read_PMU(0x01),Read_PMU(0x44));
			Read_PMU(0x01);
			Write_PMU(0xf1,WakeUpReg);
			while((--Timer)&&(Read_PMU(0x44)) != 0x8002)
			{
				delay_ms(1);
			}
			if(Timer == 0)
			{
				SendErrorCode(1,Read_PMU(0x44));
				//printf(" %x \r\n",Read_PMU(0x01));
				//printf(" %x %x %x %x \r\n",Read_PMU(0x46),Read_PMU(0x47),Read_PMU(0x48),Read_PMU(0x49));
				Write_PMU(0xf8,0xff);//restart
				Write_PMU(0xf2,0xff);//sleep
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				TouchFire = 0;
				USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
				return ;
			}
			
			//LED3_ON;
			//delay_ms(1);
			Write_PMU(0x9d,Reg9d2);
			Write_PMU(0x9e,Reg9e2);
			Write_PMU(0xf6,0xff);
			delay_ms(1);
			//printf("%x %x\r\n",Read_PMU(0x1d),Read_PMU(0x1e));
			Write_PMU(0xf3,0x40);
			
			Timer=950;
			while((--Timer)&&(Read_PMU(0x44)) != 0x4069)
			{
				delay_ms(1);
			}
			
			if(Timer == 0)
			{
				SendErrorCode(2,Read_PMU(0x44));
				//printf(" %x \r\n",Read_PMU(0x01));
				//printf(" %x %x %x %x \r\n",Read_PMU(0x46),Read_PMU(0x47),Read_PMU(0x48),Read_PMU(0x49));
				Write_PMU(0xf8,0xff);//restart
				Write_PMU(0xf2,0xff);//sleep
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				TouchFire = 0;
				USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
				return ;
			}
			
			SendOK();
			SendImageData();
			
			Write_PMU(0xf8,0xff);				//restart
			Timer=100;
			while((--Timer)&&(Read_PMU(0x44)) != 0x8002){delay_ms(1);}
			if(Timer == 0)
			{
				SendErrorCode(3,Read_PMU(0x44));
				Write_PMU(0xf8,0xff);//restart
				Write_PMU(0xf2,0xff);//sleep
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				TouchFire = 0;
				USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
				return ;
			}
			Write_PMU(0xf2,0xff);				//sleep
			//delay_ms(3);
			Write_PMU(0x9d,Reg9d1);
			Write_PMU(0x9e,Reg9e1);
			TouchFire = 0;
		}
		else
		{
			SendError();
			TouchFire = 0;
		}
	}
	else if(TouchCMOS == 1)
	{
		if(Touch_Scan(TouchMod))
		{
			Write_PMU(0xcf,0x00);
			Write_PMU(0xf1,0xff);
			delay_ms(5);
			while((--Timer)&&((Read_PMU(0x3f)&0x0400) != 0x0400))
			{
				delay_ms(1);
			}
			if(Timer == 0)
			{
				SendErrorCode(1,Read_PMU(0x3f));
				Write_PMU(0xf8,0xff);//restart
				Write_PMU(0xcf,0x27);
				Write_PMU(0xf2,0xff);//sleep
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				TouchCMOS = 0;
				USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
				return ;
			}
			
			Write_PMU(0xf3,0xff);
			
			Timer=500;
			while((--Timer)&&(Read_PMU(0x3f)) != 0x8B0B)
			{
				delay_ms(1);
			}
			
			if(Timer == 0)
			{
				SendErrorCode(2,Read_PMU(0x3f));
				Write_PMU(0xf8,0xff);//restart
				Write_PMU(0xcf,0x27);
				Write_PMU(0xf2,0xff);//sleep
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				TouchCMOS = 0;
				USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
				return ;
			}
			SendOK();
			SendCMOSImageData();
			
			Write_PMU(0xf8,0xff);				//restart
			Timer=100;
			while((--Timer)&&((Read_PMU(0x3f)&0x0400) != 0x0400)){delay_ms(1);}
			if(Timer == 0)
			{
				SendErrorCode(3,Read_PMU(0x3f));
				Write_PMU(0xf8,0xff);//restart
				Write_PMU(0xcf,0x27);
				Write_PMU(0xf2,0xff);//sleep
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				TouchCMOS = 0;
				USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
				return ;
			}
			Write_PMU(0xcf,0x27);
			Write_PMU(0xf2,0xff);				//sleep
			//delay_ms(3);
			
			TouchCMOS = 0;
		}
		else
		{
			SendError();
			TouchCMOS = 0;
		}
	}
	else if(TouchOPPO_all==1)
	{
		if(Touch_Scan(TouchMod))
		{
				delay_ms(5);
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x05,0x02);
			
//				switch(USB_Rx_Buffer[3])
//			{
//					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9
						
//					break;
//					case 2:
//					Write_PMU(0x00,0x02);//2
//					Write_PMU(0x00,0x0a);//a
//					
//					break;
//			}	
//					
				//delay_ms(200);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(20);
					PWRON_1117;
					delay_ms(2);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
					delay_ms(200);
					TouchOPPO_all=0;
					return ;
				}
				
			
				for(Timer=0;Timer<5;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(20);
						PWRON_1117;
						delay_ms(2);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(200);
						TouchOPPO_all=0;
						return;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
						TouchOPPO_all=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);								
					}
					else
					{
						SendErrorCode(Timer,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(20);
						PWRON_1117;
						delay_ms(2);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(200);
						TouchOPPO_all=0;  
						return ;
					}
					
					


						delay_ms(5);
						Write_PMU(0xef,0x01);
						Write_PMU(0x01,0x00);
						Write_PMU(0x05,0x02);
			
//				switch(USB_Rx_Buffer[3])
//			{
//					case 1:
//						Write_PMU(0x00,0x01);//1
//						Write_PMU(0x00,0x09);//9
						
//					break;
//					case 2:
					Write_PMU(0x00,0x02);//2
					Write_PMU(0x00,0x0a);//a
//					
//					break;
//			}	
//					
				//delay_ms(200);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(20);
					PWRON_1117;
					delay_ms(2);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
					delay_ms(200);
					TouchOPPO_all=0;
					return ;
				}
				
			
				for(Timer=0;Timer<5;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(20);
						PWRON_1117;
						delay_ms(2);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(200);
						TouchOPPO_all=0;
						return;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
						TouchOPPO_all=0;
					}
					else
					{
						SendErrorCode(Timer,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(20);
						PWRON_1117;
						delay_ms(2);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(200);
						TouchOPPO_all=0;  
						return ;
					}
				}
			
		
			TouchOPPO_all = 0;
			}
		}
		
		else
		{
			SendError();
			TouchOPPO_all = 0;
		}
	}
	
	else if(TouchMK210_win==1)
	{
		if(Touch_Scan(TouchMod))
		{
				HANDSHAKE();
/****************************frametime******************************************/
			 if((Line_H!=0)||(Line_L!=0))
				{
					Timer = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(Timer>>8));
					SPI1_ReadWriteByte((uint8_t)Timer);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						
						TouchMK210_win=0;
						
						delay_ms(50);					
						return;
					}
					delay_ms(5);		
			   }
/*******************************************************************************/			
				Timer=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(0x0c);   
				SPI1_ReadWriteByte(0x10);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//yy
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);//yy
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//zz
				SPI1_ReadWriteByte(0);//zz
				
				SPI1_ReadWriteByte((uint8_t)(Timer>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)Timer);//sum2
				
				CS0_High;
//				Write_PMU(0xef,0x01);
//				Write_PMU(0x01,0x00);
//				Write_PMU(0x0a,0x01);
//				Write_PMU(0x01,USB_Rx_Buffer[2]);
//				Write_PMU(USB_Rx_Buffer[3],USB_Rx_Buffer[4]);
//				Write_PMU(USB_Rx_Buffer[5],0x00);
//				Write_PMU(0,0);
				delay_ms(20);//
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);						
					TouchMK210_win=0;
					return;
				}			
			
				for(Timer=0;Timer<2;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
//						printf("22222222222");
						TouchMK210_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);								
						return ;
					}
					
//					CS0_Low;
//					
//					MYDMA_Enable(DMA2_Stream2,7,(uint32_t)DT);
//					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//					DMA_Cmd(DMA2_Stream2, DISABLE);
//					DMA_Cmd(DMA2_Stream3, DISABLE);	
//					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//					
//					CS0_High;
					//add  sth
					CS0_Low;
					
					MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
					DMA_Cmd(DMA2_Stream2, DISABLE);
					DMA_Cmd(DMA2_Stream3, DISABLE);	
					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
					
					CS0_High;
					//SPIReadData(Sdata+num*296*sizeof(uint8_t));
//					for(temp=0;temp<296;temp++)
//					{
//						printf("%02X ",Sdata[temp]);
//					}
//					delay_ms(20);///////////////attention!!!!!////////////////////////////////////
/////////////////////this delay could cause part sample during 3000ms/////////////////////////////
//					printf("22222222222");
					if(ReceEnd())
					{
						SendOK();
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//						PWROFF_1117;
//						delay_ms(100);			
//						printf("22222222222");						
//						SendErrorCode(num,0x0002);
//						break;
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else 
					{
	//					printf("22222222222");
						SendErrorCode(Timer,0x0002);
						TouchMK210_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);								
						return ;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}

					touch_4=0;
					TouchMK210_win = 0;
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
//					delay_ms(5);
		}
	
		else
		{
			SendError();
			TouchMK210_win = 0;
		}
	}	

	else if(TouchOPPO_win==1)
	{
		if(Touch_Scan(TouchMod))
		{
				HANDSHAKE();	
/****************************frametime******************************************/
			 if((Line_H!=0)||(Line_L!=0))
				{
					Timer = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(Timer>>8));
					SPI1_ReadWriteByte((uint8_t)Timer);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						
						TouchOPPO_win=0;
						
						delay_ms(50);					
						return;
					}
					delay_ms(5);		
			   }
/*******************************************************************************/				
				Timer=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(0x0c);   
				SPI1_ReadWriteByte(0x10);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//yy
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);//yy
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//zz
				SPI1_ReadWriteByte(0);//zz
				
				SPI1_ReadWriteByte((uint8_t)(Timer>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)Timer);//sum2
				
				CS0_High;
//				Write_PMU(0xef,0x01);
//				Write_PMU(0x01,0x00);
//				Write_PMU(0x0a,0x01);
//				Write_PMU(0x01,USB_Rx_Buffer[2]);
//				Write_PMU(USB_Rx_Buffer[3],USB_Rx_Buffer[4]);
//				Write_PMU(USB_Rx_Buffer[5],0x00);
//				Write_PMU(0,0);
				delay_ms(20);//
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					TouchOPPO_win=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);						
					return;
				}
											
				for(Timer=0;Timer<2;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						TouchOPPO_win=0;
						return ;
					}
					
//					CS0_Low;
//					
//					MYDMA_Enable(DMA2_Stream2,7,(uint32_t)DT);
//					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//					DMA_Cmd(DMA2_Stream2, DISABLE);
//					DMA_Cmd(DMA2_Stream3, DISABLE);	
//					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//					
//					CS0_High;
					//add  sth
					CS0_Low;
					
					MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
					DMA_Cmd(DMA2_Stream2, DISABLE);
					DMA_Cmd(DMA2_Stream3, DISABLE);	
					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
					
					CS0_High;
					//SPIReadData(Sdata+num*296*sizeof(uint8_t));
//					for(temp=0;temp<296;temp++)
//					{
//						printf("%02X ",Sdata[temp]);
//					}
//					delay_ms(20);///////////////attention!!!!!////////////////////////////////////
/////////////////////this delay could cause part sample during 3000ms/////////////////////////////
					if(ReceEnd())
					{
								
//						SendErrorCode(num,0x0002);
//						break;
						SendOK();
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else 
					{
						SendErrorCode(Timer,0x0002);
						TouchOPPO_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);						
						return ;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				}
			touch_4=0;
			
			TouchOPPO_win = 0;
//			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//			PWROFF_1117;
//			delay_ms(5);

		}
	
		else
		{
			SendError();
			TouchOPPO_win = 0;
		}
	}
	
	else if(Touchmk310_win==1)
	{
		if(Touch_Scan(TouchMod))
		{
				HANDSHAKE();	
/****************************frametime******************************************/
			 if((Line_H!=0)||(Line_L!=0))
				{
					Timer = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(Timer>>8));
					SPI1_ReadWriteByte((uint8_t)Timer);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						
						Touchmk310_win=0;
						
						delay_ms(50);					
						return;
					}
					delay_ms(5);		
			   }
/*******************************************************************************/			
				Timer=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(0x0c);   
				SPI1_ReadWriteByte(0x10);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//yy
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);//yy
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//zz
				SPI1_ReadWriteByte(0);//zz
				
				SPI1_ReadWriteByte((uint8_t)(Timer>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)Timer);//sum2
				
				CS0_High;
//				Write_PMU(0xef,0x01);
//				Write_PMU(0x01,0x00);
//				Write_PMU(0x0a,0x01);
//				Write_PMU(0x01,USB_Rx_Buffer[2]);
//				Write_PMU(USB_Rx_Buffer[3],USB_Rx_Buffer[4]);
//				Write_PMU(USB_Rx_Buffer[5],0x00);
//				Write_PMU(0,0);
				delay_ms(20);//
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					Touchmk310_win=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);						
					return;
				}
											
				for(Timer=0;Timer<2;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						Touchmk310_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);								
						return ;
					}
					
//					CS0_Low;
//					
//					MYDMA_Enable(DMA2_Stream2,7,(uint32_t)DT);
//					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//					DMA_Cmd(DMA2_Stream2, DISABLE);
//					DMA_Cmd(DMA2_Stream3, DISABLE);	
//					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//					
//					CS0_High;
					//add  sth
					CS0_Low;
					
					MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
					DMA_Cmd(DMA2_Stream2, DISABLE);
					DMA_Cmd(DMA2_Stream3, DISABLE);	
					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
					
					CS0_High;
					//SPIReadData(Sdata+num*296*sizeof(uint8_t));
//					for(temp=0;temp<296;temp++)
//					{
//						printf("%02X ",Sdata[temp]);
//					}
//					delay_ms(20);///////////////attention!!!!!////////////////////////////////////
/////////////////////this delay could cause part sample during 3000ms/////////////////////////////
					if(ReceEnd())
					{
//						SendErrorCode(num,0x0002);
//						break;							
						SendOK();
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else 
					{
						SendErrorCode(Timer,0x0002);
						Touchmk310_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						return ;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				}
			touch_4=0;
			
			Touchmk310_win = 0;
//			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//			PWROFF_1117;
//			delay_ms(5);

		}
	
		else
		{
			SendError();
			Touchmk310_win = 0;
		}
	}	
	
	else if(Touchmk510_win==1)
	{
		if(Touch_Scan(TouchMod))
		{
				HANDSHAKE();
/****************************frametime******************************************/
			 if((Line_H!=0)||(Line_L!=0))
				{
					Timer = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(Timer>>8));
					SPI1_ReadWriteByte((uint8_t)Timer);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						
						Touchmk510_win=0;
						
						delay_ms(50);					
						return;
					}
					delay_ms(5);		
			   }
/*******************************************************************************/				
				Timer=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(0x0c);   
				SPI1_ReadWriteByte(0x10);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//yy
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);//yy
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//zz
				SPI1_ReadWriteByte(0);//zz
				
				SPI1_ReadWriteByte((uint8_t)(Timer>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)Timer);//sum2
				
				CS0_High;
//				Write_PMU(0xef,0x01);
//				Write_PMU(0x01,0x00);
//				Write_PMU(0x0a,0x01);
//				Write_PMU(0x01,USB_Rx_Buffer[2]);
//				Write_PMU(USB_Rx_Buffer[3],USB_Rx_Buffer[4]);
//				Write_PMU(USB_Rx_Buffer[5],0x00);
//				Write_PMU(0,0);
//				delay_ms(20);//
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);					
					Touchmk510_win=0;
					return;
				}
											
				for(Timer=0;Timer<2;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);						
						Touchmk510_win=0;
						return ;
					}
					
//					CS0_Low;
//					
//					MYDMA_Enable(DMA2_Stream2,7,(uint32_t)DT);
//					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//					DMA_Cmd(DMA2_Stream2, DISABLE);
//					DMA_Cmd(DMA2_Stream3, DISABLE);	
//					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//					
//					CS0_High;
					//add  sth
					CS0_Low;
					
					MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
					DMA_Cmd(DMA2_Stream2, DISABLE);
					DMA_Cmd(DMA2_Stream3, DISABLE);	
					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
					
					CS0_High;
					//SPIReadData(Sdata+num*296*sizeof(uint8_t));
//					for(temp=0;temp<296;temp++)
//					{
//						printf("%02X ",Sdata[temp]);
//					}
//					delay_ms(20);///////////////attention!!!!!////////////////////////////////////
/////////////////////this delay could cause part sample during 3000ms/////////////////////////////
					if(ReceEnd())
					{
//						SendErrorCode(num,0x0002);
//						break;
						SendOK();						
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else 
					{
						SendErrorCode(Timer,0x0002);
						Touchmk510_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						return ;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				}
			touch_4=0;
			
			Touchmk510_win = 0;
//			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//			PWROFF_1117;
//			delay_ms(100);

		}
	
		else
		{
			SendError();
			Touchmk510_win = 0;
		}
	}
	else if(Touchmk610_win==1)
	{
		if(Touch_Scan(TouchMod))
		{
				HANDSHAKE();
/****************************frametime******************************************/
			 if((Line_H!=0)||(Line_L!=0))
				{
					Timer = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(Timer>>8));
					SPI1_ReadWriteByte((uint8_t)Timer);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						
						Touchmk610_win=0;
						
						delay_ms(50);					
						return;
					}
					delay_ms(5);		
			   }
/*******************************************************************************/				
				Timer=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(0x0c);   
				SPI1_ReadWriteByte(0x10);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//yy
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);//yy
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//zz
				SPI1_ReadWriteByte(0);//zz
				
				SPI1_ReadWriteByte((uint8_t)(Timer>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)Timer);//sum2
				
				CS0_High;
//				Write_PMU(0xef,0x01);
//				Write_PMU(0x01,0x00);
//				Write_PMU(0x0a,0x01);
//				Write_PMU(0x01,USB_Rx_Buffer[2]);
//				Write_PMU(USB_Rx_Buffer[3],USB_Rx_Buffer[4]);
//				Write_PMU(USB_Rx_Buffer[5],0x00);
//				Write_PMU(0,0);
//				delay_ms(20);//
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);					
					Touchmk610_win=0;
					return;
				}
											
				for(Timer=0;Timer<2;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);						
						Touchmk610_win=0;
						return ;
					}
					
//					CS0_Low;
//					
//					MYDMA_Enable(DMA2_Stream2,7,(uint32_t)DT);
//					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//					DMA_Cmd(DMA2_Stream2, DISABLE);
//					DMA_Cmd(DMA2_Stream3, DISABLE);	
//					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//					
//					CS0_High;
					//add  sth
					CS0_Low;
					
					MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
					DMA_Cmd(DMA2_Stream2, DISABLE);
					DMA_Cmd(DMA2_Stream3, DISABLE);	
					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
					
					CS0_High;
					//SPIReadData(Sdata+num*296*sizeof(uint8_t));
//					for(temp=0;temp<296;temp++)
//					{
//						printf("%02X ",Sdata[temp]);
//					}
//					delay_ms(20);///////////////attention!!!!!////////////////////////////////////
/////////////////////this delay could cause part sample during 3000ms/////////////////////////////
					if(ReceEnd())
					{
//						SendErrorCode(num,0x0002);
//						break;
						SendOK();						
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else 
					{
						SendErrorCode(Timer,0x0002);
						Touchmk610_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						return ;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				}
			touch_4=0;
			
			Touchmk610_win = 0;
//			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//			PWROFF_1117;
//			delay_ms(100);

		}
	
		else
		{
			SendError();
			Touchmk610_win = 0;
		}
	}
	else if(Touchmk710_win==1)
	{
		if(Touch_Scan(TouchMod))
		{
				HANDSHAKE();
/****************************frametime******************************************/
			 if((Line_H!=0)||(Line_L!=0))
				{
					Timer = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(Timer>>8));
					SPI1_ReadWriteByte((uint8_t)Timer);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						
						Touchmk710_win=0;
						
						delay_ms(50);					
						return;
					}
					delay_ms(5);		
			   }
/*******************************************************************************/				
				Timer=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(0x0c);   
				SPI1_ReadWriteByte(0x10);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//yy
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);//yy
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//zz
				SPI1_ReadWriteByte(0);//zz
				
				SPI1_ReadWriteByte((uint8_t)(Timer>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)Timer);//sum2
				
				CS0_High;
//				Write_PMU(0xef,0x01);
//				Write_PMU(0x01,0x00);
//				Write_PMU(0x0a,0x01);
//				Write_PMU(0x01,USB_Rx_Buffer[2]);
//				Write_PMU(USB_Rx_Buffer[3],USB_Rx_Buffer[4]);
//				Write_PMU(USB_Rx_Buffer[5],0x00);
//				Write_PMU(0,0);
//				delay_ms(20);//
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);					
					Touchmk710_win=0;
					return;
				}
											
				for(Timer=0;Timer<2;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);						
						Touchmk710_win=0;
						return ;
					}
					
//					CS0_Low;
//					
//					MYDMA_Enable(DMA2_Stream2,7,(uint32_t)DT);
//					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//					DMA_Cmd(DMA2_Stream2, DISABLE);
//					DMA_Cmd(DMA2_Stream3, DISABLE);	
//					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//					
//					CS0_High;
					//add  sth
					CS0_Low;
					
					MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
					DMA_Cmd(DMA2_Stream2, DISABLE);
					DMA_Cmd(DMA2_Stream3, DISABLE);	
					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
					
					CS0_High;
					//SPIReadData(Sdata+num*296*sizeof(uint8_t));
//					for(temp=0;temp<296;temp++)
//					{
//						printf("%02X ",Sdata[temp]);
//					}
//					delay_ms(20);///////////////attention!!!!!////////////////////////////////////
/////////////////////this delay could cause part sample during 3000ms/////////////////////////////
					if(ReceEnd())
					{
//						SendErrorCode(num,0x0002);
//						break;
						SendOK();						
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else 
					{
						SendErrorCode(Timer,0x0002);
						Touchmk710_win=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						return ;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				}
			touch_4=0;
			
			Touchmk710_win = 0;
//			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//			PWROFF_1117;
//			delay_ms(100);

		}
	
		else
		{
			SendError();
			Touchmk710_win = 0;
		}
	}
	
	else if(Touchmk710_win_B==1)
	{
		if(Touch_Scan(TouchMod))
		{
				HANDSHAKE();
/****************************frametime******************************************/
			 if((Line_H!=0)||(Line_L!=0))
				{
					Timer = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(Timer>>8));
					SPI1_ReadWriteByte((uint8_t)Timer);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						
						Touchmk710_win_B=0;
						
						delay_ms(50);					
						return;
					}
					delay_ms(5);		
			   }
/*******************************************************************************/				
				Timer=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(0x0c);   
				SPI1_ReadWriteByte(0x10);
				SPI1_ReadWriteByte(1);   
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);//xx
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//yy
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);//yy
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//ww
				SPI1_ReadWriteByte(0);//zz
				SPI1_ReadWriteByte(0);//zz
				
				SPI1_ReadWriteByte((uint8_t)(Timer>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)Timer);//sum2
				
				CS0_High;
//				Write_PMU(0xef,0x01);
//				Write_PMU(0x01,0x00);
//				Write_PMU(0x0a,0x01);
//				Write_PMU(0x01,USB_Rx_Buffer[2]);
//				Write_PMU(USB_Rx_Buffer[3],USB_Rx_Buffer[4]);
//				Write_PMU(USB_Rx_Buffer[5],0x00);
//				Write_PMU(0,0);
//				delay_ms(20);//
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);					
					Touchmk710_win_B=0;
					return;
				}
											
				for(Timer=0;Timer<2;Timer++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(Timer,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);						
						Touchmk710_win_B=0;
						return ;
					}
					
//					CS0_Low;
//					
//					MYDMA_Enable(DMA2_Stream2,7,(uint32_t)DT);
//					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//					DMA_Cmd(DMA2_Stream2, DISABLE);
//					DMA_Cmd(DMA2_Stream3, DISABLE);	
//					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//					
//					CS0_High;
					//add  sth
					CS0_Low;
					
					MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
					while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
					while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
					DMA_Cmd(DMA2_Stream2, DISABLE);
					DMA_Cmd(DMA2_Stream3, DISABLE);	
					DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
					DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
					
					CS0_High;
					//SPIReadData(Sdata+num*296*sizeof(uint8_t));
//					for(temp=0;temp<296;temp++)
//					{
//						printf("%02X ",Sdata[temp]);
//					}
//					delay_ms(20);///////////////attention!!!!!////////////////////////////////////
/////////////////////this delay could cause part sample during 3000ms/////////////////////////////
					if(ReceEnd())
					{
//						SendErrorCode(num,0x0002);
//						break;
						SendOK();						
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else 
					{
						SendErrorCode(Timer,0x0002);
						Touchmk710_win_B=0;
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						return ;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				}
			touch_4=0;
			
			Touchmk710_win_B = 0;
//			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//			PWROFF_1117;
//			delay_ms(100);

		}
	
		else
		{
			SendError();
			Touchmk710_win_B = 0;
		}
	}	
	
	
}





void TaskProcess(void)
{
	uint8_t i;
	for(i=0;i<TASKS_MAX;i++)
	{
		if(TaskComps[i].Run)
		{
			TaskComps[i].TaskHook();
			TaskComps[i].Run = 0;
		}
	}
}


void TaskRemarks(void)
{
	uint8_t i;
	for(i=0;i<TASKS_MAX;i++)
	{
		if(TaskComps[i].Timer)
		{
			TaskComps[i].Timer--;
			if(TaskComps[i].Timer==0)
			{
				TaskComps[i].Timer = TaskComps[i].ItvTimer;
				TaskComps[i].Run = 1;
			}
		}
	}
}


void TIM3_IRQHandler(void)
{
//	static uint16_t i=0;
	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		TaskRemarks();
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}

void PVD_IRQHandler(void)
{
	low_power_flag=1;
	LED4_ON;
}

int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
//  
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART */
//  USART_SendData(Open_USART, (uint8_t) ch);

//  /* Loop until the end of transmission */
//  while (USART_GetFlagStatus(Open_USART, USART_FLAG_TC) == RESET)
//  {}

//  return ch;
//}

#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


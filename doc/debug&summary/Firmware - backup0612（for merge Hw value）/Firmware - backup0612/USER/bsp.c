#include "bsp.h"
#include "delay.h"

extern  __IO uint8_t USB_StatusDataSended;
extern uint8_t USB_Rx_Buffer[];
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

extern uint16_t SPI1Speed;
uint32_t Image_Size=128*200*2;
uint8_t Sdata[25600];//25k
//uint8_t SPIData[2048];
uint8_t SPI_Data=0xff;
//static const uint8_t FWID[4] __attribute__((at(ADDR_FLASH_SECTOR_11))) ={0x01,0x00,0x00,0x00};
const uint8_t FWID[4]={0x2,0x1,0x15,0x00}; //01实验室 v2.1.21 2019/5/17
//uint8_t DFUID[4];
//uint8_t Size_Reg[4]={180,128,0,0};
uint8_t WakeUpReg = 0x20;
uint8_t Reg9d1 = 0x58,Reg9d2 = 0x54;
uint8_t Reg9e1 = 0x20,Reg9e2 = 0x22;
//uint16_t Dtime=0;

uint8_t RegParameter[250]={
	0x83,0x45,0x00,0x84,0x40,0x00,0x85,0xf3,0x00,0x87,0x80,0x00,0x88,0x95,0x00,0x89,0x40,0x00,//PMU.TXT
	0x8a,0x00,0x00,0x8b,0xa2,0x00,0x8c,0x14,0x00,0x8d,0x4a,0x00,0x8e,0x44,0x00,0x8f,0x40,0x00,
	0x90,0x02,0x00,0x91,0x04,0x00,0x92,0x06,0x00,0x93,0x08,0x00,0x94,0x1f,0x00,0x95,0x1f,0x00,0x96,0x01,0x00,0x97,0x03,0x00,
	0x98,0x05,0x00,0x99,0x07,0x00,0x9a,0xc3,0x00,0x9b,0x03,0x00,0x9c,0x3f,0x00,0x9d,0x58,0x00,0x9e,0x20,0x00,0x9f,0x70,0x00,
	0xa0,0x00,0x00,0xa5,0x10,0x00,0xa6,0x00,0x00,0xa7,0x90,0x00,0xa8,0x04,0x00,0xa9,0x02,0x00,
	0xaa,0x08,0x00,0xab,0x08,0x00,0xac,0x08,0x00,0xad,0x08,0x00,0xae,0x08,0x00,0xaf,0x08,0x00,
	0xb0,0x02,0x00,0xb1,0x88,0x00,0xb2,0x02,0x00,0xb3,0x08,0x00,0xb4,0x08,0x00,0xb5,0x04,0x00,
	0xb6,0x04,0x00,0xba,0x02,0x00,0xbb,0x0f,0x00,0xbe,0x21,0x00,0xbf,0x22,0x00,
	0xc0,0x68,0x00,							//200-96
	0xc1,0x80,0x00,							//128
	0xc2,0x00,0x00,
	0xc3,0x00,0x00,
	0xe8,0x82,0x00,
	0xe9,0x00,0x00,							//00
	0xea,0x48,0x00,							//200-128
	0xeb,0x4f,0x00,
	0xec,0x20,0x00,
	0xa1,0x06,0x00,
	0xf6,0xff,0x14,							//config finish command at least delay11ms
	
	0xf1,0x20,0x14,							//wake up delay20ms
	
	0x9d,0x54,0x00,0x9e,0x22,0x00,0xff,0xff};//ResendPMU.TXT

void TXDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u16 ndtr)
{
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	DMA_DeInit(DMA_Streamx);
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}//等待DMA可配置 
	
	 /* 配置 Tx DMA Stream */
  DMA_InitStructure.DMA_Channel = chx;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(&Sdata);//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//
  DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);//初始化DMA Stream
}	
	
//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar:存储器地址
//ndtr:数据传输量  
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{ 
 
//	uint16_t i;
	DMA_InitTypeDef  DMA_InitStructure;
	
//	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
//	{
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
//		
//	}else 
//	{
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
//	}
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
   DMA_DeInit(DMA_Streamx);
	DMA_DeInit(DMA2_Stream3);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}//等待DMA可配置 
	
  /* 配置 Rx DMA Stream */
  DMA_InitStructure.DMA_Channel = chx;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
  DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
	
	 /* 配置 Tx DMA Stream */
  DMA_InitStructure.DMA_Channel = chx;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(&SPI_Data);//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//外设到存储器模式
  DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);//初始化DMA Stream
//  for(i=0;i<2048;i++)
//  {
//	  SPIData[i]=0xff;
//  }
} 
//开启一次DMA传输
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量  
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr,uint32_t rx_buf)
{
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 Rx
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
//	DMA2_Stream2->M0AR = rx_buf;
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
		
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);	
	DMA_Cmd(DMA2_Stream3, DISABLE);                      //关闭DMA传输 Tx
	
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA2_Stream3,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA2_Stream3, ENABLE);                      //开启DMA传输 
}	  
	
void TIM3_Config(uint16_t arr, uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
   TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x04; //子优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//TIM4 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM4_PWM_Config(uint16_t arr,uint16_t psc,uint16_t PWM_Val)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTD时钟	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); //GPIOD12-13复用为定时器4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); //GPIOD12-13复用为定时器4
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;           //GPIOD12-13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化PD12-13
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM4 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OCInitStructure.TIM_Pulse = PWM_Val;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 4OC1
	TIM_OCInitStructure.TIM_Pulse = PWM_Val;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 4OC2

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
//	TIM_Cmd(TIM4, ENABLE);  //使能TIM4	
	
}

//初始化独立看门狗
//prer:分频数:0~7(只有低3位有效!)
//rlr:自动重装载值,0~0XFFF.
//分频因子=4*2^prer.但最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	
	IWDG_SetPrescaler(prer); //设置IWDG分频系数

	IWDG_SetReload(rlr);   //设置IWDG装载值

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //使能看门狗
}


//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
uint8_t STMFLASH_ReadByte(u32 faddr)
{
	return *(vu8*)faddr; 
} 
void STMFlash_EraseSect(u32 WriteAddr, u32 NumToWrite)
{
	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
   if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
   FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	//endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	endaddr=WriteAddr+NumToWrite;
	if(addrx<0x1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0xFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}

   FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
}
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
   FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
   if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
   FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	//endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	endaddr=WriteAddr+NumToWrite;
	if(addrx<0x1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0xFFFFFFFF)//有非0xFFFFFFFF的地方,要擦除这个扇区
			{   
				//printf("Flash Error!!\r\n");
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)
				{
					printf("Flash Erase Error!!\r\n");
					break;	//发生错误了
				}
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				printf("Flash Write Error!!\r\n");
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 


void MY_STMFLASH_Write(u32 WriteAddr,u8 *pBuffer,u32 NumToWrite)	
{ 
   FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
   if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
   FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	//endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	endaddr=WriteAddr+NumToWrite;
	if(addrx<0x1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0xFFFFFFFF)//有非0xFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramByte(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr++;
			//WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}

void STMFLASH_ReadforByte(u32 ReadAddr,uint8_t *pBuffer,u32 NumToRead)   
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadByte(ReadAddr);//读取1个字节.
		ReadAddr++;//偏移1个字节.	
	}
}	

void RST_PMU(void)
{
	GPIO_ResetBits(GPIOC,RST);
	delay_ms(10);
	GPIO_SetBits(GPIOC,RST);
}
void BUF_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//buf1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//buf2
  //GPIOA1,E9初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE
	
	GPIO_ResetBits(GPIOE,GPIO_Pin_9);//BUF2 B TO A
	
	GPIO_SetBits(GPIOA,GPIO_Pin_1);//BUF1 A TO B
}
void KEY_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟
 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //KEY2 KEY3对应引脚
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE10,11
}
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY2按下
//2，KEY3按下
//注意此函数有响应优先级,KEY2>KEY3!!
uint8_t KEY_Scan(uint8_t mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		

	if(key_up&&(KEY2==0||KEY3==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY2==0&&KEY3==0)return 3;
		else if(KEY2==0)return 1;
		else if(KEY3==0)return 2;
	}else if(KEY2==1&&KEY3==1)key_up=1; 	    
 	return 0;// 无按键按下
}
//fire处理函数
//返回fire值
//mode:0,不支持连续按;1,支持连续按;
//0，没有fire信号
//1，有fire信号

uint8_t Touch_Scan(uint8_t mode)
{	 
	static u8 fire_up=1;//按键按松开标志
	if(mode)fire_up=1;  //支持连按		

	if(fire_up&&(!KEY2))
	{
		if(mode == 0)
		{
			delay_ms(10);//去抖动 
		}
		else
		{
			delay_ms(90);
		}
		
		fire_up=0;
		if(!KEY2)return 1;
	}else if((!KEY2) == 0)fire_up=1; 	    
 	return 0;// 无fire
}




void Both_INA230_Init(uint8_t *INA230Buf)
{
	uint16_t CalibrationData;
	IIC_WriteByte(Open_I2Cx,PMOD1_DET_addr,0x00,0x4267,2);
	delay_ms(1);
	IIC_WriteByte(Open_I2Cx,PMOD2_DET_addr,0x00,0x4267,2);
	delay_ms(1);
	if(INA230Buf[0x1e]==1&&INA230Buf[0x2a]==1)
	{
		CalibrationData=INA230Buf[0x1d]<<8|INA230Buf[0x1c];
		//CalibrationData[1]=INA230Buf[0x34];
		IIC_WriteByte(Open_I2Cx,PMOD1_DET_addr,CAL_ADDR,CalibrationData,2);
		//IIC2Write(PMOD1_DET_addr,CAL_ADDR,CalibrationData,2);
		delay_ms(1);
		
		CalibrationData=INA230Buf[0x19]<<8|INA230Buf[0x18];
//		CalibrationData[0]=INA230Buf[0x45];
//		CalibrationData[1]=INA230Buf[0x44];
		IIC_WriteByte(Open_I2Cx,PMOD2_DET_addr,CAL_ADDR,CalibrationData,2);
		//IIC2Write(PMOD2_DET_addr,CAL_ADDR,CalibrationData,2);
		delay_ms(1);
	}
}



int16_t Voltage_MOD1_DET(void)
{
//	int16_t value;

	return I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,0x02);
}
int16_t Voltage_MOD2_DET(void)
{
//	int16_t value;
	
	return I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,0x02);
}
int16_t Current_MOD1_DET(void)
{
//	int16_t value;
	
	return I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,0x01);
}
int16_t Current_MOD2_DET(void)
{
//	int16_t value;
	
	return I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,0x01);
}



void Voltage_Init(uint8_t *VolSetBuf)
{
	uint16_t Vol1_Dec;
	GPIO_SetBits(GPIOE,GPIO_Pin_0);//EN
	
	if((VolSetBuf[0x10]==0xff&&VolSetBuf[0x11]==0xff)	||
		(VolSetBuf[0x14]==0xff&&VolSetBuf[0x15]==0xff)	||
		(VolSetBuf[0x18]==0xff&&VolSetBuf[0x19]==0xff))
	{
		VMOD1_Voltage_Set(3300,&Vol1_Dec);
		VolSetBuf[0x12]=Vol1_Dec;VolSetBuf[0x13]=Vol1_Dec>>8;
		
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,0x1f,1);//MOD2 start is off
		VolSetBuf[0x14]=0x1f;//VolSetBuf[0x15]=0x0c;
		delay_ms(1);
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,0x1f,1);//BUF start is off 
		VolSetBuf[0x18]=0x1f;//VolSetBuf[0x19]=0x0c;
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		
	}
	else
	{
		Vol1_Dec = VolSetBuf[0x13]<<8|VolSetBuf[0x12];
		Dac1_Dec_Set(Vol1_Dec);
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VolSetBuf[0x14],1);//MOD2 start is off
		delay_ms(1);
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,VolSetBuf[0x18],1);//BUF start is off 
		delay_ms(1);
	}
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x40,0x00,1);//关闭4个LED，电流设为最低。0x4F为开启4个LED
	delay_ms(1);		
}


//void Voltage_Init(uint32_t VT1,uint32_t VT2,uint32_t BF)
//{
//	GPIO_SetBits(GPIOE,GPIO_Pin_0);//EN
//	
//	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VT2,1);//MOD2 start is off
//	delay_ms(1);
//	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BF,1);//BUF start is off 
//	delay_ms(1);
//	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x40,0x00,1);//关闭4个LED，电流设为最低。0x4F为开启4个LED
//	delay_ms(1);
//	Dac1_Set_Vol(VT1);		
//}


#define VMOD1_CAL_DEBUG
void VMOD1_Calibration(uint8_t *R_CalibrationRegister)
{
	float R_Shunt;
//	uint8_t R_Cal[2]={0,0};//R-------ILSB>10uA ?? PLSB>250uW
	uint16_t temp;
	//float cc;
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	delay_ms(50);
	//R Calibration
	//R_Shunt = (float)((I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,VSHUNT_ADDR)*2.5/1000);//------------1000ua
	//R_Shunt = (float)((I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,VSHUNT_ADDR)*2.5/4000);//------------4000ua
	//R_Shunt = (float)(I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,VSHUNT_ADDR)*2.5/7000);//------------7000ua
	R_Shunt = (float)(I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,VSHUNT_ADDR)*2.5/33000);//------------33000ua
	#ifdef VMOD1_CAL_DEBUG
		printf("%.4f\r\n",R_Shunt);
	#endif
	//temp = (uint16_t)(512/R_Shunt);
	temp = (uint16_t)(512000/(1000*R_Shunt));
//	R_Cal[0] = (uint8_t)(temp>>8);//数据写入必须是大端模式
//	R_Cal[1] = (uint8_t)(temp);
//	IIC2Write(PMOD1_DET_addr,CAL_ADDR,R_Cal,2);//write R-CalibrationRegister
	IIC_WriteByte(Open_I2Cx,PMOD1_DET_addr,CAL_ADDR,temp,2);
	#ifdef VMOD1_CAL_DEBUG
		printf("R_CalibrationRegister=%d\r\n",temp);
		//printf("%x,%x\r\n",R_Cal[0],R_Cal[1]);
	#endif
	delay_ms(50);
	
	//temp = (uint16_t)(temp*1000/10*IIC2ReadInt16(PMOD1_DET_addr,CURRENT_ADDR));//--------1000ua
	//temp = (uint16_t)(temp*100/IIC2ReadInt16(PMOD1_DET_addr,CURRENT_ADDR));//--------1000ua
	//temp = (uint16_t)(temp*400/IIC2ReadInt16(PMOD1_DET_addr,CURRENT_ADDR));//--------4000ua
	//temp = (uint16_t)(temp*700/I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,CURRENT_ADDR));//--------7000ua
	temp = (uint16_t)(temp*3300/I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,CURRENT_ADDR));//--------33000ua
	*R_CalibrationRegister = (uint8_t)(temp);//小端模式
	*(++R_CalibrationRegister) = (uint8_t)(temp>>8);
//	*R_CalibrationRegister = (uint8_t)(temp>>8);//大端模式
//	*(++R_CalibrationRegister) = (uint8_t)(temp);
//	R_Cal[0] = (uint8_t)(temp>>8);//数据写入必须是大端模式
//	R_Cal[1] = (uint8_t)(temp);
	#ifdef VMOD1_CAL_DEBUG
		printf("R_CalibrationRegister=%d\r\n",temp);
		//printf("%x,%x\r\n",R_Cal[0],R_Cal[1]);
	#endif
	IIC_WriteByte(Open_I2Cx,PMOD1_DET_addr,CAL_ADDR,temp,2);//updata R-CalibrationRegister
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
}


#define VMOD2_CAL_DEBUG
void VMOD2_Calibration(uint8_t *R_CalibrationRegister)
{
	float R_Shunt;
//	uint8_t R_Cal[2]={0,0};//R-------ILSB>10uA ?? PLSB>250uW
	uint16_t temp;
	//float cc;
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,0x1f,1);	
	delay_ms(50);
	//R Calibration
	//R_Shunt = (float)(I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VSHUNT_ADDR)*2.5/100);//------------100ua
	//R_Shunt = (float)(I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VSHUNT_ADDR)*2.5/1000);//------------1000ua
	//R_Shunt = (float)(I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VSHUNT_ADDR)*2.5/4000);//------------4000ua
	//R_Shunt = (float)(I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VSHUNT_ADDR)*2.5/7000);//------------7000ua
	R_Shunt = (float)(I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VSHUNT_ADDR)*2.5/30000);//------------30000ua
	#ifdef VMOD2_CAL_DEBUG
		printf("%.4f\r\n",R_Shunt);
	#endif
	//temp = (uint16_t)(512/R_Shunt);
	temp = (uint16_t)(512000/(1000*R_Shunt));
//	R_Cal[0] = (uint8_t)(temp>>8);
//	R_Cal[1] = (uint8_t)(temp);
//	IIC2Write(PMOD2_DET_addr,CAL_ADDR,R_Cal,2);//write R-CalibrationRegister
	IIC_WriteByte(Open_I2Cx,PMOD2_DET_addr,CAL_ADDR,temp,2);
	#ifdef VMOD2_CAL_DEBUG
		printf("R_CalibrationRegister=%d\r\n",temp);
		//printf("%x,%x\r\n",R_Cal[0],R_Cal[1]);
	#endif
	delay_ms(50);
	
	//temp = (uint16_t)(temp*100/10*I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,CURRENT_ADDR));//--------100ua
	//temp = (uint16_t)(temp*10/I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,CURRENT_ADDR));//--------100ua
	//temp = (uint16_t)(temp*100/I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,CURRENT_ADDR));//--------1000ua
	//temp = (uint16_t)(temp*400/I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,CURRENT_ADDR));//--------4000ua
	//temp = (uint16_t)(temp*700/I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,CURRENT_ADDR));//--------7000ua
	temp = (uint16_t)(temp*3000/I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,CURRENT_ADDR));//--------30000ua
	*R_CalibrationRegister = (uint8_t)(temp);//小端模式
	*(++R_CalibrationRegister) = (uint8_t)(temp>>8);
//	*R_CalibrationRegister = (uint8_t)(temp>>8);//大端模式
//	*(++R_CalibrationRegister) = (uint8_t)(temp);
//	R_Cal[0] = (uint8_t)(temp>>8);//数据写入必须是大端模式
//	R_Cal[1] = (uint8_t)(temp);
	#ifdef VMOD2_CAL_DEBUG
		printf("R_CalibrationRegister=%d\r\n",temp);
		//printf("%x,%x\r\n",R_Cal[0],R_Cal[1]);
	#endif
	IIC_WriteByte(Open_I2Cx,PMOD2_DET_addr,CAL_ADDR,temp,2);//updata R-CalibrationRegister
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,0x1f,1);
}


//void PWR_ON_DELAY_MS()
//{
//	PWRON_1117;
//	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//	delay_ms(60);
//}
void RT9367C_Init(void)
{
	GPIO_SetBits(GPIOE,GPIO_Pin_0);//EN
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,0x1f,1);//MOD2
	delay_ms(1);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,0x1f,1);//BUF
	delay_ms(1);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x4f,0x00,1);
	delay_ms(1);
}
void BreathingLightProcess(void)
{
	static uint8_t Curt1=0,Curt2=0x1e;
	if(Curt1<=0x1f)
	{
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x4f,Curt1,1);
		//printf(" %x ",Curt1);
		Curt1++;
		if(Curt1==0x20)
		{
			Curt2=0x1e;
		}
	}
	else if(Curt2<0x1f&&Curt2>0)
	{
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x4f,Curt2,1);
		//printf(" %x ",Curt2);
		Curt2--;
		if(Curt2==0)
		{
			Curt1=0;
		}
	}
}
void KeyProcess(void)
{
	uint8_t KEY/*,i*/;
	//uint8_t HandShake[8]={0xEF,0x01,0x01,0x00,0x03,0x17,0x00,0x1B},Ack[9];
	//u8 TX_Data[2]={1,0};
	KEY = KEY_Scan(0);
	
   if(KEY)
	{						   
		switch(KEY)
		{				 
			case KEY2_PRESS:	//控制LED0翻转
				LED1_Toggle;
				LED2_Toggle;
				break;
			case KEY3_PRESS:	//控制LED1翻转	 
				LED3_Toggle;
				LED4_Toggle;
				break;
			case BOTH_PRESS:	//同时控制LED0,LED1翻转 
				//LED2_Toggle;
				break;
		}
	}
}
void IIC_WriteByte(I2C_TypeDef *I2Cx,u8 addr,u8 reg,u32 data,u8 num)
{
  	u32 tmr;
	u8 I2C_Err=0;

    tmr = 1000;
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    while((--tmr)&&I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    if(tmr==0) I2C_Err = 1;

    I2C_GenerateSTART(I2Cx, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))); 
    if(tmr==0) I2C_Err = 1;

    I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter);
    tmr = 1000;		//cannot change
    while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
   if(tmr==0) I2C_Err = 1;
	 
	 I2C_SendData(I2Cx, reg);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
    if(tmr==0) I2C_Err = 1;

	 switch(num)
	 {
		 case 1:
			 I2C_SendData(I2Cx, (uint8_t)data);
			 tmr = 1000;
			 while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
			 if(tmr==0) I2C_Err = 1;
			 break;
		 case 2:
			 I2C_SendData(I2Cx, (uint8_t)(data>>8));
			 tmr = 1000;
			 while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
			 if(tmr==0) I2C_Err = 1;
		 
			 I2C_SendData(I2Cx, (uint8_t)data);
			 tmr = 1000;
			 while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
			 if(tmr==0) I2C_Err = 1;
			 break;
		 case 3:
			 break;
		 case 4:
			 break;
		 default :
			 break;
	 }

    I2C_GenerateSTOP(I2Cx, ENABLE);
	 if(I2C_Err == 1)
	 {
		 LED3_ON;
		 //GPIO_SetBits(GPIOG,GPIO_Pin_1);
	 }
    //I2C_AcknowledgeConfig(I2Cx, DISABLE);
}

void IIC_WriteByte1(I2C_TypeDef *I2Cx,u8 addr,u8 reg,u32 data,u8 num)
{
  	u32 tmr;
	u8 I2C_Err=0;

    tmr = 1000;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    while((--tmr)&&I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    if(tmr==0) I2C_Err = 1;

    I2C_GenerateSTART(I2C1, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))); 
    if(tmr==0) I2C_Err = 1;

    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
    tmr = 1000;		//cannot change
    while((--tmr)&&(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
    if(tmr==0) I2C_Err = 1;
	 
	 I2C_SendData(I2C1, reg);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
    if(tmr==0) I2C_Err = 1;
	 
	 switch(num)
	 {
		 case 1:
			 I2C_SendData(I2C1, (uint8_t)data);
			 tmr = 1000;
			 while((--tmr)&&(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
			 if(tmr==0) I2C_Err = 1;
			 break;
		 case 2:
			 I2C_SendData(I2C1, (uint8_t)(data>>8));
			 tmr = 255;
			 while((--tmr)&&(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
			 if(tmr==0) I2C_Err = 1;
		 
			 I2C_SendData(I2C1, (uint8_t)data);
			 tmr = 255;
			 while((--tmr)&&(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
			 if(tmr==0) I2C_Err = 1;
			 break;
		 case 3:
			 break;
		 case 4:
			 break;
		 default :
			 break;
	 }
	 
    I2C_GenerateSTOP(I2C1, ENABLE);
	 if(I2C_Err == 1)
	 {
		 LED3_ON;
		 //GPIO_SetBits(GPIOG,GPIO_Pin_1);
	 }
    //I2C_AcknowledgeConfig(I2Cx, DISABLE);
}

int16_t I2C_ReadTwoByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t Reg_addr)
{  
    int16_t readout;
    u16 tmr;
	 uint8_t I2C_Err=0;
	 I2C_Err=I2C_Err;
	
    tmr = 1000;
    while((--tmr)&&I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    if(tmr==0) {I2C_Err = 1;}

	 I2C_GenerateSTART(I2Cx, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
    if(tmr==0) {I2C_Err = 1;}

    I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Transmitter);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
    if(tmr==0) {I2C_Err = 1;}
	
	 I2C_SendData(I2Cx, Reg_addr);
	 tmr = 1000;
	 while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	 if(tmr==0) {I2C_Err = 1;}
	
    I2C_GenerateSTART(I2Cx, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)));
    if(tmr==0) {I2C_Err = 1;}

    I2C_Send7bitAddress(I2Cx, I2C_Addr, I2C_Direction_Receiver);
    tmr = 1000;
    while((--tmr)&&(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
    if(tmr==0) {I2C_Err = 1;} 

	 tmr = 1000;
    while((--tmr)&&(!(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)))); 
    if(tmr==0) {I2C_Err = 1;}
	 readout = I2C_ReceiveData(I2Cx);
	 
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    tmr = 1000;
    while((--tmr)&&(!(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)))); 
    if(tmr==0) {I2C_Err = 1;}

    readout = (readout<<8) | I2C_ReceiveData(I2Cx);

    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    return readout;
}
void I2C_Config(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	RCC_AHB1PeriphClockCmd(Open_I2Cx_SDA_GPIO_CLK | Open_I2Cx_SCL_GPIO_CLK,ENABLE);
	
	RCC_APB1PeriphClockCmd(Open_I2Cx_CLK,ENABLE);
	
	//RCC_APB1PeriphResetCmd(Open_I2Cx_SDA_GPIO_CLK | Open_I2Cx_SCL_GPIO_CLK,ENABLE);
	//RCC_APB1PeriphResetCmd(Open_I2Cx_SDA_GPIO_CLK | Open_I2Cx_SCL_GPIO_CLK,DISABLE);
	
	GPIO_PinAFConfig(Open_I2Cx_SDA_GPIO_PORT, Open_I2Cx_SDA_SOURCE, Open_I2Cx_SDA_AF);
	GPIO_PinAFConfig(Open_I2Cx_SCL_GPIO_PORT, Open_I2Cx_SCL_SOURCE, Open_I2Cx_SCL_AF);
	
	GPIO_InitStructure.GPIO_Pin =  Open_I2Cx_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Open_I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  Open_I2Cx_SCL_PIN;
	GPIO_Init(Open_I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);
		
	I2C_DeInit(Open_I2Cx);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2Cx_SLAVE_ADDRESS7;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2Cx_SPEED;
	
	I2C_Cmd(Open_I2Cx, ENABLE);
	I2C_Init(Open_I2Cx, &I2C_InitStructure);
	
	I2C_AcknowledgeConfig(Open_I2Cx, ENABLE);
}
void I2C1_Config(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	
	//RCC_APB1PeriphResetCmd(Open_I2Cx_SDA_GPIO_CLK | Open_I2Cx_SCL_GPIO_CLK,ENABLE);
	//RCC_APB1PeriphResetCmd(Open_I2Cx_SDA_GPIO_CLK | Open_I2Cx_SCL_GPIO_CLK,DISABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	I2C_DeInit(I2C1);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2Cx_SPEED;
	
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &I2C_InitStructure);
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}

//void WriteSizeRegToPMU(void)
//{
//	Write_PMU(0xc0,Size_Reg[0]-96);
//	Write_PMU(0xc1,Size_Reg[1]);
//	Write_PMU(0xe9,0);
//	Write_PMU(0xea,Size_Reg[0]-128);
//}
uint8_t WriteRegToCMOS(void)
{
	uint8_t num;
	uint16_t Timer=500;
	for(num = 0; num < 255 ; num=num+3)	//write reg to PMU
	{
		if((RegParameter[num] == 0xff)&&(RegParameter[num+1] == 0xff))
		{break;}
		else
		{
			Write_PMU(RegParameter[num],RegParameter[num+1]);
			if(RegParameter[num+2]==0){}
			else
			{
				delay_ms(RegParameter[num+2]);
				//printf(" %x %x %x\r\n",RegParameter[num],RegParameter[num+1],RegParameter[num+2]);
			}
			
			if(RegParameter[num] == 0xf1)
			{
				while((--Timer)&&((Read_PMU(0x3f)&0x0400) != 0x0400/*0x8002*/))
				{
					delay_ms(1);
				}
				if(Timer==0)
				{
					SendErrorCode(1,Read_PMU(0x3f));
					//printf("8002 Error!\r\n");
					LED4_ON;
					return 0;
				}
			}
			
		}
	}
	return 1;
}
uint8_t WriteRegToPMU(void)
{
	uint8_t num;
	uint16_t Timer=500;
	for(num = 0; num < 200 ; num=num+3)	//write reg to PMU
	{
		if((RegParameter[num] == 0xff)&&(RegParameter[num+1] == 0xff))
		{break;}
		else
		{
			Write_PMU(RegParameter[num],RegParameter[num+1]);
			if(RegParameter[num+2]==0){}
			else
			{
				delay_ms(RegParameter[num+2]);
				//printf(" %x %x %x\r\n",RegParameter[num],RegParameter[num+1],RegParameter[num+2]);
			}
			
			if(RegParameter[num] == 0xf1)
			{
				while((--Timer)&&((Read_PMU(0x44)) != 0x8002))
				{
					delay_ms(1);
				}
				if(Timer==0)
				{
					SendErrorCode(1,Read_PMU(0x44));
					//printf("8002 Error!\r\n");
					LED4_ON;
					return 0;
				}
				#ifdef	DEBUG
					printf("\r\nRead INT success\r\n");
				#endif
			}
			
		}
	}
	return 1;
}
uint8_t SendContinueImage(uint8_t cmd1,uint8_t cmd2,uint16_t value)
{
//	while((Read_PMU(0x44)) != 0x8002);
//#ifdef	DEBUG
//	printf("\r\nRead INT success\r\n");
//#endif
	uint16_t Timer = 950;
	
	Write_PMU(cmd1,cmd2);
	//GPIO_SetBits(GPIOE,GPIO_Pin_5);
	while((--Timer)&&(Read_PMU(0x44) != value)){delay_ms(1);}
	if(Timer==0)
	{
		SendErrorCode(2,Read_PMU(0x44));
		return 0;
	}
#ifdef	DEBUG
	printf("\r\nRead ROIC data success\r\n");
#endif
	SendOK();
	SendImageData();
	return 1;
}

void SendSingleFrameImage(uint8_t cmd)
{
	uint16_t Timer = 950;
//	while((Read_PMU(0x44)) != 0x8002);
//	printf(" %x\r\n",cmd);
	Write_PMU(0xf3,cmd);
//	LED4_ON;
	while((--Timer)&&(Read_PMU(0x44) != 0x4069))
	{
		delay_ms(1);
	}
	if(Timer==0)
	{
		SendErrorCode(2,Read_PMU(0x44));
		return ;
	}
#ifdef	DEBUG
	printf("\r\nRead ROIC data success\r\n");
#endif
	SendOK();
	SendImageData();
}
void SendCMOSImageData(void)
{
	uint16_t i=Image_Size;
	
	Write_PMU(0xfc,0xff);
	Write_PMU(0xff,0xff);
	
	CS0_Low;

	for(i=Image_Size;i>512;i=i-512)
	{
		MYDMA_Enable(DMA2_Stream2,512,(uint32_t)Sdata);
		
		while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);	
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);

		while(USB_StatusDataSended==0);
		USB_StatusDataSended = 0;
		DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
	}
	
	MYDMA_Enable(DMA2_Stream2,i,(uint32_t)Sdata);
		
	while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);	
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);

	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
	
	CS0_High;
//	Read_data(Sdata);
	
//	Write_PMU(0xff,0xff);
//	Write_PMU(0xff,0xff);
//	Write_PMU(0xff,0xff);
//	Write_PMU(0xff,0xff);
}
void SendImageData(void)
{
	Write_PMU(0xfc,0xff);
	Write_PMU(0xff,0xff);
	Write_PMU(0xff,0xff);
	
	Read_data(Sdata);
	
//	while(USB_StatusDataSended==0);
//	USB_StatusDataSended = 0;
//	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,Image_Size);
	
	Write_PMU(0xff,0xff);
	Write_PMU(0xff,0xff);
	Write_PMU(0xff,0xff);
	Write_PMU(0xff,0xff);
}
void SendOK(void)
{
	uint8_t OKdata[]={0xf0,0x00,0xff,0xff};
   while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
   DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,OKdata,PackSize);					//等待数据发送完毕,实际测试发现必须加延时函数才能正常收发数据
	//DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,NULL,0);
}
void SendError(void)
{
	uint8_t Errordata[]={0xf0,0x01,0xff,0xff};	
   while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
   DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Errordata,PackSize);
}
void SendErrorCode(uint8_t Code,uint16_t Result)
{
	static uint8_t ErrorCodedata[]={0xf0,0x02,0,0,0,0xff,0xff};
	ErrorCodedata[2] = Code;
   ErrorCodedata[3] = (uint8_t)(Result>>8);
	ErrorCodedata[4] = (uint8_t)Result;
	
   while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
   DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,ErrorCodedata,PackSize);
}

void Send_FWID(void)
{
	Sdata[0] = 0xf0;
   Sdata[1] = 0x00;
	
	Sdata[2] = FWID[0];
   Sdata[3] = FWID[1];
	Sdata[4] = FWID[2];
	
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)(Sdata+5),1);
//	Sdata[6] = DFUID[0];
//   Sdata[7] = DFUID[1];
//	Sdata[8] = DFUID[2];
	
   Sdata[10] = 0xff;
   Sdata[11] = 0xff;
	
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
   DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
	
}

void Command(uint8_t *parameter)
{
	
}

void USART_Configuration(void)
{
	
  //GPIO端口设置
   GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
   USART_Init(USART1, &USART_InitStructure); //初始化串口1
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
   USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
}
void SPI3_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStruct;	 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC /*| RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOC*/,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,  GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//SCK
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//MOSI
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //MISO
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
//	GPIO_InitStructure.GPIO_Pin = Open_SPIx_MISO_PIN;
//	GPIO_Init(Open_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = Open_SPIx_MOSI_PIN;
//	GPIO_Init(Open_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
   
	//这里只针对SPI口初始化
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//复位SPI1
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1
   
	SPI_I2S_DeInit(SPI3);
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;		//SPI_CPOL_High  SPI_CPOL_Low
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;		//SPI_CPHA_2Edge  SPI_CPHA_1Edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI3, &SPI_InitStruct);

	SPI_Cmd(SPI3, ENABLE);
	//SPI1_ReadWriteByte(0xff);//启动传输	维持MOSI为高电平
	//SPI_I2S_ClearITPendingBit(Open_SPIx, SPI_I2S_IT_RXNE);
	
	RCC_AHBxPeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//	RCC_AHBxPeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;					//CS0
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CS1 | RST;			//CS1 RST
//   GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStruct;	 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(Open_SPIx_SCK_GPIO_CLK | Open_SPIx_MISO_GPIO_CLK | Open_SPIx_MOSI_GPIO_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(Open_RCC_APB2Periph_SPIx,ENABLE);
	
	GPIO_PinAFConfig(Open_SPIx_SCK_GPIO_PORT, Open_SPIx_SCK_SOURCE,  Open_SPIx_MOSI_AF);
	GPIO_PinAFConfig(Open_SPIx_MISO_GPIO_PORT, Open_SPIx_MISO_SOURCE, Open_SPIx_MOSI_AF);
	GPIO_PinAFConfig(Open_SPIx_MOSI_GPIO_PORT, Open_SPIx_MOSI_SOURCE, Open_SPIx_MOSI_AF);
	
	GPIO_InitStructure.GPIO_Pin = Open_SPIx_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
	GPIO_Init(Open_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = Open_SPIx_MOSI_PIN;
	GPIO_Init(Open_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
		
	GPIO_InitStructure.GPIO_Pin = Open_SPIx_MISO_PIN; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(Open_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);//初始化
//	GPIO_InitStructure.GPIO_Pin = Open_SPIx_MISO_PIN;
//	GPIO_Init(Open_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = Open_SPIx_MOSI_`PIN;
//	GPIO_Init(Open_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
   
	//这里只针对SPI口初始化
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1
   
	SPI_I2S_DeInit(Open_SPIx);
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;		//SPI_CPOL_High  SPI_CPOL_Low
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;		//SPI_CPHA_2Edge  SPI_CPHA_1Edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(Open_SPIx, &SPI_InitStruct);

	SPI_Cmd(Open_SPIx, ENABLE);
	//SPI1_ReadWriteByte(0xff);//启动传输	维持MOSI为高电平
	//SPI_I2S_ClearITPendingBit(Open_SPIx, SPI_I2S_IT_RXNE);
	
	RCC_AHBxPeriphClockCmd(RCC_GPIO_CS,ENABLE);
	RCC_AHBxPeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
   GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CS0;					//CS0
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(GPIO_CS_PORT, &GPIO_InitStructure);
	CS0_High;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_CS1 | RST;			//CS1 RST
   GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	CS1_High;
	GPIO_SetBits(GPIOC,RST);
}
void SPI1_ReConfig(void)
{
	SPI_InitTypeDef SPI_InitStruct;
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1
   
	SPI_I2S_DeInit(Open_SPIx);
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;		//SPI_CPOL_High  SPI_CPOL_Low
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;		//SPI_CPHA_2Edge  SPI_CPHA_1Edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(Open_SPIx, &SPI_InitStruct);

	SPI_Cmd(Open_SPIx, ENABLE);
}
void SPI1_SetDirection(uint32_t SPI_Direction)
{
	assert_param(IS_SPI_DIRECTION_MODE(SPI_Direction));//判断有效性
	
	SPI1->CR1&=0xFBFF;//位10清零，用来设置工作模式
	SPI1->CR1|=SPI_Direction;	//设置SPI1工作模式
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
	//SPI1_ReadWriteByte(0xff);
}
//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
} 

//#define MOD1_VOLTAGE_DEBUG
uint8_t VMOD1_Voltage_Set(uint16_t Voltage,uint16_t *DAC_Dec)
{
//	double R1_Value;
//	double Vol_MOD1=Voltage/1000;
	uint16_t Dec;
	uint16_t Vol_MOD1;
	uint16_t temp;
	uint8_t num=50;
	
	if(Voltage==0)
	{GPIO_ResetBits(GPIOB,GPIO_Pin_14);}
	
	if(Voltage!=0)
	{
		
//	Dec = (uint16_t)(((Voltage-1250)/1200)*4096/3.3);
//	Dec = (uint16_t)((Voltage-1250)*4096/3960);
	Dec = (Voltage-1250)*512/495;
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	
	#ifdef MOD1_VOLTAGE_DEBUG
		printf("MOD1 voltage set.\r\n");
	#endif
	do
	{
		Dac1_Dec_Set(Dec);
		
		Voltage_MOD1_DET();
		
		delay_ms(50);
		Vol_MOD1 = Voltage_MOD1_DET()*125/100;
		temp = (Vol_MOD1 > Voltage) ? (Vol_MOD1 - Voltage) : (Voltage - Vol_MOD1);
		#ifdef MOD1_VOLTAGE_DEBUG
			printf("%d,%d,%d\r\n",Dec,Vol_MOD1,temp);
		#endif
		Dec = (Vol_MOD1 > Voltage) ? (Dec - temp) : (Dec + temp);
	}while((temp>=10)&&(--num));
	
	//PWR_MOD1_EN(0);
	*DAC_Dec = Dec;
	
	}
	
	if(num == 0)
	{
		//#ifdef MOD1_VOLTAGE_DEBUG
			printf("MOD1 voltage set ERROR?\r\n");
		//#endif
		return ERROR;
	}
	else
	{
		#ifdef MOD1_VOLTAGE_DEBUG
			printf("MOD1 voltage set SUCCESS!\r\n");
		#endif
		return SUCCESS;
	}
}

void Dac1_Dec_Set(uint16_t Dec)
{
	DAC_SetChannel1Data(DAC_Align_12b_R,Dec);
}


void Dac1_Set_Vol(uint16_t vol)
{
	double temp=vol;
	if(temp <= 1250)
	{
		DAC_SetChannel1Data(DAC_Align_12b_R,0);
	}
	else
	{
		temp = (temp - 1250)/1165;//1200----->1165
		temp=temp*4096/3.3;
		DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
	}
}

void DAC1_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);		//使能DAC时钟
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;			//模拟输入注意
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//下拉GPIO_PuPd_DOWN
  GPIO_Init(GPIOA, &GPIO_InitStructure);					//初始化

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //初始化DAC通道1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC通道1
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
}

void GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//RT9367

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//Power EN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				//rst
//	GPIO_Init(GPIOH, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				//RTEN
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				//PE2------G0361_LED1
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				//PE3------G0361_LED2
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				//PE4------FLASH_CS
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				//PC4------FLASH_CS_MK510
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				//PE5
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				//PE6
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE,GPIO_Pin_0);			//RTEN
	GPIO_ResetBits(GPIOE,GPIO_Pin_2);		//PE2------G0361_LED1
	GPIO_ResetBits(GPIOE,GPIO_Pin_3);		//PE3------G0361_LED2
	GPIO_SetBits(GPIOE,GPIO_Pin_4);			//PE4------FLASH_CS	
	GPIO_SetBits(GPIOC,GPIO_Pin_4);			//PE4------FLASH_CS_MK510		
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);		//PE5
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);		//PE6
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);		//Power EN
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //FIRE对应引脚
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉  GPIO_PuPd_NOPULL-----> GPIO_PuPd_DOWN
   GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE14
	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//NST
//	
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	GPIO_ResetBits(GPIOC,GPIO_Pin_9);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//Power EN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
}

void LED_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOH时钟

  //GPIOE12,13,14,15初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//LED1,LED2,LED3,LED4对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE
	
//	GPIO_SetBits(GPIOE,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);//LED1,LED2,LED3,LED4设置高，灯亮
	
	GPIO_SetBits(GPIOE,GPIO_Pin_12 | GPIO_Pin_13);//GPIOF14,F15设置高，灯亮
	GPIO_ResetBits(GPIOE,GPIO_Pin_14 | GPIO_Pin_15);//GPIOG0,G1设置低，灯灭
}

void Start_Process(void)
{
	uint8_t KEY;
	uint16_t InvailData;
	
	KEY = KEY_Scan(0);
	if(KEY==BOTH_PRESS)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_9);//BUF2 A TO B
		GPIO_SetBits(GPIOA,GPIO_Pin_1);//BUF1 A TO B
		I2C1_Config();
		DAC1_init();
		Both_INA230_Init(Sdata);
		
		   USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
#else            
            USB_OTG_FS_CORE_ID,
#endif
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
		
		VMOD1_Voltage_Set(3300,&InvailData);
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,0x1f,1);
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,0x1f,1);
		printf("VMOD1 Voltage set 3300mV\r\n");
		printf("VMOD1 Voltage result = %dmV\r\n",I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,VOLTAGE_ADDR)*125/100);
		//IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,0x1f,1);
		printf("VMOD2 Voltage set 3300mV\r\n");
		printf("VMOD2 Voltage result = %dmV\r\n",I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VOLTAGE_ADDR)*125/100);
		//IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,0x1f,1);
		printf("BUFFER Voltage set 3300mV\r\n");
		MX_GPIO_TEST_Init();
		delay_ms(100);
		I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,VSHUNT_ADDR);
		I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VSHUNT_ADDR);
		printf("VMOD1 CURRENT = %d \r\n",I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,VSHUNT_ADDR));
		printf("VMOD2 CURRENT = %d uA\r\n",I2C_ReadTwoByte(Open_I2Cx,PMOD2_DET_addr,VSHUNT_ADDR)*5);
		while(1)
		{
			KeyProcess();
		}
	}
	
}

void MX_GPIO_TEST_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//RT9367

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_6|GPIO_Pin_7;//Power EN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//Power EN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_13;//Power EN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;//Power EN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_3);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_1);
	GPIO_SetBits(GPIOC,GPIO_Pin_4);
	GPIO_SetBits(GPIOC,GPIO_Pin_5);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	
	GPIO_SetBits(GPIOE,GPIO_Pin_4);
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);
	GPIO_SetBits(GPIOE,GPIO_Pin_7);
	GPIO_SetBits(GPIOE,GPIO_Pin_8);

}


void delay(uint16_t i)
{
	for(;i>0;i--)
	{}
}

uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
	
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个byte  数据
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
 
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
 		    
}  
uint8_t Read_Byte(void)
{
	uint8_t data;
//	delay_ms(1);
	CS0_Low;

	data = SPI1_ReadWriteByte(0xff);
	//data = (data<<8)|SPI1_ReadWriteByte(0xff);

	CS0_High;
	
	return data;
}
uint16_t Read_PMU(uint8_t cmd)
{
	uint16_t data;
	CS0_Low;

	SPI1_ReadWriteByte(cmd);//发送读取INT命令	    
	SPI1_ReadWriteByte(0xff);

	CS0_High;
//	delay_ms(1);
	CS0_Low;

	data = SPI1_ReadWriteByte(0xff);
	data = (data<<8)|SPI1_ReadWriteByte(0xff);

	CS0_High;
	
	return data;
}

void Read_data(uint8_t *data)
{
	uint16_t i;
//	uint16_t j,k;

	CS0_Low;
	
	if(Image_Size == 35840)//丢弃M0301的第一行 256 Byte数据
	{
		MYDMA_Enable(DMA2_Stream2,256,(uint32_t)Sdata);
		while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);	
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
	}
//	MYDMA_Enable(DMA2_Stream2,512,(uint32_t)Sdata);
	for(i=0;i<Image_Size;i=i+512)
	{
//		if(i%2 == 0)
//		{j = 512;k=0;}
//		else
//		{j = 0;k=512;}
		MYDMA_Enable(DMA2_Stream2,512,(uint32_t)Sdata);
		while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);	
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//		if((Image_Size-i)<=512){}
//		else{MYDMA_Enable(DMA2_Stream2,512,(uint32_t)(Sdata+j));}
		while(USB_StatusDataSended==0);
		USB_StatusDataSended = 0;
		DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
	}
	
	CS0_High;
//	MYDMA_Enable(DMA2_Stream2,Image_Size,(uint32_t)Sdata);
//	while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//	DMA_Cmd(DMA2_Stream2, DISABLE);
//	DMA_Cmd(DMA2_Stream3, DISABLE);	
//	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//	
//	while(USB_StatusDataSended==0);
//	USB_StatusDataSended = 0;
//	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,Image_Size);
//	
//	CS0_High;
	
//	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);
//	MYDMA_Enable(DMA2_Stream2,Image_Size,(uint32_t)Sdata);
//	SPI1_SetDirection(SPI_Direction_2Lines_RxOnly);
//	while(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2)==RESET);
//	CS0_High;
//	SPI_Cmd(SPI1,DISABLE);
//	SPI1_ReConfig();
//	SPI1_SetSpeed(SPI1Speed);
//	DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);
	
//	for(i=0;i<Image_Size;i=i+2)
//	{
//		data[i] = SPI1_ReadWriteByte(0xff);
//	   data[i+1] = SPI1_ReadWriteByte(0xff);
//	}
//	for(i=0;i<512;i++)
//	{
//		for(j=0;j<512;j++)
//		{
//			data[j] = SPI1_ReadWriteByte(0xff);
//			if((i*512+j+1)==Image_Size)
//			{
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
//				CS0_High;
//				return ;
//			}
//		}
//		DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,data,512);
//	}

	//CS0_High;
}
void Write_PMU(uint8_t addr, uint8_t data)
{
	CS0_Low;
	
	SPI1_ReadWriteByte(addr);   
	SPI1_ReadWriteByte(data);
	
	CS0_High;
}
void SendUserNamePassWord(void)
{
	Sdata[0] = 0xf0;
	Sdata[1] = 0x00;
	STMFLASH_ReadforByte(USER_INFOR_SAVE_ADDR,Sdata+2,128);
	Sdata[128] = 0xff;
	Sdata[129] = 0xff;
	
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
   DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
}
void UserNamePassWordSave(void)
{
	uint8_t i;
	STMFLASH_ReadforByte(FLASH_SAVE_ADDR,Sdata,372);
	
	for(i=0;i<128;i++)
	{
		Sdata[12+i] = USB_Rx_Buffer[i+2];
	}
	
	MY_STMFLASH_Write(USER_INFOR_SAVE_ADDR,Sdata,372);
	SendOK();
}



/*********** End **************/



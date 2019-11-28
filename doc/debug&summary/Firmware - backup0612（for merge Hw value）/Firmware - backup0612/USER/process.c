
#include "bsp.h"
#include "delay.h"
#include "w25qxx.h"
#include "stdlib.h"				//abs函数
#include "stdio.h"
#include "tsl2561.h"
#include "string.h"

//#define mk110
//#define mk510_first_gain_addr 0x0 
//#define mk510_second_gain 0x180000

volatile uint8_t Line_H=0,Line_L=0,version_state=0,version_time=0/*,SensorTpye*/;
uint8_t mk510_which_gain =0;
uint8_t mk510_last_pack=0;
uint16_t led0pwmval=0;
uint32_t mk510_first_gain_addr	=0x0;
uint32_t mk510_second_gain_addr	=0x180000;

//uint8_t LineTimePara[2],u8Res,MKImagePart[2];
uint8_t Module_Cur_Detec_EN;
uint32_t ImageSize,ModuleCurrent=0;
static void sDev_SPI_SendCmd(u8 Cmd, u8 ParaLen, u8* ParaBuf);

uint8_t iCount;

const u8 cu8EndBuf[2] = {0xAA, 0x00};
const u8 u8SPICmdHeader[2] = {0xEF, 0x01};

static const uint16_t CrcCCITTTable[256] =
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};



typedef enum _EN_SPIM_RECV_STATUS
{
	EN_SPIM_RECV_NULL = 0,
	EN_SPIM_RECV_HEADER_INCOMP,//receive header incomplete
	EN_SPIM_RECV_IDENTIFY,
	EN_SPIM_RECV_DATA_START,	//receive complete
	EN_SPIM_RECV_DATA_FINISH,   //receive complete
	EN_SPIM_RECV_DATA_ERROE,
	EN_SPIM_RECV_DATA_TIMEOUT,
	
}EN_SPIM_RECV_STATUS;

typedef enum _EN_SPI_CMD
{
	/*normal function API*/
	EN_SPI_CMD_CAPTURE_WIN_IMAGE					= 0x01,
	EN_SPI_CMD_CAPTURE_GAIN_IMAGE					= 0x02, 
	EN_SPI_CMD_CAPTURE_MK_IMAGE					= 0x02,
	EN_SPI_CMD_CLEAR_FRAM							= 0x03,
	EN_SPI_CMD_WRITE_GAIN							= 0x04,
	EN_SPI_CMD_WRITE_GAIN_FOR_TEST_BOARD		= 0x50+EN_SPI_CMD_WRITE_GAIN,
	EN_SPI_CMD_READ_GAIN								= 0x05,
	EN_SPI_CMD_READ_GAIN_FOR_TEST_BOARD			= 0x50+EN_SPI_CMD_READ_GAIN,
	EN_SPI_CMD_UPDATA_CODE							= 0x06,
	EN_SPI_CMD_GET_FW_VERSION						= 0x07,
	EN_SPI_CMD_WRITE_SN								= 0x08,
	EN_SPI_CMD_READ_SN								= 0x09,
	EN_SPI_CMD_WRITE_CLEAR_FRAME_SET				= 0x0A,
	EN_SPI_CMD_RAED_CLEAR_FRAME_SET				= 0x0B,
	EN_SPI_CMD_RAED_FRAME_LINE_SET				= 0x81,
	EN_SPI_CMD_WRITE_GAIN_INFOR					= 0x18,
	EN_SPI_CMD_READ_GAIN_INFOR						= 0x19,
	EN_SPI_CMD_SLEEP_MODULE							= 0xF1,
	EN_SPI_CMD_MODULE_VCOM_SW						= 0xF2,
	EN_SPI_CMD_MODULE_HARDWARE_SET				= 0xF3,

	/*send window image data with simpify SPI protocal*/
	EN_SPI_CMD_CAPTURE_WIN_IMAGE_SIMPLIFY			= 0x10,

	/*clr frame use normal capture window*/
	EN_SPI_CMD_CLR_FRAME_CPTURE						= 0x11,

//#if DEBUG_CLR_DOUBLE_CAPTURE_EN
	EN_SPI_CMD_FAST_CAPTURE_WIN_IMAGE				= 0x21,
	EN_SPI_CMD_FAST_CAPTURE_GAIN_IMAGE				= 0x22, 
//#endif

	/*the API taht capture window image use gain way*/
	EN_SPI_CMD_CAPTURE_WIN_IMAGE_SIMPLIFY_MGGAIN	= 0x30,
	EN_SPI_CMD_CAPTURE_WIN_IMAGE_MG_GAIN			= 0x31,

	/*API:set timing sequence parameter*/
	EN_SPI_CMD_SET_INTEGRAL_LINE			= 0x80,
	EN_SPI_CMD_SET_CAPTURE_IMAGE_PARA		= 0x84,
	EN_SPI_CMD_SET_CLEAR_FRAME_PARA			= 0x85,

	/*for checking SPI interface is OK or not*/
	EN_SPI_CMD_HANDSHAKE									= 0x17, 

	EN_SPI_CMD_BONDING_RESIST_FOR_EXPENDING_BOARD= 0x18, 
	
}EN_UART_CMD;



volatile uint8_t ContinueFlag,auto_burn_cnt,SensorGain,SensorGain_mk210,SensorGain_mk310;
volatile uint8_t SensorGain_mk710,SensorGain_mk710_B,SensorGain_mk510,SensorGain_mk610,mk510_read_gain=0,Sensor_Update,cnnt,code_cnt=0,capture_num=0;
uint16_t SPI1Speed = SPI_BaudRatePrescaler_8;

extern uint8_t DFUDataFlag,DataFlag,GainDataFlag,FlashDataFlag;
extern uint8_t TouchFire,TouchCMOS,TouchOPPO_all,TouchOPPO_win,TouchMK210_win,Touchmk310_win,Touchmk510_win,Touchmk610_win,Touchmk710_win,Touchmk710_win_B;
extern uint8_t TouchMod;
extern __IO uint16_t VMOD1,VMOD2,BUF;

extern uint32_t Image_Size;
extern uint8_t RegParameter[];

extern uint8_t WakeUpReg;
extern uint8_t Reg9d1,Reg9d2;
extern uint8_t Reg9e1,Reg9e2;
//extern uint16_t Dtime;

extern uint8_t Sdata[];
extern uint8_t USB_Rx_Buffer[];
extern  __IO uint8_t USB_StatusDataSended;
extern  uint32_t USB_ReceivedCount;
extern __ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

extern uint16_t ch0;
extern uint16_t ch1;//tsl2561

void MOD1_MOD2_Buffer_Set(uint8_t *VoltageSetDataBuf);
void Voltage_Send_Error(uint8_t *ErrorBuf,uint8_t ErrorType);
void Power_MOD1_MOD2_Calibration(uint8_t *CalibrationBuf);
void Key_Detect(uint8_t *KeyDetectBuf);
void Capture_Full_Image(uint8_t *CaptureFullImageBuf);
void Capture_Win_Image(uint8_t *CaptureWinImageBuf);
void Write_Module_Gain(uint8_t *WriteGainBuf);
void Read_Module_Gain(uint8_t *ReadGainBuf);
void Normal_Current_Detect(uint8_t *NormalCurrentBuf);
void Module_SW_Vcom(uint8_t *VcomSwitchBuf);
void ClearFrameSet(uint8_t *ClearFrameBuf);
void ReadClearFrameSetting(uint8_t *ReadClearFrameBuf);
void ReadFrameLineSetting(uint8_t *ReadFrameLineBuf);
void ModlueHardwareSetting(uint8_t *ModuleHardwareBuf);

uint16_t Check_Sum16(uint8_t* Buf, uint32_t Length);
void SPI_Receive_Data(uint8_t *ReceiveDataBuf,uint16_t num);
void SPI_Transmit_Data(uint8_t *TransmitDataBuf,uint16_t num);
void SPI_Receive_Data_DMA(uint8_t *ReceiveDataBuf,uint16_t num1,uint16_t num2,uint8_t StartFlag,uint8_t LastFlag);
void SPI_DMA_Wait(void);
static u8 sDev_SPI_WaitRes(u8* Result, u16 RecvParaLen, u8* RecvParaBuf, u32 OutTime,uint8_t msDelayFlag);
uint8_t Receive_Packet_Head(uint8_t DelayTimeOneByte);
uint8_t Receive_Packet_End(void);
void USB_Send(uint8_t *SendData,uint32_t Sendlength);
void USB_Receive(uint8_t *ReceiveData,uint32_t Receivelength);

static u8 sDev_SPI_WaitRes(u8* Result, u16 RecvParaLen, u8* RecvParaBuf, u32 OutTime,uint8_t msDelayFlag)
{	
//	u8 *pu8RecvParaBuf=NULL;
	u8 pu8RecvParaBuf[30];
	u8 u8RecvByte,/*u8SendBuf=0xff,*/u8ReturnRes;
	volatile u32 vu32Locatick, vu32ByteCnt = 0;
	volatile EN_SPIM_RECV_STATUS enSpimRecvStuts = EN_SPIM_RECV_NULL;
	volatile u16 u16PacLen ;
	u16 u16CalChecksum, u16RecvChecksum;
	uint16_t Cunt=0;

	memset(pu8RecvParaBuf, 0xFF, sizeof(pu8RecvParaBuf));
	vu32Locatick = OutTime;
	
//	pu8RecvParaBuf = new_malloc(2+1+2+1+RecvParaLen+2);
//	if(pu8RecvParaBuf == NULL)
//	{
////		printf("sApp_Uart_RespondCmd malloc pResBuf %d failed\n",(6+1+2+1+ParaLen+2));
////		printf("left free Ram size:%d\n",xPortGetFreeHeapSize());
//		printf("sDev_SPI_WaitRes malloc error!");
//		return ERROR;
//	}
	
	while(1)
	{		
		u8RecvByte = 0xff;
		SPI_Receive_Data(&u8RecvByte,1);

		switch(enSpimRecvStuts)
		{
			case EN_SPIM_RECV_NULL:
				if(u8RecvByte == 0xEF)
				{
					enSpimRecvStuts = EN_SPIM_RECV_HEADER_INCOMP;
				}
				if(msDelayFlag==1)
				{
					delay_ms(1);
				}
				break;

			case EN_SPIM_RECV_HEADER_INCOMP:
				if(u8RecvByte == 0x01)
				{
					enSpimRecvStuts = EN_SPIM_RECV_IDENTIFY;
					if(Module_Cur_Detec_EN)
					{
//						printf("\r\n0x%x,0x%x",CurrentCunt,Cunt);
						ModuleCurrent = ModuleCurrent/Cunt*10;
//						printf("\r\n0x%x",CurrentCunt);
						Module_Cur_Detec_EN=0;
					}
				}
				else if(u8RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = EN_SPIM_RECV_NULL;
				}
				break;

			case EN_SPIM_RECV_IDENTIFY:
				if(u8RecvByte == 0x07)
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_START;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			case EN_SPIM_RECV_DATA_START:
				pu8RecvParaBuf[vu32ByteCnt++] = u8RecvByte;
				if(vu32ByteCnt >= 2)
				{
					u16PacLen = (u16)pu8RecvParaBuf[0]<<8 | pu8RecvParaBuf[1];
					
					if(vu32ByteCnt>= u16PacLen+2)
					{
						enSpimRecvStuts = EN_SPIM_RECV_DATA_FINISH;
					}
				}
				break;

			default:
				break;
		}
		
		if(Module_Cur_Detec_EN)
		{
			if(Cunt==0)
			{
				ModuleCurrent = 0;
				I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,CURRENT_ADDR);
			}
			ModuleCurrent += I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,CURRENT_ADDR);
			Cunt++;
		}

		if((enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH) \
			||(enSpimRecvStuts == EN_SPIM_RECV_DATA_ERROE))
		{
			break;
		}
		
		vu32Locatick--;
		if(vu32Locatick == 0)
		{
			enSpimRecvStuts = EN_SPIM_RECV_DATA_TIMEOUT;
			break;
		}
	}


	if(enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH)
	{
		u16CalChecksum = Check_Sum16(pu8RecvParaBuf, u16PacLen) + 0x07;/*cal checksum shold add identify*/
		u16RecvChecksum = ((u16)pu8RecvParaBuf[u16PacLen])<<8 | pu8RecvParaBuf[u16PacLen+1];
		if(u16CalChecksum != u16RecvChecksum)
		{
			printf("checksum err, cal checksum:%04X, recv checksum:%04X\n",u16CalChecksum,u16RecvChecksum);
//			new_free(pu8RecvParaBuf);
//			return 0;
			u8ReturnRes = 0;
		}
		else
		{
			*Result = pu8RecvParaBuf[2];
			if(RecvParaLen)
			{
				memcpy(RecvParaBuf, &pu8RecvParaBuf[3], RecvParaLen);
			}
//			new_free(pu8RecvParaBuf);
//			return 1;
			u8ReturnRes = 1;
		}
	}
	else if(enSpimRecvStuts == EN_SPIM_RECV_DATA_TIMEOUT)
	{
		printf("receive respond time out\n");
//		new_free(pu8RecvParaBuf);
//		return 0;
		u8ReturnRes = 0;
	}
	else
	{
		printf("receive data err\n");
//		new_free(pu8RecvParaBuf);
//		return 0;
		u8ReturnRes = 0;
	}
	
//	new_free(pu8RecvParaBuf);
	return u8ReturnRes;
}

void SPI_Transmit_Data(uint8_t *TransmitDataBuf,uint16_t num)
{		 			 
	uint16_t i;
	uint8_t invaildata;
	
	CS0_Low;
	for(i=0;i<num;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
		SPI_I2S_SendData(SPI1, TransmitDataBuf[i]); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
		invaildata = SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
	}	
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空 
	CS0_High;
}  

void SPI_Receive_Data(uint8_t *ReceiveDataBuf,uint16_t num)
{
	uint16_t i;
	
	CS0_Low;
	for(i=0;i<num;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
		SPI_I2S_SendData(SPI1, ReceiveDataBuf[i]); //通过外设SPIx发送一个byte  数据
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
		ReceiveDataBuf[i] = SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
	}
	CS0_High;
}


void SPI_Receive_Data_DMA(uint8_t *ReceiveDataBuf,uint16_t num1,uint16_t num2,uint8_t StartFlag,uint8_t LastFlag)
{
	if(StartFlag == 1)
	{
		CS0_Low;
		memset(ReceiveDataBuf,0xff,num1+num2);
		MYDMA_Enable(DMA2_Stream2,num1,(uint32_t)Sdata);
	}
	
	while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);	
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
		
	if(StartFlag == 1)
	{
		MYDMA_Enable(DMA2_Stream2,num2,(uint32_t)(Sdata+num1));
	}
	else
	{
		if(LastFlag!=1)
		{
			memset(ReceiveDataBuf,0xff,num1);
			MYDMA_Enable(DMA2_Stream2,num1,(uint32_t)Sdata);
		}
	}
	
	if(LastFlag==1)
	{CS0_High;}
}


void SPI_DMA_Wait(void)
{
	while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);	
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
	CS0_High;
}



uint8_t Receive_Packet_Head(uint8_t DelayTimeOneByte)
{
	uint8_t RecvByte,enSpimRecvStuts=0;
	uint16_t Cunt=0;
	
	while(1)
	{
		RecvByte = 0xff;
		SPI_Receive_Data(&RecvByte,1);
		switch(enSpimRecvStuts)
		{
			case 0:
				if(RecvByte == 0xEF)
				{
					enSpimRecvStuts = 1;
				}
				delay_ms(DelayTimeOneByte);
				break;

			case 1:
				if(RecvByte == 0x01)
				{
					return 1;
				}
				else if(RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = 0;
				}
				break;

			default:
				break;
		}
		
		Cunt++;
		if(Cunt==2000)
		{
			return 0;
		}
	}
}

uint8_t Receive_Packet_End(void) 
{
	uint8_t EndData[2];
	uint8_t FinResult;
	
//	Read_Byte(EndData,2);
	memset(EndData,0xff,2);
	SPI_Receive_Data(EndData,2);
	
	if((EndData[0] != 0xAA) ||( EndData[1] != 0x00))
	{
		printf("receive package end fail:%02X, %02X\n",EndData[0],EndData[1]);
		FinResult = 0;
	}
	else
	{
		FinResult = 1;
	}
	
	return FinResult;
}


void USB_Send(uint8_t *SendData,uint32_t Sendlength)
{
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
   DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,SendData,Sendlength);
}


void USB_Receive(uint8_t *ReceiveData,uint32_t Receivelength)
{
	DCD_EP_PrepareRx (&USB_OTG_dev,CDC_OUT_EP,(uint8_t *)USB_Rx_Buffer,Receivelength);
	memcpy(ReceiveData,USB_Rx_Buffer,Receivelength);	
}


uint16_t Check_Sum16(uint8_t* Buf, uint32_t Length)
{
	uint32_t i;
	uint16_t CheckSum = 0;

	for(i=0;i<Length;i++)
	{
		CheckSum += Buf[i];
	}
	return CheckSum;	
}

static void sDev_SPI_SendCmd(u8 Cmd, u8 ParaLen, u8* ParaBuf)
{
	u8 pCmdBuf[77];
	u16 u16PackageLen;
	u16 u16Checksum = 0;

	/*	
		package header(2 bytes) + package ID(1 byte) + 
		package length(2 bytes) + CMD(1 byte) +
		CMD parameters(ParaLen bytes) + checksum(2 bytes)
	*/
	
	memcpy(pCmdBuf, u8SPICmdHeader,2);
	pCmdBuf[2] = 0x01;/*package respond CMD ID*/
	u16PackageLen = 1+ParaLen+2;/*u16PackageLen = Cmd(1 byte) + ParaLen + Checksum(2 bytes)*/
	pCmdBuf[3] = (u8)(u16PackageLen>>8);/*package length*/
	pCmdBuf[4] = (u8)(u16PackageLen&0xFF);/*package length*/
	pCmdBuf[5] = Cmd;/*assume success*/ 
	if(ParaLen)/*paraLen > 0*/
	{
		memcpy(&pCmdBuf[6], ParaBuf, ParaLen);
	}
	u16Checksum = Check_Sum16(&pCmdBuf[2], (4+ParaLen));
	pCmdBuf[6+ParaLen] = (u8)(u16Checksum>>8);/*package length*/
	pCmdBuf[7+ParaLen] = (u8)(u16Checksum&0xFF);/*package length*/
	
//	CS0(0);
//	HAL_SPI_Transmit(&hspi1,pCmdBuf,8+ParaLen,10000);
//	CS0(1);
	SPI_Transmit_Data(pCmdBuf,8+ParaLen);
	
//	new_free(pCmdBuf);
}

uint8_t Handshake(void)
{
	uint8_t u8RecvByte;
	uint16_t TIMER=0;
	volatile EN_SPIM_RECV_STATUS enSpimRecvStuts;	
	
	enSpimRecvStuts = EN_SPIM_RECV_NULL;
	
	Write_PMU(0xef,0x01);
	Write_PMU(0x01,0x00);
	Write_PMU(0x03,0x17);
	Write_PMU(0x00,0x1b);//checksum
//	delay_ms(5);
	
	while(1)
	{
		
		TIMER++;
//		delay_us(10);
		u8RecvByte = Read_Byte();
		switch(enSpimRecvStuts)
		{
			case EN_SPIM_RECV_NULL:
				if(u8RecvByte == 0xEF)
				{
					enSpimRecvStuts = EN_SPIM_RECV_HEADER_INCOMP;
				}
				else 
				{
					delay_ms(10);
				}
				break;

			case EN_SPIM_RECV_HEADER_INCOMP:
				if(u8RecvByte == 0x01)
				{
					enSpimRecvStuts = EN_SPIM_RECV_IDENTIFY;
				}
				else if(u8RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = EN_SPIM_RECV_NULL;
				}
				break;

			case EN_SPIM_RECV_IDENTIFY:
				if(u8RecvByte == 0x07)
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_START;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			case EN_SPIM_RECV_DATA_START:
				//u8RecvParaBuf[vu32ByteCnt++] = u8RecvByte;
				if(u8RecvByte == 0x00)
				{
					u8RecvByte = Read_Byte();
					if(u8RecvByte == 0x03)
					{
						u8RecvByte = Read_Byte();
						if(u8RecvByte == 0x00)
						{
							u8RecvByte = Read_Byte();
							if(u8RecvByte == 0x00)
							{
								u8RecvByte = Read_Byte();
								if(u8RecvByte == 0x0A)
								{
									enSpimRecvStuts = EN_SPIM_RECV_DATA_FINISH;
								}
								else
								{
									enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
								}	
							}
							else
							{
								enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
							}	
						}
						else
						{
							enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
						}	
					}
					else
					{
						enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
					}	
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			default:
				break;
		}
		
		if((enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH) \
			||(enSpimRecvStuts == EN_SPIM_RECV_DATA_ERROE))
		{
			break;
		}

		if(TIMER == 1000)
		{
			enSpimRecvStuts = EN_SPIM_RECV_DATA_TIMEOUT;
			break;
		}
	}
	
	if(enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH)
	{
		printf("Handshake OK\n");
		return 1;
	}
	else if(enSpimRecvStuts == EN_SPIM_RECV_DATA_TIMEOUT)
	{
		printf("receive respond time out\n");
		return 0;
	}
	else
	{
		printf("receive data err\n");
		return 0;
	}
	
//	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//	PWROFF_1117;
}

void HANDSHAKE()
{
	PWRON_1117;
//	delay_ms(15);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(60);
   
}
uint8_t ReadyFor_module_sn_cmd(void)
{
	uint8_t u8RecvByte;
	uint8_t i;
	uint16_t TIMER=0;
	volatile EN_SPIM_RECV_STATUS enSpimRecvStuts;
	
	enSpimRecvStuts = EN_SPIM_RECV_NULL;
	
	while(1)
	{
		TIMER++;
		u8RecvByte = Read_Byte();
		switch(enSpimRecvStuts)
		{
			case EN_SPIM_RECV_NULL:
				if(u8RecvByte == 0xEF)
				{
					enSpimRecvStuts = EN_SPIM_RECV_HEADER_INCOMP;
				}
				delay_ms(1);
				break;

			case EN_SPIM_RECV_HEADER_INCOMP:
				if(u8RecvByte == 0x01)
				{
					enSpimRecvStuts = EN_SPIM_RECV_IDENTIFY;
				}
				else if(u8RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = EN_SPIM_RECV_NULL;
				}
				break;

			case EN_SPIM_RECV_IDENTIFY:
				if(u8RecvByte == 0x07)
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_START;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			case EN_SPIM_RECV_DATA_START:
				//u8RecvParaBuf[vu32ByteCnt++] = u8RecvByte;
				if(u8RecvByte == 0x00)
				{
					u8RecvByte = Read_Byte();
					if(u8RecvByte == 0x43)
					{
							u8RecvByte = Read_Byte();
							if(u8RecvByte==0x00)
							{
								Sdata[0]=0xf0;
								Sdata[1]=0x00;
//								Read_Byte();
								for(i=0;i<64;i++)
								{
									Sdata[2+i]=Read_Byte();
								}
								Sdata[66]=0xff;
								Sdata[67]=0xff;
								Read_Byte();
								Read_Byte();
								printf("endnenndndndndndn\r\n");
								while(USB_StatusDataSended==0) 
								printf("ssss \r\n");
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
								printf("endnenndndndndndn\r\n");
								enSpimRecvStuts=EN_SPIM_RECV_DATA_FINISH;
							}
							else
							{
								enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
							}	
							break;
					}
					else
					{
						enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
					}	
					break;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			default:
				delay_ms(1);
				break;
		}
		
		if((enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH) \
			||(enSpimRecvStuts == EN_SPIM_RECV_DATA_ERROE))
		{
			break;
		}

		if(TIMER == 2000)
		{
			enSpimRecvStuts = EN_SPIM_RECV_DATA_TIMEOUT;
			break;
		}	
		
	}
	
	if(enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH)
	{
		//printf("receive CMD OK\n");
		return 1;
	}
	else if(enSpimRecvStuts == EN_SPIM_RECV_DATA_TIMEOUT)
	{
		printf("receive respond time out\n");
		return 0;
	}
	else
	{
		printf("receive data err\n");
		return 0;
	}
}

uint8_t ReadyForMK_VOL(void)
{
	uint8_t u8RecvByte;
	uint16_t TIMER=0;
	volatile EN_SPIM_RECV_STATUS enSpimRecvStuts;
	
	enSpimRecvStuts = EN_SPIM_RECV_NULL;
	
	while(1)
	{
		
		TIMER++;
		u8RecvByte = Read_Byte();
//		printf("died here %d    timer:%d \r\n",enSpimRecvStuts,TIMER);
//		delay_ms(2);
		switch(enSpimRecvStuts)
		{
			case EN_SPIM_RECV_NULL:	
			if(u8RecvByte == 0xEF)
				{
					enSpimRecvStuts = EN_SPIM_RECV_HEADER_INCOMP;
				}
				delay_ms(1);
				break;

			case EN_SPIM_RECV_HEADER_INCOMP:
				if(u8RecvByte == 0x01)
				{
					enSpimRecvStuts = EN_SPIM_RECV_IDENTIFY;
				}
				else if(u8RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = EN_SPIM_RECV_NULL;
				}
				break;

			case EN_SPIM_RECV_IDENTIFY:
				if(u8RecvByte == 0x07)
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_START;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			case EN_SPIM_RECV_DATA_START:
				//u8RecvParaBuf[vu32ByteCnt++] = u8RecvByte;
				if(u8RecvByte == 0x00)
				{
					u8RecvByte = Read_Byte();
					if(u8RecvByte == 0x05)
					{
						u8RecvByte = Read_Byte();
						if(u8RecvByte == 0x00)
						{
							printf("%x ",Read_Byte()); 
							printf("%x \n",Read_Byte());
							enSpimRecvStuts = EN_SPIM_RECV_DATA_FINISH;
							
						}
						else
						{
							enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
							
						}	
					}
					else
					{
						enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
					}	
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;
 
			default:
				delay_ms(1);
				break;
		}
		
		if((enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH) \
			||(enSpimRecvStuts == EN_SPIM_RECV_DATA_ERROE))
		{
			break;
		}

		if(TIMER == 3000)// 60000
		{
			enSpimRecvStuts = EN_SPIM_RECV_DATA_TIMEOUT;
			break;
		}
	}
	
	if(enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH)
	{
		//printf("receive CMD OK\n");
		return 1;
	}
	else if(enSpimRecvStuts == EN_SPIM_RECV_DATA_TIMEOUT)
	{
		printf("receive respond time out\n");
		return 0;
	}
	else
	{
		printf("receive data err\n");
		return 0;
	}
}

uint8_t ReadyFor_moduleCMD(void)
{
	uint8_t u8RecvByte;
	uint16_t TIMER=0;
	volatile EN_SPIM_RECV_STATUS enSpimRecvStuts;
	
	enSpimRecvStuts = EN_SPIM_RECV_NULL;
	
	while(1)
	{
		TIMER++;
		u8RecvByte = Read_Byte();
		switch(enSpimRecvStuts)
		{
			case EN_SPIM_RECV_NULL: 
				if(u8RecvByte == 0xEF)
				{
					enSpimRecvStuts = EN_SPIM_RECV_HEADER_INCOMP;
				}
				delay_ms(1);
				break;

			case EN_SPIM_RECV_HEADER_INCOMP:
				if(u8RecvByte == 0x01)
				{
					enSpimRecvStuts = EN_SPIM_RECV_IDENTIFY;
				}
				else if(u8RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = EN_SPIM_RECV_NULL;
				}
				break;

			case EN_SPIM_RECV_IDENTIFY:
				if(u8RecvByte == 0x07)
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_START;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			case EN_SPIM_RECV_DATA_START:
				//u8RecvParaBuf[vu32ByteCnt++] = u8RecvByte;
				if(u8RecvByte == 0x00)
				{
					u8RecvByte = Read_Byte();
					if(u8RecvByte == 0x07)
					{
							Sdata[0] = 0xf0;
						   Sdata[1] = 0x00;
						
							Read_Byte();
							Sdata[2] = Read_Byte();
						   Sdata[3] = Read_Byte();
							Sdata[4] = Read_Byte();
							Sdata[5] = Read_Byte();
						
						   Sdata[6] = 0xff;
							Sdata[7] = 0xff;							
							Read_Byte();
						   Read_Byte();
//							printf("^^^^^^^^6666666666666\r\n");
							while(USB_StatusDataSended==0);
//							printf("^^^^88888888888\r\n");
							USB_StatusDataSended = 0;
						   DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							enSpimRecvStuts = EN_SPIM_RECV_DATA_FINISH;
							break;
					}
					else
					{
						enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
					}	
					break;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			default:
				delay_ms(1);
				break;
		}
		
		if((enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH) \
			||(enSpimRecvStuts == EN_SPIM_RECV_DATA_ERROE))
		{
			break;
		}

		if(TIMER == 50)
		{
			enSpimRecvStuts = EN_SPIM_RECV_DATA_TIMEOUT;
			break;
		}
	}
	
	if(enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH)
	{
		//printf("receive CMD OK\n");
		return 1;
	}
	else if(enSpimRecvStuts == EN_SPIM_RECV_DATA_TIMEOUT)
	{
		printf("receive respond time out\n");
		return 2;
	}
	else
	{
		printf("receive data err\n");
		return 0;
	}
}


uint8_t ReadyForCMD(void)
{
	uint8_t u8RecvByte;
	uint16_t TIMER=0;
	volatile EN_SPIM_RECV_STATUS enSpimRecvStuts;
	
	enSpimRecvStuts = EN_SPIM_RECV_NULL;
	
	while(1)
	{
		
		TIMER++;
		u8RecvByte = Read_Byte();
//		printf("died here %d    timer:%d \r\n",enSpimRecvStuts,TIMER);
//		delay_ms(2);
		switch(enSpimRecvStuts)
		{
			case EN_SPIM_RECV_NULL:	
			if(u8RecvByte == 0xEF)
				{
					enSpimRecvStuts = EN_SPIM_RECV_HEADER_INCOMP;
				}
				delay_ms(1);
				break;

			case EN_SPIM_RECV_HEADER_INCOMP:
				if(u8RecvByte == 0x01)
				{
					enSpimRecvStuts = EN_SPIM_RECV_IDENTIFY;
				}
				else if(u8RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = EN_SPIM_RECV_NULL;
				}
				break;

			case EN_SPIM_RECV_IDENTIFY:
				if(u8RecvByte == 0x07)
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_START;
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;

			case EN_SPIM_RECV_DATA_START:
				//u8RecvParaBuf[vu32ByteCnt++] = u8RecvByte;
				if(u8RecvByte == 0x00)
				{
					u8RecvByte = Read_Byte();
					if(u8RecvByte == 0x03)
					{
						u8RecvByte = Read_Byte();
						if(u8RecvByte == 0x00)
						{
							u8RecvByte = Read_Byte();
							if(u8RecvByte == 0x00)
							{
								u8RecvByte = Read_Byte();
								if(u8RecvByte == 0x0A)
								{
									enSpimRecvStuts = EN_SPIM_RECV_DATA_FINISH;
									//printf("升级成功\r\n");
								}
								else
								{
									enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
								}	
							}
							else
							{
								enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
							}	
						}
						else
						{
							enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
							
						}	
					}
					else
					{
						enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
					}	
				}
				else
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_ERROE;
				}
				break;
 
			default:
				delay_ms(1);
				break;
		}
		
		if((enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH) \
			||(enSpimRecvStuts == EN_SPIM_RECV_DATA_ERROE))
		{
			break;
		}

		if(TIMER == 2000)// 60000
		{
			enSpimRecvStuts = EN_SPIM_RECV_DATA_TIMEOUT;
			break;
		}
	}
	
	if(enSpimRecvStuts == EN_SPIM_RECV_DATA_FINISH)
	{
		//printf("receive CMD OK\n");
		return 1;
	}
	else if(enSpimRecvStuts == EN_SPIM_RECV_DATA_TIMEOUT)
	{
		printf("receive respond time out\n");
		return 0;
	}
	else
	{
		printf("receive data err\n");
		return 0;
	}
}

void DMAReceData(void)
{
	
}

void SPIReadData(uint8_t *data)
{
//		uint16_t i;
//	uint16_t j,k;

	CS0_Low;

	
	MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)data);
	while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
	while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	DMA_Cmd(DMA2_Stream3, DISABLE);	
	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
//		if((Image_Size-i)<=512){}
//		else{MYDMA_Enable(DMA2_Stream2,512,(uint32_t)(Sdata+j));}
//	while(USB_StatusDataSended==0);
//	USB_StatusDataSended = 0;
//	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
	
	
	CS0_High;
}

uint8_t ReceHead(void)
{
	uint8_t u8RecvByte;
	uint16_t Cunt=0;
	volatile EN_SPIM_RECV_STATUS enSpimRecvStuts;
	
	enSpimRecvStuts = EN_SPIM_RECV_NULL;
	
	while(1)
	{
		u8RecvByte = Read_Byte();
		switch(enSpimRecvStuts)
		{
			case EN_SPIM_RECV_NULL:
				if(u8RecvByte == 0xEF)
				{
					enSpimRecvStuts = EN_SPIM_RECV_HEADER_INCOMP;
				}
				break;

			case EN_SPIM_RECV_HEADER_INCOMP:
				if(u8RecvByte == 0x01)
				{
					enSpimRecvStuts = EN_SPIM_RECV_DATA_START;
					return 1;
				}
				else if(u8RecvByte != 0xEF)/*eliminate the specail case that double '0xEF'*/
				{
					enSpimRecvStuts = EN_SPIM_RECV_NULL;
				}
				break;


			default:
				break;
		}
		
		Cunt++;
		if(Cunt==2000)
		{
			return 0;
		}
	}
}
uint8_t ReceEnd1(void) //55aa end 
{
	uint8_t u8EndData[2];
	uint8_t FinResult;
	
	u8EndData[0] = Read_Byte();
	u8EndData[1] = Read_Byte();
	if((u8EndData[0] != 0x55) ||( u8EndData[1] != 0xaa))
	{
		printf("receive package end fail:%02X, %02X\n",u8EndData[0],u8EndData[1]);
		FinResult = 0;
	}
	else
	{
		FinResult = 1;
	}
	
	return FinResult;
}
uint8_t ReceEnd(void) //aa00 end
{
	uint8_t u8EndData[2];
	uint8_t FinResult;
	
	u8EndData[0] = Read_Byte();
	u8EndData[1] = Read_Byte();
	if((u8EndData[0] != 0xAA) ||( u8EndData[1] != 0x00))
	{
		printf("receive package end fail:%02X, %02X\n",u8EndData[0],u8EndData[1]);
		FinResult = 0;
	}
	else
	{
		FinResult = 1;
	}
	
	return FinResult;
}

void ProcessNewCmd(TWO_BYTE_CMD *CMD)
{
	uint16_t num;
	uint16_t temp;
	uint8_t cmd1,cmd2,SmartPhoneMod,Restart,Rst;
	uint8_t DT[8];
	static uint8_t RegBE;
//	static uint8_t TX_Data[2]={1,0};
	static uint32_t CheckCRC,CodeLength;
//	printf(" 0x%x \r\n",CMD->TwoByte);
	
	if(DataFlag == 1)
	{
		if(CodeDataCopy(Sdata,CodeLength)) //copy code to 0x08040000 
		{
//			for(num=0;num<1024;num++)
//			{printf("%x ",Sdata[num]);}
			//i=0;
			
			if(CheckCRC == CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0))
			{
//				CodeLength = CodeLength/4;
//				if((CodeLength%4) != 0)
//				{
//					CodeLength++;
//				}

				//STMFLASH_Write(APP_DEFAULT_ADD,(uint32_t*)Sdata,CodeLength+1);
				LED1_OFF;
				STMFLASH_Write(FLASH_SAVE_ADDR+4,&CodeLength,1);
				STMFLASH_Write(FLASH_SAVE_ADDR+8,&CheckCRC,1);
				
				LED3_ON;
				STMFLASH_ReadforByte(FLASH_SAVE_ADDR,Sdata,500);//新写法
				Sdata[0] = 1;
				STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)Sdata,500);
				//STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)TX_Data,1);//旧写法，增加DFU版本后出现更新程序失败的情况，改成新写法
				
				//printf("Last packet num: %d \r\n",TT++);
				SendOK();
				//TT=0;
				DataFlag = 0;
				while(USB_StatusDataSended==0);
				USB_StatusDataSended = 0;
				NVIC_SystemReset();
				TIM_Cmd(TIM3,DISABLE);
				while(1);
			}
			else
			{
				LED4_ON;
				printf("\r\nCode data error: %x  %x\r\n",CheckCRC,CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0));
				DataFlag = 0;
				SendError();
			}
		}
		
	}
	
//	else if(mk510_read_gain==1)
//	{
////		SendOK();
//		printf("55e9!!!!!!!!!!!!");
//		mk510_read_gain--;
//		if(mk510_which_gain==1)
//		{
//			W25QXX_Read(Sdata,mk510_first_gain_addr,25600);
//			mk510_first_gain_addr+=25600;
//		}
//		else if(mk510_which_gain==2)
//		{
//			W25QXX_Read(Sdata,mk510_second_gain_addr,25600);
//			mk510_second_gain_addr+=25600;						
//		}		
////		SendOK();
//		while(USB_StatusDataSended==0);
//		USB_StatusDataSended = 0;
//		DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
//		if(mk510_read_gain==0)
//		{			
//				mk510_first_gain_addr=0;
//				mk510_second_gain_addr=0x180000;

//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(20);		
//		}
//	}	
	else if(DFUDataFlag == 1)
	{
		if(CodeDataCopy(Sdata,CodeLength))
		{
			if(CheckCRC == CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0))
			{
				STMFLASH_Write(STM32_FLASH_BASE,(uint32_t*)APP_DEFAULT_ADD,CodeLength+1);
				LED1_OFF;
				STMFlash_EraseSect(APP_DEFAULT_ADD, CodeLength+1);
				LED3_ON;
				//printf(" %d \r\n",TT++);
				SendOK();
				//TT=0;
				DFUDataFlag = 0;
				
				while(USB_StatusDataSended==0);
				USB_StatusDataSended = 0;
				NVIC_SystemReset();
				TIM_Cmd(TIM3,DISABLE);
				while(1);
			}
			else
			{
				LED4_ON;
				printf("\r\n%x  %x\r\n",CheckCRC,CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0));
				DFUDataFlag = 0;
				SendError();
			}
		}
	}
	else if(GainDataFlag == 1)
	{
		if(CodeDataCopy(Sdata,CodeLength))
		{
			if(CheckCRC == CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0))
			{
//				LED1_OFF;
//				STMFLASH_Write(APP_DEFAULT_ADD,&CodeLength,4);
//				STMFLASH_Write(APP_DEFAULT_ADD+4,&CheckCRC,4);
				
				LED3_ON;
				SendOK();
				GainDataFlag = 0;
			}
			else
			{
				LED4_ON;
				printf("\r\n%x  %x\r\n",CheckCRC,CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0));
				GainDataFlag = 0;
				SendError();
			}
		}
	}
	else if(FlashDataFlag == 1)
	{
		if(CodeDataCopy(Sdata,CodeLength))
		{
			if(CheckCRC == CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0))
			{
				PWRON_1117;
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
				delay_ms(2);
				
				W25QXX_Write((uint8_t*)APP_DEFAULT_ADD,0x4000,CodeLength+1);
				
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				LED3_ON;
				FlashDataFlag = 0;
				SendOK();
			}
			else
			{
				LED4_ON;
				printf("\r\n%x  %x\r\n",CheckCRC,CRC16((uint8_t*)APP_DEFAULT_ADD,CodeLength+1,0));
				FlashDataFlag = 0;
				SendError();
			}
		}
	}
		else if(SensorGain_mk210==1)
	{
		cnnt++;
		for(num=0;num<512;num++)
		{
			Sdata[5+num+(cnnt-1)*512]=USB_Rx_Buffer[num];
		}
		if(cnnt==0x08)
		{
//			temp=0x01+0x05+0x04+USB_Rx_Buffer[2];
//			CS0_Low;
//			SPI1_ReadWriteByte(0xef);
//			SPI1_ReadWriteByte(0x01);
//			SPI1_ReadWriteByte(0x01);
//			SPI1_ReadWriteByte(0x00);
//			SPI1_ReadWriteByte(0x05);
//			SPI1_ReadWriteByte(0x04);
//			SPI1_ReadWriteByte(0x00);
//			SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
//			SPI1_ReadWriteByte(0x00);
//			SPI1_ReadWriteByte(temp);
//			CS0_High;
			 
//			TXDMA_Config(DMA2_Stream5,DMA_Channel_3,(uint32_t)Sdata,4103);
			//delay_ms(100);
			 
			if(!ReadyForCMD())
			{
				SendErrorCode(0,0x0000);//data error
				SensorGain_mk210=0;
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(50);
				return;
			}
				
//						if(USB_Rx_Buffer[2]==55)
//						{
//							Sdata[2]=0x0f;
//						}
			//	dma_send//  dma_send()
			CS0_Low;
			for(num=0;num<4103;num++)
			{
				SPI1_ReadWriteByte(Sdata[num]);
			}
			
			//MYDMA_Enable(DMA2_Stream2,5127,(uint32_t)Sdata);
//			SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);	
//			DMA_Cmd(DMA2_Stream3, DISABLE);                      //关闭DMA传输 Tx
//			while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	//确保DMA可以被设置  
//			DMA_SetCurrDataCounter(DMA2_Stream3,4103);          //数据传输量  
//			DMA_Cmd(DMA2_Stream3, ENABLE);                      //开启DMA传输 
//			
//			while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//			//while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//			//DMA_Cmd(DMA2_Stream2, DISABLE);
//			DMA_Cmd(DMA2_Stream3, DISABLE);	
//			//DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//			DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
	
			CS0_High;
			
			if(Sdata[2] == 0x0f)
			{
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);//data error
					SensorGain_mk210=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					return;
				}
				if(USB_Rx_Buffer[2]==0x02)
				{
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(50);
				}
			}
			cnnt=0;
			SensorGain_mk210=0;
		}
		SendOK();
	}

	else if(SensorGain_mk510==1)
	{
		cnnt++;
		for(num=0;num<512;num++)
		{
			Sdata[num+(cnnt-1)*512]=USB_Rx_Buffer[num];//from sdata[0] store data
		}
		if(cnnt==0x08)
		{	
			if(mk510_which_gain==1)
			{
				W25QXX_Write(Sdata,mk510_first_gain_addr,4096);		
				mk510_first_gain_addr  += 4096;
			}
			else if(mk510_which_gain==2) 
			{
				W25QXX_Write(Sdata,mk510_second_gain_addr,4096);		
				mk510_second_gain_addr += 4096;
			}

			if(mk510_last_pack==1)//last 4
			{
				mk510_first_gain_addr 	= 0x0;  //clear addr 
				mk510_second_gain_addr	= 0x180000;
			}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;	
			
				cnnt=0;
				SensorGain_mk510=0;
		}
		SendOK();
	}		
	else if(SensorGain_mk310==1)
	{
		cnnt++;
		for(num=0;num<512;num++)
		{
			Sdata[5+num+(cnnt-1)*512]=USB_Rx_Buffer[num];
		}
		if(cnnt==0x08)
		{			 
			if(!ReadyForCMD())
			{
				SendErrorCode(0,0x0000);//data error
				SensorGain_mk310=0;
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(50);
				return;
			}

			CS0_Low;
			for(num=0;num<4103;num++)
			{
					SPI1_ReadWriteByte(Sdata[num]);
			}					
			CS0_High;
			
			if(Sdata[2] == 0x0f)
			{
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);//data error
					SensorGain_mk310=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					return;
				}
				if(USB_Rx_Buffer[2]==0x02)
				{
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
				}
			}
			cnnt=0;
			SensorGain_mk310=0;
		}
		SendOK();
	}

	else if(SensorGain_mk610==1)
	{
		cnnt++;
		for(num=0;num<512;num++)
		{
			Sdata[5+num+(cnnt-1)*512]=USB_Rx_Buffer[num];
			IWDG_ReloadCounter();
		}
		if(cnnt==0x08)
		{		 
			if(!ReadyForCMD())
			{
				SendErrorCode(0,0x0000);//data error
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				SensorGain_mk610=0;
				delay_ms(50);
				return;
			}
				
//						if(USB_Rx_Buffer[2]==55)
//						{
//							Sdata[2]=0x0f;
//						}
			//	dma_send//  dma_send()
			CS0_Low;
			for(num=0;num<4103;num++)
			{
				SPI1_ReadWriteByte(Sdata[num]);
			}
			
			//MYDMA_Enable(DMA2_Stream2,5127,(uint32_t)Sdata);
//			SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);	
//			DMA_Cmd(DMA2_Stream3, DISABLE);                      //关闭DMA传输 Tx
//			while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	//确保DMA可以被设置  
//			DMA_SetCurrDataCounter(DMA2_Stream3,4103);          //数据传输量  
//			DMA_Cmd(DMA2_Stream3, ENABLE);                      //开启DMA传输 
//			
//			while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//			//while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//			//DMA_Cmd(DMA2_Stream2, DISABLE);
//			DMA_Cmd(DMA2_Stream3, DISABLE);	
//			//DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//			DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
			
			CS0_High;
			
			if(Sdata[2] == 0x0f)
			{
				if(!ReadyForCMD()) 
				{
					SendErrorCode(0,0x0000);//data error
					SensorGain_mk610=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					return;
				} 
//				if(USB_Rx_Buffer[2]==0x02)
//				{
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(50);
//				}
			}
			cnnt=0;
			SensorGain_mk610=0;
		}
		
		SendOK();
	}	
	
	else if(SensorGain_mk710==1)
	{
		cnnt++;
		for(num=0;num<512;num++)
		{
			Sdata[5+num+(cnnt-1)*512]=USB_Rx_Buffer[num];
			IWDG_ReloadCounter();
		}
		if(cnnt==0x08)
		{		 
			if(!ReadyForCMD())
			{
				SendErrorCode(0,0x0000);//data error
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				SensorGain_mk710=0;
				delay_ms(50);
				return;
			}
				
//						if(USB_Rx_Buffer[2]==55)
//						{
//							Sdata[2]=0x0f;
//						}
			//	dma_send//  dma_send()
			CS0_Low;
			for(num=0;num<4103;num++)
			{
				SPI1_ReadWriteByte(Sdata[num]);
			}
			
			//MYDMA_Enable(DMA2_Stream2,5127,(uint32_t)Sdata);
//			SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);	
//			DMA_Cmd(DMA2_Stream3, DISABLE);                      //关闭DMA传输 Tx
//			while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	//确保DMA可以被设置  
//			DMA_SetCurrDataCounter(DMA2_Stream3,4103);          //数据传输量  
//			DMA_Cmd(DMA2_Stream3, ENABLE);                      //开启DMA传输 
//			
//			while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//			//while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//			//DMA_Cmd(DMA2_Stream2, DISABLE);
//			DMA_Cmd(DMA2_Stream3, DISABLE);	
//			//DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//			DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
			
			CS0_High;
			
			if(Sdata[2] == 0x0f)
			{
				if(!ReadyForCMD()) 
				{
					SendErrorCode(0,0x0000);//data error
					SensorGain_mk710=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					return;
				} 
//				if(USB_Rx_Buffer[2]==0x02)
//				{
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(50);
//				}
			}
			cnnt=0;
			SensorGain_mk710=0;
		}
	else if(SensorGain_mk710_B==1)
	{
		cnnt++;
		for(num=0;num<512;num++)
		{
			Sdata[5+num+(cnnt-1)*512]=USB_Rx_Buffer[num];
			IWDG_ReloadCounter();
		}
		if(cnnt==0x08)
		{		 
			if(!ReadyForCMD())
			{
				SendErrorCode(0,0x0000);//data error
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				SensorGain_mk710_B=0;
				delay_ms(50);
				return;
			}
				
//						if(USB_Rx_Buffer[2]==55)
//						{
//							Sdata[2]=0x0f;
//						}
			//	dma_send//  dma_send()
			CS0_Low;
			for(num=0;num<4103;num++)
			{
				SPI1_ReadWriteByte(Sdata[num]);
			}
			
			//MYDMA_Enable(DMA2_Stream2,5127,(uint32_t)Sdata);
//			SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);	
//			DMA_Cmd(DMA2_Stream3, DISABLE);                      //关闭DMA传输 Tx
//			while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	//确保DMA可以被设置  
//			DMA_SetCurrDataCounter(DMA2_Stream3,4103);          //数据传输量  
//			DMA_Cmd(DMA2_Stream3, ENABLE);                      //开启DMA传输 
//			
//			while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//			//while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//			//DMA_Cmd(DMA2_Stream2, DISABLE);
//			DMA_Cmd(DMA2_Stream3, DISABLE);	
//			//DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//			DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
			
			CS0_High;
			
			if(Sdata[2] == 0x0f)
			{
				if(!ReadyForCMD()) 
				{
					SendErrorCode(0,0x0000);//data error
					SensorGain_mk710_B=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					return;
				} 
//				if(USB_Rx_Buffer[2]==0x02)
//				{
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(50);
//				}
			}
			cnnt=0;
			SensorGain_mk710_B=0;
		}
		
		SendOK();
	}		
		
		SendOK();
	}		
	
	else if(SensorGain==1)
	{
		cnnt++;
		for(num=0;num<512;num++)
		{
			Sdata[5+num+(cnnt-1)*512]=USB_Rx_Buffer[num];
		}
		if(cnnt==0x08)
		{		 
			if(!ReadyForCMD())
			{
				SendErrorCode(0,0x0000);//data error
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				SensorGain=0;
				delay_ms(50);
				return;
			}
				
//						if(USB_Rx_Buffer[2]==55)
//						{
//							Sdata[2]=0x0f;
//						}
			//	dma_send//  dma_send()
			CS0_Low;
			for(num=0;num<4103;num++)
			{
				SPI1_ReadWriteByte(Sdata[num]);
			}
			
			//MYDMA_Enable(DMA2_Stream2,5127,(uint32_t)Sdata);
//			SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);	
//			DMA_Cmd(DMA2_Stream3, DISABLE);                      //关闭DMA传输 Tx
//			while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	//确保DMA可以被设置  
//			DMA_SetCurrDataCounter(DMA2_Stream3,4103);          //数据传输量  
//			DMA_Cmd(DMA2_Stream3, ENABLE);                      //开启DMA传输 
//			
//			while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
//			//while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
//			//DMA_Cmd(DMA2_Stream2, DISABLE);
//			DMA_Cmd(DMA2_Stream3, DISABLE);	
//			//DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
//			DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
			
			CS0_High;
			
			if(Sdata[2] == 0x0f)
			{
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);//data error
					SensorGain=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					return;
				}
				if(USB_Rx_Buffer[2]==0x02)
				{
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(50);
				}
			}
			cnnt=0;
			SensorGain=0;
		}
		SendOK();
	}
	else if(Sensor_Update==1)
	{
		code_cnt++;
		for(num=0;num<512;num++)
		{
			Sdata[5+num+(code_cnt-1)*512]=USB_Rx_Buffer[num];  //拼装4kdata
		}
		if(code_cnt==0x08)
		{
			if(!ReadyForCMD())
			{
				IWDG_ReloadCounter();
				delay_ms(2000);
				IWDG_ReloadCounter();
				SendErrorCode(0,0x0000);//data error
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				Sensor_Update=0;
				delay_ms(100);
				return;
			}
		    CS0_Low;
			for(num=0;num<4103;num++)
			{
				SPI1_ReadWriteByte(Sdata[num]);
			}
			CS0_High;
			
			if(Sdata[2] == 0x0f)
			{
				if(!ReadyForCMD())
				{
					IWDG_ReloadCounter();
					delay_ms(2000);
					IWDG_ReloadCounter();
					SendErrorCode(0,0x0000);//data error
//					delay_ms(5);
					code_cnt=0;
					Sensor_Update=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					return;
				}
				delay_ms(5);
				if(!ReadyForCMD())
				{
					IWDG_ReloadCounter();
					delay_ms(2000);		
					IWDG_ReloadCounter();		
					SendErrorCode(0,0x0000);//data error
//					delay_ms(5);
					code_cnt=0;
					Sensor_Update=0;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					return;
				}
					IWDG_ReloadCounter();
					delay_ms(3000);//40ms
					IWDG_ReloadCounter();
			}
			code_cnt=0;
			Sensor_Update=0; 
		}
//		delay_ms(5);
		SendOK();
			
//		if(Sdata[2] == 0x0f&&code_cnt==0)
//		{	
//		delay_ms(4000);
//		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//		PWROFF_1117;

//		}
//		if(Sdata[2]==0x0f&&code_cnt==0)
//		{
//			if(!ReadyForCMD())
//				{
//					SendErrorCode(0,0x0000);//data error
//					return;
//				}
//			else 
//				SendOK();
//		}
//		else SendOK();

	}
	else if(auto_burn_cnt>0)
	{
			auto_burn_cnt++;
			if(auto_burn_cnt==3)
			{
				auto_burn_cnt=0;
				
			}
	}
	else if((DFUDataFlag == 0)&&(DataFlag == 0)&&(GainDataFlag == 0)&&(FlashDataFlag == 0)&&(SensorGain==0)&&(Sensor_Update==0)&&(auto_burn_cnt==0)&&(SensorGain_mk210==0)&&(SensorGain_mk310==0)&&(SensorGain_mk610==0)&&(SensorGain_mk710==0)&&(SensorGain_mk710_B==0))
	{
		switch(CMD->TwoByte)
		{
			case 0x5500:
				
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{Send_FWID();
				}
				else{SendError();}
				break;
				
			case 0x5501:
				if((USB_Rx_Buffer[10] == 0xff) && (USB_Rx_Buffer[11] == 0xff))
				{
					//EndByteEN = USB_Rx_Buffer[2];
					CodeLength = (USB_Rx_Buffer[2]<<24)|(USB_Rx_Buffer[3]<<16)|(USB_Rx_Buffer[4]<<8)|(USB_Rx_Buffer[5]);
					CheckCRC = (USB_Rx_Buffer[8]<<8)|(USB_Rx_Buffer[9]);
					STMFlash_EraseSect(APP_DEFAULT_ADD, CodeLength+1);
					printf("DFU data: CodeLength=%x  CheckCRC=%x\r\n",CodeLength,CheckCRC);
					SendOK();
					DFUDataFlag = 1;	
					//CMD->TwoByte = 0;
				}
				else{SendError();}
				break;
						
			case 0xaa55:
				if((USB_Rx_Buffer[10] == 0xff) && (USB_Rx_Buffer[11] == 0xff))
				{
					//EndByteEN = USB_Rx_Buffer[2];
					CodeLength = (USB_Rx_Buffer[2]<<24)|(USB_Rx_Buffer[3]<<16)|(USB_Rx_Buffer[4]<<8)|(USB_Rx_Buffer[5]);
					CheckCRC = (USB_Rx_Buffer[8]<<8)|(USB_Rx_Buffer[9]); 
					STMFlash_EraseSect(APP_DEFAULT_ADD, CodeLength+1);//free space for code
					printf("Code data: CodeLength=%x  CheckCRC=%x\r\n",CodeLength,CheckCRC);
					SendOK();
					DataFlag = 1;	
					//CMD->TwoByte = 0;
				}
	//			if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
	//			{
	//				STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)TX_Data,1);
	//				delay_ms(10);
	//				TIM_Cmd(TIM3,DISABLE);
	//				SendOK();
	//				while(1);
	//				GPIO_SetBits(GPIOC,GPIO_Pin_9);
	//				delay_ms(10);
	//			}
				else{SendError();}
				break;
				
			case 0x5502:			

				if((USB_Rx_Buffer[8] == 0xff) && (USB_Rx_Buffer[9] == 0xff))
				{
//					VMOD1=(USB_Rx_Buffer[2]<<8)|USB_Rx_Buffer[3];

//					VMOD2=(USB_Rx_Buffer[4]<<8)|USB_Rx_Buffer[5];

//					BUF=(USB_Rx_Buffer[6]<<8)|USB_Rx_Buffer[7];
//				
//					Dac1_Set_Vol(VMOD1);	//MOD1
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VMOD2,1);//MOD2 off
//					//delay_ms(1);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//BUF off
//					delay_ms(50);
//					//delay_ms(1);
//					PWRON_1117;
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);
//					delay_ms(75);//等待电源稳定
//					if((abs((int)(VMOD1 - (Voltage_MOD1_DET()*1.25)))<120)/*&&(abs((int)(VMOD2 - (Voltage_MOD2_DET()*1.25)))<100)*/)//1.25为datasheet规定相乘的系数
//					{
//						if(((STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+4)&0x0000ffff)==VMOD1)&&
//							((STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+8)&0x0000ffff)==VMOD2)&&
//							((STMFLASH_ReadWord(PARAMETER_SAVE_ADDR+12)&0x0000ffff)==BUF))
//						{}
//						else
//						{
//							STMFLASH_ReadforByte(FLASH_SAVE_ADDR,Sdata,512);
//							Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+4]  = USB_Rx_Buffer[3];
//							Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+5]  = USB_Rx_Buffer[2];
//							Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+8]  = USB_Rx_Buffer[5];
//							Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+9]  = USB_Rx_Buffer[4];
//							Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+12] = USB_Rx_Buffer[7];
//							Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+13] = USB_Rx_Buffer[6];
//							STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t *)Sdata,512);
//						}						
//						//printf(" %d 0x%x 0x%x\r\n",VMOD1,VMOD2,BUF);
//						SendOK();
//					}//电压设置在误差范围内OK,否则Error.
//					else{SendErrorCode(0,0x0a);}
//					
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//BUF off
//					PWROFF_1117;//电压测试完成，下电
//					delay_ms(100);
					MOD1_MOD2_Buffer_Set(USB_Rx_Buffer);
					/*Need add sth*/
					
				}
				else{SendError();}
				break;
				
			case 0x5503:
				//printf(" %d \r\n",Image_Size);
				if((USB_Rx_Buffer[6] == 0xff) && (USB_Rx_Buffer[7] == 0xff))
				{
					Image_Size = ((USB_Rx_Buffer[2]<<8)|USB_Rx_Buffer[3])*((USB_Rx_Buffer[4]<<8)|USB_Rx_Buffer[5])<<1;//<<1 -> *2
					//printf(" %d \r\n",Image_Size);
					if((STMFLASH_ReadWord(PARAMETER_SAVE_ADDR)==Image_Size))
					{}
					else
					{
						STMFLASH_ReadforByte(FLASH_SAVE_ADDR,Sdata,512);
						Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR] = (uint8_t)Image_Size;
						Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+1] = (uint8_t)(Image_Size>>8);
						Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+2] = (uint8_t)(Image_Size>>16);
						Sdata[PARAMETER_SAVE_ADDR-FLASH_SAVE_ADDR+3] = (uint8_t)(Image_Size>>24);
						STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t *)Sdata,512);
//						STMFLASH_Write(PARAMETER_SAVE_ADDR,&Image_Size,4);
//						STMFLASH_Write(PARAMETER_SAVE_ADDR+4,(uint32_t*)&VMOD1,4);
//						STMFLASH_Write(PARAMETER_SAVE_ADDR+8,(uint32_t*)&VMOD2,4);
//						STMFLASH_Write(PARAMETER_SAVE_ADDR+12,(uint32_ t*)&BUF,4);
//						STMFLASH_Write(PARAMETER_SAVE_ADDR+16,(uint32_t*)RegParameter,200);
					}
					//STMFLASH_Write(FLASH_SAVE_ADDR+12,&Image_Size,4);
					//printf(" %d \r\n",Image_Size);
	//				Size_Reg[0] = USB_Rx_Buffer[5];
	//				Size_Reg[1] = USB_Rx_Buffer[3];
	//			#ifdef	DEBUG
	//				printf(" %d %d\r\n",Size_Reg[0],Size_Reg[1]);
	//			#endif
					SendOK();
					//printf(" %d \r\n",Image_Size);
				}
				else{SendError();}
				//printf(" %d \r\n",Image_Size);
				break;
				
			case 0x5504:
				
				for(num=2;num<250;num=num+3)
				{
					RegParameter[num-2] = USB_Rx_Buffer[num];
					RegParameter[num-1] = USB_Rx_Buffer[num+1];
					RegParameter[num] = USB_Rx_Buffer[num+2];
					
					/*if(USB_Rx_Buffer[num] == 0x94)
					{
						Dtime = ((USB_Rx_Buffer[num+4]<<8)|USB_Rx_Buffer[num+1])>>3;
						printf(" %x \r\n",Dtime);
					}
					else */if(USB_Rx_Buffer[num] == 0x9c)
					{
						Reg9d1 = USB_Rx_Buffer[num+4];
						Reg9e1 = USB_Rx_Buffer[num+7];
					}
					else if(USB_Rx_Buffer[num] == 0xf1)
					{
						WakeUpReg = USB_Rx_Buffer[num+1];
						Reg9d2 = USB_Rx_Buffer[num+4];
						Reg9e2 = USB_Rx_Buffer[num+7];
					}
					else if((USB_Rx_Buffer[num] == 0xff)&&(USB_Rx_Buffer[num+1] == 0xff))
					{break;}
					
				}
//				for(num=0;num<=197;num++)
//				{
//					if(RegParameter[num] != STMFLASH_ReadByte(PARAMETER_SAVE_ADDR+16+num))
//					{
//						//LED3_ON;
////						STMFLASH_Write(PARAMETER_SAVE_ADDR,&Image_Size,4);
////						STMFLASH_Write(PARAMETER_SAVE_ADDR+4,(uint32_t*)&VMOD1,4);
////						STMFLASH_Write(PARAMETER_SAVE_ADDR+8,(uint32_t*)&VMOD2,4);
////						STMFLASH_Write(PARAMETER_SAVE_ADDR+12,(uint32_t*)&BUF,4);
//						STMFLASH_ReadforByte(FLASH_SAVE_ADDR,Sdata,172);
//						STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t *)Sdata,172);
//						STMFLASH_Write(PARAMETER_SAVE_ADDR+16,(uint32_t*)RegParameter,200);
//						break;
//					}
//				}
//				printf("Reg9d1=%x Reg9e1=%x\r\n",Reg9d1,Reg9e1);
//				printf("Reg9d2=%x Reg9e2=%x WakeUpReg=%x\r\n",Reg9d2,Reg9e2,WakeUpReg);
//				for(num=0;num<200;num++)
//				{printf("0x%x ",RegParameter[num]);}
		
				
				SendOK();
				break;
				
			case 0x5505:
						
				for(num = 2; num < 200 ; num=num+3)	//write reg to PMU
				{
					if((USB_Rx_Buffer[num] == 0xff)&&(USB_Rx_Buffer[num+1] == 0xff))
					{break;}
					else
					{
						Write_PMU(USB_Rx_Buffer[num],USB_Rx_Buffer[num+1]);
						if(USB_Rx_Buffer[num+2]==0){}
						else{delay_ms(USB_Rx_Buffer[num+2]);}
					}
				}
			
				SendOK();
				break;
				
			case 0x5506:
				if((USB_Rx_Buffer[3] == 0xff) && (USB_Rx_Buffer[4] == 0xff))
				{
//					PWRON_1117;
//					delay_ms(5);//等待电源稳定
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(100);//等待电源稳定
					if(Touch_Scan(USB_Rx_Buffer[2]))
					{
						SendOK();
					}
					else
					{
						SendError();
					}
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
				}
				else
				{SendError();}
				
				break;
			
			case 0x5507:
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{
					PWRON_1117;
					delay_ms(15);//等待电源稳定
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
					temp = Voltage_MOD1_DET();
					Sdata[0] = 0xf0;
					Sdata[1] = 0x00;
					Sdata[2] = (uint8_t)(temp>>8);
					Sdata[3] = (uint8_t)(temp);
					Sdata[4] = 0xff;
					Sdata[5] = 0xff;
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
					printf("VMOD1 = %f mV\r\n",(temp*1.25));
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
				}
				else
				{SendError();}
				break;
			
			case 0x5508:	//SPI速度调整
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					switch(USB_Rx_Buffer[3])
					{
						case 1:
							SPI1_SetSpeed(SPI_BaudRatePrescaler_4);
							SPI1Speed = SPI_BaudRatePrescaler_4;
							break;
						case 2:
							SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
							SPI1Speed = SPI_BaudRatePrescaler_8;
							break;
						case 3:
							SPI1_SetSpeed(SPI_BaudRatePrescaler_16);
							SPI1Speed = SPI_BaudRatePrescaler_16;
							break;
						case 4:
							SPI1_SetSpeed(SPI_BaudRatePrescaler_32);
							SPI1Speed = SPI_BaudRatePrescaler_32;
							break;
						case 5:
							SPI1_SetSpeed(SPI_BaudRatePrescaler_64);
							SPI1Speed = SPI_BaudRatePrescaler_64;
							break;
						case 6:
							SPI1_SetSpeed(SPI_BaudRatePrescaler_128);
							SPI1Speed = SPI_BaudRatePrescaler_128;
							break;
						case 7:
							SPI1_SetSpeed(SPI_BaudRatePrescaler_256);
							SPI1Speed = SPI_BaudRatePrescaler_256;
							break;
						default :
							SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
							SPI1Speed = SPI_BaudRatePrescaler_8;
							break;
					}
					SendOK();
				}
				else
				{SendError();}
				break;
				
			case 0x5509:
				if((USB_Rx_Buffer[3] == 0xff) && (USB_Rx_Buffer[4] == 0xff))
				{
					if(Flash_Judgment()){SendOK();}
					else{SendError();}
				}
				else
				{SendError();}
				break;
			//0x5510已被采图相关功能使用
			case 0x5521: //mk110
				if((USB_Rx_Buffer[3] == 0xff) && (USB_Rx_Buffer[4] == 0xff))
				{
					switch(USB_Rx_Buffer[2])
					{
						case 0: //正常工作电流
							PWRON_1117;//enable
							//delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							delay_ms(600);
							//Current_MOD1_DET();
							temp = Current_MOD1_DET();

							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
//							while(USB_StatusDataSended==0);
//							USB_StatusDataSended = 0;
//							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							printf("VMOD1 enable current = %d uA\r\n",(temp*10));
//							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//							PWROFF_1117;	
							break;
						
						case 1://静态工作电流
							PWRON_1117;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VMOD2,1);//MOD2 off disable
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							G0361_POWEROFF;
							G0360_POWEROFF;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							//GPIO_SetBits(GPIOB,GPIO_Pin_14);//1117_ON
							delay_ms(100);//等待电源稳定
							//Current_MOD1_DET();
							temp = Current_MOD1_DET();

							if(temp > 32767)
							{
								temp = 0;
							}
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
//							while(USB_StatusDataSended==0);
//							USB_StatusDataSended = 0;
//							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							printf("VMOD1 disable current = %d uA\r\n",(temp*10));
							//GPIO_ResetBits(GPIOB,GPIO_Pin_14);
							//IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//							PWROFF_1117;
							break;
					}
				}
				else
				{SendError();}
				break;
				
			case 0x5511:
				if((USB_Rx_Buffer[3] == 0xff) && (USB_Rx_Buffer[4] == 0xff))
				{
					switch(USB_Rx_Buffer[2])
					{
						case 0:
							PWRON_1117;//enable
							//delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							delay_ms(100);
							//Current_MOD1_DET();
							temp = Current_MOD1_DET();
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
							while(USB_StatusDataSended==0);
							USB_StatusDataSended = 0;
							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							printf("VMOD1 enable current = %d uA\r\n",(temp*5));
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;	
							break;
						
						case 1:
							PWRON_1117;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VMOD2,1);//MOD2 off disable
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							G0361_POWEROFF;
							G0360_POWEROFF;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							//GPIO_SetBits(GPIOB,GPIO_Pin_14);//1117_ON
							delay_ms(100);//等待电源稳定
							//Current_MOD1_DET();
							temp = Current_MOD1_DET();
							if(temp > 32767)
							{
								temp = 0;
							}
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
							while(USB_StatusDataSended==0);
							USB_StatusDataSended = 0;
							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							printf("VMOD1 disable current = %d uA\r\n",(temp*5));
							//GPIO_ResetBits(GPIOB,GPIO_Pin_14);
							//IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							break;
						
						case 2:
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToPMU())
							{
								Write_PMU(0xf6,0xff);
								delay_ms(50);//40ms左右电压会稳定
								temp = Current_MOD1_DET();
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								printf("VMOD1 wakeup current = %d uA\r\n",(temp*5));
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							break;
							
						case 3:
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToPMU())
							{
								Write_PMU(0xf6,0xff);
								delay_ms(2);
								Write_PMU(0xf8,0xff);//restart
								num=500;
								while((--num)&&(Read_PMU(0x44)) != 0x8002){delay_ms(1);}
								if(num == 0)
								{
									SendErrorCode(3,Read_PMU(0x44));
									//printf(" %x \r\n",Read_PMU(0x01));
									//printf(" %x %x %x %x \r\n",Read_PMU(0x46),Read_PMU(0x47),Read_PMU(0x48),Read_PMU(0x49));
									//Write_PMU(0xf8,0xff);//restart
									Write_PMU(0xf2,0xff);//sleep
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
									PWROFF_1117;
									delay_ms(100);
									USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
									return ;
								}
								Write_PMU(0xf2,0xff);//sleep
								delay_ms(25);
								temp = Current_MOD1_DET();
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								printf("VMOD1 sleep current = %d uA\r\n",(temp*5));
								
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							break;
							
						case 4:
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToPMU())
							{
								Write_PMU(0xf6,0xff);
								delay_ms(2);
								
								Write_PMU(0xf3,0x40);
								//delay_ms(20);//delay时间需要实际测量，要等待LED亮起的时候才测量电流
								
								num=500;
								while((--num)&&(Read_PMU(0x44) != 0x0059))//0x0049亮灯，马上采样会不准确，0x0059较为准确
								{
									delay_ms(1);
								}
								temp = Current_MOD1_DET();
								while((--num)&&(Read_PMU(0x44) != 0x4069))
								{
									delay_ms(1);
								}
								if(num==0)
								{
									SendErrorCode(2,Read_PMU(0x44));
									Write_PMU(0xf8,0xff);//restart
									Write_PMU(0xf2,0xff);//sleep
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
									PWROFF_1117;
									delay_ms(100);
									return ;
								}
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								printf("VMOD1 F340 current = %d uA\r\n",(temp*5));
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							break;
							default :
   						break;
					}
				}
				else
				{SendError();}
				break;
			
			case 0x552c://burn_device
				
				IWDG_Init(6,1875);		//10s watchdog//2500// 6,1875
				Burn_on; //burn device open 
				delay_ms(600);
			
				PWRON_1117;//module open
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);
				delay_ms(8000);
			    
				Burn_off; 
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(1000);
				PWRON_1117;
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);
				delay_ms(15);
			
				HANDSHAKE();
				CS0_Low;	
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x03);
			
				SPI1_ReadWriteByte(0x07);
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x0b);
				CS0_High;
				
				delay_ms(10);
					
				if(!ReadyFor_moduleCMD())
				{
				
					SendError();
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(5);

				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(5);	
			   IWDG_Init(5,2000);		//10s watchdog//2500// 6,1875
			break;
				
			case 0x5512:
				UserNamePassWordSave();
				break;
			case 0x5513:
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{SendUserNamePassWord();}
				else{SendError();}
				break;
				
			case 0x5514:
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					if(USB_Rx_Buffer[3]==0xff)
					{
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x40,0,1);//关闭4个LED，电流设为最低。0x4F为开启4个LED
					}
					else
					{
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x4f,USB_Rx_Buffer[3],1);
					}
					SendOK();
				}
				else{SendError();}
				break;
				
			case 0x5515:
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					if(USB_Rx_Buffer[3]==0x00)
					{
						G0361_RLED_OFF;
					}
					else
					{
						G0361_RLED_ON;
					}
					SendOK();
				}
				else{SendError();}
				break;
			
			case 0x5516:
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					if(USB_Rx_Buffer[3]==0x00)
					{
						G0361_BLED_OFF;
					}
					else
					{
						G0361_BLED_ON;
					}
					SendOK();
				}
				else{SendError();}
				break;
				
			case 0x5517:
				if((USB_Rx_Buffer[3] == 0xff) && (USB_Rx_Buffer[4] == 0xff))
				{
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,VMOD2,1);//MOD2 on
					//GPIO_SetBits(GPIOB,GPIO_Pin_14);//1117_ON
					delay_ms(100);//等待电源稳定
					//Current_MOD1_DET();
					temp = Current_MOD2_DET();
					if(temp > 32767)
					{
						temp = 0;
					}
					Sdata[0] = 0xf0;
					Sdata[1] = 0x00;
					Sdata[2] = (uint8_t)(temp>>8);
					Sdata[3] = (uint8_t)(temp);
					Sdata[4] = 0xff;
					Sdata[5] = 0xff;
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
					printf("G0360 disable current = %d uA\r\n",(temp*5));
					
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VMOD2,1);//MOD2 on
					//GPIO_ResetBits(GPIOB,GPIO_Pin_14);
					//IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					//PWROFF_1117;
				}
				else
				{SendError();}
				break;
				
			//0x5518已被采图相关功能使用
			case 0x5519:
				if((USB_Rx_Buffer[3] == 0xff) && (USB_Rx_Buffer[4] == 0xff))
				{
					switch(USB_Rx_Buffer[2])
					{
						case 0:
							PWRON_1117;//enable
							//delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							delay_ms(100);
							//Current_MOD1_DET();
							temp = Current_MOD1_DET();
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
							while(USB_StatusDataSended==0);
							USB_StatusDataSended = 0;
							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							printf("CMOS enable current = %d uA\r\n",(temp*5));
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;	
							break;
						
						case 1:
							PWRON_1117;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,VMOD2,1);//MOD2 off disable
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							G0361_POWEROFF;
							G0360_POWEROFF;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							//GPIO_SetBits(GPIOB,GPIO_Pin_14);//1117_ON
							delay_ms(100);//等待电源稳定
							//Current_MOD1_DET();
							temp = Current_MOD1_DET();
							if(temp > 32767)
							{
								temp = 0;
							}
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
							while(USB_StatusDataSended==0);
							USB_StatusDataSended = 0;
							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							printf("CMOS disable current = %d uA\r\n",(temp*5));
						
							PWROFF_1117;
							break;
						
						case 2:
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToCMOS())
							{
								delay_ms(50);//40ms左右电压会稳定
								temp = Current_MOD1_DET();
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								printf("CMOS wakeup current = %d uA\r\n",(temp*5));
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							break;
							
						case 3:
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToCMOS())
							{
								Write_PMU(0xf8,0xff);//restart
								num=500;
								while((--num)&&((Read_PMU(0x3f)&0x0400) != 0x0400)){delay_ms(1);}
								if(num == 0)
								{
									SendErrorCode(3,Read_PMU(0x3f));
									Write_PMU(0xcf,0x27);
									Write_PMU(0xf2,0xff);//sleep
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
									PWROFF_1117;
									delay_ms(100);
									USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
									return ;
								}
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								SPI1_ReadWriteByte(0x00);
								//CS0_Low;
								RST_PMU();
								CS0_High;
								CS_FLASH_High;
								delay_ms(200);
								Current_MOD1_DET();
								temp = Current_MOD1_DET()+3;
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								CS0_High;
								printf("CMOS sleep current = %d uA\r\n",(temp*5));
								
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							break;
							
						case 4:
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToCMOS())
							{	
								Write_PMU(0xf3,0xff);
								delay_ms(20);
								
								temp = Current_MOD1_DET();
								while((--num)&&(Read_PMU(0x3f) != 0x8B0B))
								{
									delay_ms(1);
								}
								if(num==0)
								{
									SendErrorCode(2,Read_PMU(0x3f));
									Write_PMU(0xf8,0xff);//restart
									Write_PMU(0xcf,0x27);
									Write_PMU(0xf2,0xff);//sleep
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
									PWROFF_1117;
									delay_ms(100);
									return ;
								}
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								printf("CMOS F3FF current = %d uA\r\n",(temp*5));
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							break;
							
						default :
							
							break;
					}
				}
				else
				{SendError();}
				break;
				
			case 0x5429://mk110 mk210 mk310 mk410 mk510
				if((USB_Rx_Buffer[3] == 0xff) && (USB_Rx_Buffer[4] == 0xff))
				{
					IWDG_Init(6,1875);		//10s watchdog//2500// 6,1875
					switch(USB_Rx_Buffer[2])
					{
						case 0: //pwr_on current
							PWRON_1117;//enable
							//delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							delay_ms(6000);
							//Current_MOD1_DET();
							if(Current_MOD1_DET()<=0)
								temp=0;
							else
								temp = Current_MOD1_DET();//capture current
							//if()
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
							while(USB_StatusDataSended==0);
							USB_StatusDataSended = 0;
							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							printf("mk enable current = %d uA\r\n",(temp*10));
							
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;	
							break;
						
						case 1://capture current
							PWRON_1117;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
							//GPIO_SetBits(GPIOB,GPIO_Pin_14);//1117_ON
							delay_ms(60);//等待电源稳定
							Write_PMU(0xef,0x01);
							Write_PMU(0x01,0x00);
							Write_PMU(0x05,0x02);
							Write_PMU(0x00,0x01);//1
						   Write_PMU(0x00,0x09);//9
						
							if(!ReadyForCMD())
							{
								SendErrorCode(0,0x0000);
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
								delay_ms(100);
			//					PWRON_1117;
			//					delay_ms(2);
			//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
			//					delay_ms(200);
								break;
							}						
						
							if(Current_MOD1_DET()<=0)
								temp=0;
							else
								temp = Current_MOD1_DET();
							if(temp > 32767)
							{
								temp = 0;
							}
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
							
							while(USB_StatusDataSended==0);
							USB_StatusDataSended = 0;
							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							
							printf("mk capture current = %d uA\r\n",(temp*10));
						
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer							
							PWROFF_1117;
							break;
						
						case 2://sleep current
							PWRON_1117;
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//							delay_ms(6000);//等待电源稳定						
/************************************进入低功耗****************************************/
							temp=0x01+0x04+0xf1+0x01;
							Write_PMU(0xef,0x01);
							Write_PMU(0x01,0x00);
							Write_PMU(0x04,0xf1);
							CS0_Low;				
							SPI1_ReadWriteByte(0x01);
							SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
							SPI1_ReadWriteByte((uint8_t)temp);//sum2
							CS0_High;
						
						if(!ReadyForCMD())
							{
								SendErrorCode(0,0x0000);
								printf("go into low resume error!");
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
								delay_ms(100);
			//					PWRON_1117;
			//					delay_ms(2);
			//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
			//					delay_ms(200);
								break;
							}
//						else 
//							{
//								delay_ms(100);
//								if(Current_MOD1_DET()<=0)
//								printf("VMOD1 disable current = %d uA\r\n",0);
//								else
//								printf("VMOD1 disable current = %d uA\r\n",Current_MOD1_DET()*10);
//							}
/**************************************************************************************/
						
							if(Current_MOD1_DET()<=0)
								temp=0;
							else
								temp = Current_MOD1_DET();
							Sdata[0] = 0xf0;
							Sdata[1] = 0x00;
							Sdata[2] = (uint8_t)(temp>>8);
							Sdata[3] = (uint8_t)(temp);
							Sdata[4] = 0xff;
							Sdata[5] = 0xff;
							
							while(USB_StatusDataSended==0);
							USB_StatusDataSended = 0;
							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
							
							printf("sleep current = %d uA\r\n",(temp*10));
							
/*****************************************退出低功耗*********************************/
							temp=0x01+0x04+0xf1;
							Write_PMU(0xef,0x01);
							Write_PMU(0x01,0x00);
							Write_PMU(0x04,0xf1);
							CS0_Low;				
							SPI1_ReadWriteByte(0x00);
							SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
							SPI1_ReadWriteByte((uint8_t)temp);//sum2
							CS0_High;
						
						if(!ReadyForCMD())
							{
								SendErrorCode(0,0x0000);
								printf("leave low resume error!");
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
								delay_ms(100);
			//					PWRON_1117;
			//					delay_ms(2);
			//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
			//					delay_ms(200);
								break;
							}
//						else 
//						{
//							delay_ms(100);
//							if(Current_MOD1_DET()<=0)
//								printf("leave low consume current = %d uA\r\n",0);
//							else
//								printf("leave low consume  current = %d uA\r\n",Current_MOD1_DET()*10);
//						}							
/************************************************************************************/							
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							break;
							
						case 3:
							#if 0
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToCMOS())
							{
								Write_PMU(0xf8,0xff);//restart
								num=500;
								while((--num)&&((Read_PMU(0x3f)&0x0400) != 0x0400)){delay_ms(1);}
								if(num == 0)
								{
									SendErrorCode(3,Read_PMU(0x3f));
									Write_PMU(0xcf,0x27);
									Write_PMU(0xf2,0xff);//sleep
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
									PWROFF_1117;
									
									USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
									return ;
								}
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								SPI1_ReadWriteByte(0x00);
								//CS0_Low;
								RST_PMU();
								CS0_High;
								CS_FLASH_High;
								delay_ms(200);
//								Current_MOD1_DET();
								if(Current_MOD1_DET()<=0)
									temp=0;
								else							
								temp = Current_MOD1_DET()+3;
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								CS0_High;
								printf("CMOS sleep current = %d uA\r\n",(temp*5));
								
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							#endif
							break;
							
						case 4:
							#if 0
							PWRON_1117;
							delay_ms(15);//等待电源稳定
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						
							RST_PMU();
						
							if(WriteRegToCMOS())
							{	
								Write_PMU(0xf3,0xff);
								delay_ms(20);
								
							if(Current_MOD1_DET()<=0)
								temp=0;
							else							
								temp = Current_MOD1_DET();
								while((--num)&&(Read_PMU(0x3f) != 0x8B0B))
								{
									delay_ms(1);
								}
								if(num==0)
								{
									SendErrorCode(2,Read_PMU(0x3f));
									Write_PMU(0xf8,0xff);//restart
									Write_PMU(0xcf,0x27);
									Write_PMU(0xf2,0xff);//sleep
									IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
									PWROFF_1117;
									return ;
								}
								Sdata[0] = 0xf0;
								Sdata[1] = 0x00;
								Sdata[2] = (uint8_t)(temp>>8);
								Sdata[3] = (uint8_t)(temp);
								Sdata[4] = 0xff;
								Sdata[5] = 0xff;
								while(USB_StatusDataSended==0);
								USB_StatusDataSended = 0;
								DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
								printf("CMOS F3FF current = %d uA\r\n",(temp*5));
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							else
							{
								Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);//sleep
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117;
							}
							#endif
							break;
							
						default :
							
							break;
						
					}
					IWDG_Init(5,2000);		//10s watchdog//2500// 6,1875
				}
				else
				{SendError();}
				break;				
				
			case 0x55a5://////////////spi update
				if(USB_Rx_Buffer[2]==1)
				{
				PWRON_1117;
			//	delay_ms(15);
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
				delay_ms(200);				
				}		
				CodeLength = (USB_Rx_Buffer[4]<<24)|(USB_Rx_Buffer[5]<<16)|(USB_Rx_Buffer[6]<<8)|(USB_Rx_Buffer[7]);
				temp=(CodeLength+4095)/4096;
				Sdata[0]=0xef;
				Sdata[1]=0x01;
				Sdata[2]=0x08;//  08/0f				
				Sdata[3]=0;
				Sdata[4]=USB_Rx_Buffer[2];//pack_num
				if(USB_Rx_Buffer[2]==temp) //pack num 4k/pack
				{
					Sdata[2]=0x0f;
				}
				
				Sdata[4101]=0xaa;
				Sdata[4102]=0x00;
				temp=1+9+6+USB_Rx_Buffer[4]+USB_Rx_Buffer[5]+USB_Rx_Buffer[6]+USB_Rx_Buffer[7]+USB_Rx_Buffer[8]+USB_Rx_Buffer[9];	
				
				if(USB_Rx_Buffer[2]==1)
				{
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
				SPI1_ReadWriteByte(0x01);
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x09);
				SPI1_ReadWriteByte(0x06);
				SPI1_ReadWriteByte(USB_Rx_Buffer[4]);//code_size
				SPI1_ReadWriteByte(USB_Rx_Buffer[5]);
				SPI1_ReadWriteByte(USB_Rx_Buffer[6]);
				SPI1_ReadWriteByte(USB_Rx_Buffer[7]);
				
				SPI1_ReadWriteByte(USB_Rx_Buffer[8]);//crc
				SPI1_ReadWriteByte(USB_Rx_Buffer[9]);
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
				}
				delay_ms(10);
				SendOK();
				Sensor_Update=1;

				break;	
			case 0x55a6://///////////设置2字节参数
				HANDSHAKE();
				temp = 1+5+0x80+USB_Rx_Buffer[2]+USB_Rx_Buffer[3];
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
				SPI1_ReadWriteByte(0x01);
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);
				SPI1_ReadWriteByte(0x80);
				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
			    SendOK();
					break;

			case 0x55e7: //mk510 write auto gain
					HANDSHAKE();
					mk510_which_gain = USB_Rx_Buffer[2];
					mk510_last_pack  = USB_Rx_Buffer[3];
					SendOK();
					IWDG_ReloadCounter();//weigou 
					SensorGain_mk510 = 1;
				break;			
		
			case 0x55c7: //mk110 write auto gain

					Sdata[0]=0xef;
					Sdata[1]=0x01;
					Sdata[2]=0x08;
					Sdata[3]=0;
					Sdata[4]=USB_Rx_Buffer[3];
			
					if(USB_Rx_Buffer[3]==62)
					{
						Sdata[2]=0x0f;
					}
//					else
//					{
//						Sdata[4]=USB_Rx_Buffer[3];
//					}
					
					Sdata[4101]=0xaa; 
					Sdata[4102]=0x00;
					
//					if(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1)
//					{delay_ms(10);}
					
					if((USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1))
					{
						if(USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)
						HANDSHAKE();
						temp=0x01+0x05+0x04+USB_Rx_Buffer[2];
						
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
						SPI1_ReadWriteByte(0x04);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
						
//						if(!ReadyForCMD())
//						{
//							SendErrorCode(0,0x0000);//data error
//							return;
//						}
					}
					SendOK();
					SensorGain_mk310=1;
//				}
				break;			
			
			case 0x55b7://mk210 write auto gain

					Sdata[0]=0xef;
					Sdata[1]=0x01;
					Sdata[2]=0x08;
					Sdata[3]=0;
					Sdata[4]=USB_Rx_Buffer[3];
			 
					if(USB_Rx_Buffer[3]==0x11)
					{
						Sdata[2]=0x0f;
					}
//					else
//					{
//						Sdata[4]=USB_Rx_Buffer[3];
//					}
					
					Sdata[4101]=0xaa; 
					Sdata[4102]=0x00;
					
//					if(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1)
//					{delay_ms(10);}
					
					if((USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1))
					{
						if(USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)
						HANDSHAKE();
						temp=0x01+0x05+0x04+USB_Rx_Buffer[2];
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
						SPI1_ReadWriteByte(0x04);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
						
//						if(!ReadyForCMD())
//						{
//							SendErrorCode(0,0x0000);//data error
//							return;
//						}
					}					
					SendOK();
					SensorGain_mk210=1;			
					break;
			 case 0x55f7: //mk610 write auto gain

					Sdata[0]=0xef;
					Sdata[1]=0x01;
					Sdata[2]=0x08;
					Sdata[3]=0;
					Sdata[4]=USB_Rx_Buffer[3];
			
					if(USB_Rx_Buffer[3]==49)
					{
						Sdata[2]=0x0f;
					}
					
					Sdata[4101]=0xaa; 
					Sdata[4102]=0x00;
					
					if((USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==3&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==4&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==5&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==6&&USB_Rx_Buffer[3]==1))
					{
//						if(USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)
						HANDSHAKE();
						temp=0x01+0x05+0x04+USB_Rx_Buffer[2];
						
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
						SPI1_ReadWriteByte(0x04);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
					}
					
					SendOK();
					SensorGain_mk610=1;
				break;	

				case 0x5537: //mk710-small write auto gain

					Sdata[0]=0xef;
					Sdata[1]=0x01;
					Sdata[2]=0x08;
					Sdata[3]=0;
					Sdata[4]=USB_Rx_Buffer[3];
			
					if(USB_Rx_Buffer[3]==38)
					{
						Sdata[2]=0x0f;
					}
					
					Sdata[4101]=0xaa; 
					Sdata[4102]=0x00;
					
					if((USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==3&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==4&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==5&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==6&&USB_Rx_Buffer[3]==1))
					{
//						if(USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)
						HANDSHAKE();
						temp=0x01+0x05+0x04+USB_Rx_Buffer[2];
						
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
						SPI1_ReadWriteByte(0x04);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
					}
					
					SendOK();
					SensorGain_mk710_B=1;
				break;										

			case 0x5527: //mk710 write auto gain

					Sdata[0]=0xef;
					Sdata[1]=0x01;
					Sdata[2]=0x08;
					Sdata[3]=0;
					Sdata[4]=USB_Rx_Buffer[3];
			
					if(USB_Rx_Buffer[3]==61) //38
					{
						Sdata[2]=0x0f;
					}
					
					Sdata[4101]=0xaa; 
					Sdata[4102]=0x00;
					
					if((USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==3&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==4&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==5&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==6&&USB_Rx_Buffer[3]==1))
					{
//						if(USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)
						HANDSHAKE();
						temp=0x01+0x05+0x04+USB_Rx_Buffer[2];
						
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
						SPI1_ReadWriteByte(0x04);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
					}
					
					SendOK();
					SensorGain_mk710=1;
				break;					
					
			case 0x55a7: //mk110 write auto gain

					Sdata[0]=0xef;
					Sdata[1]=0x01;
					Sdata[2]=0x08;
					Sdata[3]=0;
					Sdata[4]=USB_Rx_Buffer[3];
			
					if(USB_Rx_Buffer[3]==55)
					{
						Sdata[2]=0x0f;
					}
					
					Sdata[4101]=0xaa; 
					Sdata[4102]=0x00;
					
					if((USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)||(USB_Rx_Buffer[2]==2&&USB_Rx_Buffer[3]==1))
					{
						if(USB_Rx_Buffer[2]==1&&USB_Rx_Buffer[3]==1)
						HANDSHAKE();
						temp=0x01+0x05+0x04+USB_Rx_Buffer[2];
						
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x01);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
						SPI1_ReadWriteByte(0x04);
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
					}
					
					SendOK();
					SensorGain=1;
				break;
					
			case 0x551c:
				PWRON_1117;
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
				delay_ms(200);			

			do{
				delay_ms(10);

				CS0_Low;	
				
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x03);
			
				SPI1_ReadWriteByte(0x07);
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x0b);
				
				CS0_High;
				
				IWDG_ReloadCounter();
				version_state=ReadyFor_moduleCMD();
//				delay_ms(5);
				}while(((version_time++)<9)&&(version_state==2||version_state==0));
				if(version_time==9||version_state==2||version_state==0)
				{
					version_time=0;
				}	
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(100);
				
				break;

			case 0x553c: //check mk module if conmunicate
//				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff)) 
//				HANDSHAKE();
				PWRON_1117;
			//	delay_ms(15);
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
				delay_ms(49);			
//				delay_ms(4000);
				//	if(Handshake())
//		printf("handshake_using_10ms\r\n");
				CS0_Low;	
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x03);
			
				SPI1_ReadWriteByte(0x07);
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x0b);
				CS0_High;
				
				delay_ms(20);
					
				if(!ReadyFor_moduleCMD())
				{
				//	IWDG_ReloadCounter();
				//	delay_ms(5000);
				//	IWDG_ReloadCounter();	
					/*****************第二次查询*******************/
//					CS0_Low;	
//					SPI1_ReadWriteByte(0xef);
//					SPI1_ReadWriteByte(0x01);
//				
//					SPI1_ReadWriteByte(0x01);
//				
//					SPI1_ReadWriteByte(0x00);
//					SPI1_ReadWriteByte(0x03);
//				
//					SPI1_ReadWriteByte(0x07);
//				
//					SPI1_ReadWriteByte(0x00);
//					SPI1_ReadWriteByte(0x0b);
//					CS0_High;
					
//					delay_ms(20);	
//					if(!ReadyFor_moduleCMD())
//					{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);		
					break;
//					}
/*****************第二次查询*******************/					
				}
//				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
//				{Send_FWID();}
//				else{SendError();}
//				break;
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					break;				
			case 0x55b8: //mk210
				
				HANDSHAKE();	
//*******************************************************************************************//
			if((Line_H!=0)||(Line_L!=0))
			{			
				temp = 1+5+0x80+Line_H+Line_L;

				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);

				SPI1_ReadWriteByte(0x01);
					
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);

				SPI1_ReadWriteByte(0x80);
			
				SPI1_ReadWriteByte(Line_H);
				SPI1_ReadWriteByte(Line_L);

				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);					
					break;
				}
				delay_ms(5);
				
			} 
//*****************************************以上是设置积分时间部分****************************//				
				temp=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
		
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
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
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
					delay_ms(50);						
					break;
				}
				
			
				for(num=0;num<2;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);								
						break;
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
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);								
						break;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
//				SendOK();
//				while(USB_StatusDataSended==0);
////				USB_StatusDataSended = 0;
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,44032);//43K
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(50);
				break;

			case 0x5538: //win mk110
				
				HANDSHAKE();	
//*******************************************************************************************//
			if((Line_H!=0)||(Line_L!=0))
			{			
				temp = 1+5+0x80+Line_H+Line_L;			
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
								
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);
			
				SPI1_ReadWriteByte(0x80);
			
				SPI1_ReadWriteByte(Line_H);
				SPI1_ReadWriteByte(Line_L);
			
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);						
					break;
				}
				delay_ms(5);
			}
//*****************************************以上是设置积分时间部分****************************//	
				temp=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];		
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
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
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
					break;
				}
			
				for(num=0;num<2;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						break;
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
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						break;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
//				SendOK();
//				while(USB_StatusDataSended==0);
////				USB_StatusDataSended = 0;
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,44032);//43K
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
//					delay_ms(100);
				break;				
				
			case 0x5528: //win mk110
				
				HANDSHAKE();	
//*******************************************************************************************//
			if((Line_H!=0)||(Line_L!=0))
			{			
				temp = 1+5+0x80+Line_H+Line_L;			
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
								
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);
			
				SPI1_ReadWriteByte(0x80);
			
				SPI1_ReadWriteByte(Line_H);
				SPI1_ReadWriteByte(Line_L);
			
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);						
					break;
				}
				delay_ms(5);
			}
//*****************************************以上是设置积分时间部分****************************//	
				temp=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];		
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
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
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
					break;
				}
			
				for(num=0;num<2;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						break;
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
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						break;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
//				SendOK();
//				while(USB_StatusDataSended==0);
////				USB_StatusDataSended = 0;
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,44032);//43K
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
//					delay_ms(100);
				break;
				
			case 0x55a8: //win mk110
				
				HANDSHAKE();	
//*******************************************************************************************//
			if((Line_H!=0)||(Line_L!=0))
			{			
				temp = 1+5+0x80+Line_H+Line_L;			
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
								
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);
			
				SPI1_ReadWriteByte(0x80);
			
				SPI1_ReadWriteByte(Line_H);
				SPI1_ReadWriteByte(Line_L);
			
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);						
					break;
				}
				delay_ms(5);
			}
//*****************************************以上是设置积分时间部分****************************//	
				temp=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];		
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
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
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
					break;
				}
			
				for(num=0;num<2;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						break;
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
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);							
						break;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
//				SendOK();
//				while(USB_StatusDataSended==0);
////				USB_StatusDataSended = 0;
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,44032);//43K
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
//					delay_ms(100);
				break;

		case 0x55f8:
				
				HANDSHAKE();	
//*******************************************************************************************//
		 if((Line_H!=0)||(Line_L!=0))
			{			
				temp = 1+5+0x80+Line_H+Line_L;
			
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
					
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);
			
				SPI1_ReadWriteByte(0x80);
			
				SPI1_ReadWriteByte(Line_H);
				SPI1_ReadWriteByte(Line_L);
			
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					break;
				}
				delay_ms(5);
			 }
//*****************************************以上是设置积分时间部分****************************//				
				temp=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
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
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
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
					delay_ms(50);
					break;
				}
				
			
				for(num=0;num<2;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);						
						break;
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
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);						
						break;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				delay_ms(5);
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
//				SendOK();
//				while(USB_StatusDataSended==0);
////				USB_StatusDataSended = 0;
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,44032);//43K
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
//					delay_ms(50);
				break;				

			case 0x55e8:
				
				HANDSHAKE();	
//*******************************************************************************************//
		 if((Line_H!=0)||(Line_L!=0))
			{			
				temp = 1+5+0x80+Line_H+Line_L;
			
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
					
			
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);
			
				SPI1_ReadWriteByte(0x80);
			
				SPI1_ReadWriteByte(Line_H);
				SPI1_ReadWriteByte(Line_L);
			
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					break;
				}
				delay_ms(5);
			 }
//*****************************************以上是设置积分时间部分****************************//				
				temp=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
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
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
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
					delay_ms(50);
					break;
				}
				
			
				for(num=0;num<2;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);						
						break;
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
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);						
						break;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				delay_ms(5);
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
//				SendOK();
//				while(USB_StatusDataSended==0);
////				USB_StatusDataSended = 0;
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,44032);//43K
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
//					delay_ms(50);
				break;
				
			case 0x55c8:
				
				HANDSHAKE();
//*******************************************************************************************//
			if((Line_H!=0)||(Line_L!=0))
			{			
				temp = 1+5+0x80+Line_H+Line_L;
			
				CS0_Low;
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
			
				SPI1_ReadWriteByte(0x01);
					
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x05);
			
				SPI1_ReadWriteByte(0x80);
			
				SPI1_ReadWriteByte(Line_H);
				SPI1_ReadWriteByte(Line_L);
			
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)temp);
				CS0_High;
			
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					break;
				}
				delay_ms(5);
			}
//*****************************************以上是设置积分时间部分****************************//				
				temp=1+12+0x10+1+USB_Rx_Buffer[2]+USB_Rx_Buffer[3]+USB_Rx_Buffer[4]+USB_Rx_Buffer[5];
			
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
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
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
					delay_ms(50);
					break;
				}
				
			
				for(num=0;num<2;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);						
						break;
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
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);						
						break;
					}
					
//					if(num==0)
//					{SendOK();}
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);//43K
				}
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
//				SendOK();
//				while(USB_StatusDataSended==0);
////				USB_StatusDataSended = 0;
//				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,44032);//43K
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(50);
				break;

			case 0x55e9: //mk510 read gain
				HANDSHAKE();	
				mk510_which_gain=USB_Rx_Buffer[3];		
				for(num=0;num<60;num++)
				{
					IWDG_ReloadCounter();//weigou 
					if(mk510_which_gain==1)
					{
						W25QXX_Read(Sdata,mk510_first_gain_addr,25600);
						mk510_first_gain_addr+=25600;
					}
					else if(mk510_which_gain==2)
					{
						W25QXX_Read(Sdata,mk510_second_gain_addr,25600);
						mk510_second_gain_addr+=25600;						
					}		
					SendOK();
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					if(num==60)
					{			
							mk510_first_gain_addr=0;
							mk510_second_gain_addr=0x180000;

							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							delay_ms(20);		
					}		
				}				
				
				break;				
				
			case 0x55c9: //mk310 read gain
				
				HANDSHAKE();
				temp=1+5+5+USB_Rx_Buffer[3];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(0);   
				SPI1_ReadWriteByte(5);
				SPI1_ReadWriteByte(5);  
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
				CS0_High;
					
//				delay_ms(1);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);

					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					
//					PWRON_1117;
//					delay_ms(2);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(20);	
					
					break;
				}
				
			
				for(num=0;num<10;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);

						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(20);							
						
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);

						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(20);							
						
						break;
					}
				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(20);				
				break;				
				
			case 0x55b9: //mk210 read gain
				HANDSHAKE();
				temp=1+5+5+USB_Rx_Buffer[3];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(0);   
				SPI1_ReadWriteByte(5);
				SPI1_ReadWriteByte(5);  
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
				CS0_High;
					
//				delay_ms(1);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					break;
				}
				
			
				for(num=0;num<3;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						break;
					}
				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(20);
				
				break;
			case 0x55f9: //mk110 read gain
				
				HANDSHAKE();
				temp=1+5+5+USB_Rx_Buffer[3];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(0);   
				SPI1_ReadWriteByte(5);
				SPI1_ReadWriteByte(5);  
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
				CS0_High;
					
//				delay_ms(1);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					break;
				}
				
			
				for(num=0;num<8;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(20);
				
				break;				
			case 0x55a9: //mk110 read gain
				
				HANDSHAKE();
				temp=1+5+5+USB_Rx_Buffer[3];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(0);   
				SPI1_ReadWriteByte(5);
				SPI1_ReadWriteByte(5);  
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
				CS0_High;
					
//				delay_ms(1);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					break;
				}
				
			
				for(num=0;num<9;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(20);
				
				break;
			
			case 0x5529: //mk710 read gain
				
				HANDSHAKE();
				temp=1+5+5+USB_Rx_Buffer[3];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(0);   
				SPI1_ReadWriteByte(5);
				SPI1_ReadWriteByte(5);  
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
				CS0_High;
					
//				delay_ms(1);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					break;
				}
				
			
				for(num=0;num<6;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(100);
				
				break;
				
		case 0x5539: //mk710 read gain
				
				HANDSHAKE();
				temp=1+5+5+USB_Rx_Buffer[3];
			
				CS0_Low;
	
				SPI1_ReadWriteByte(0xef);   
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(1);
				SPI1_ReadWriteByte(0);   
				SPI1_ReadWriteByte(5);
				SPI1_ReadWriteByte(5);  
				SPI1_ReadWriteByte(0);
				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				
				CS0_High;
					
//				delay_ms(1);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					break;
				}
				
			
				for(num=0;num<8;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						break;
					}
				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(100);
				
				break;				
				
				
			case 0x55aa://单帧采集
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					PWRON_1117;
					delay_ms(14);//等待电源稳定
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
					delay_ms(2);
				#ifdef	DEBUG
					printf(" %d \r\n",Voltage_MOD1_DET());
				#endif
					RST_PMU();
				
					if(WriteRegToPMU())
					{
						Write_PMU(0xf6,0xff);
						delay_ms(1);
						switch(USB_Rx_Buffer[3])
						{
							case 1:
								Write_PMU(0xbe,0x21);
								//Write_PMU(0xf6,0xff);
								SendSingleFrameImage(0x40);
								break;
							case 2:
								Write_PMU(0xbe,0x21);
								//Write_PMU(0xf6,0xff);
								SendSingleFrameImage(0x00);
								break;
							case 3:
								Write_PMU(0xbe,0x20);
								//Write_PMU(0xf6,0xff);
								SendSingleFrameImage(0x40);
								break;
							case 4:
								Write_PMU(0xbe,0x20);
								//Write_PMU(0xf6,0xff);
								SendSingleFrameImage(0x00);
								break;
							case 5: 
								switch(USB_Rx_Buffer[2])
								{
									case 1:
										Write_PMU(0xbe,0x21);
										break;
									case 2:
										Write_PMU(0xbe,0x20);
										break;
									default :
										break;
								}
								//SendSingleFrameImage(0x40);
								Write_PMU(0xf2,0xff);
								Write_PMU(0x9d,Reg9d1);
								Write_PMU(0x9e,Reg9e1);
								
								SendOK();
								//return;
							
								break;
							default : 
								//LED3_ON;
								SendSingleFrameImage(0x40);
								break;
						}
					}
					else
					{
						if(USB_Rx_Buffer[3] == 5)
						{
							Write_PMU(0xf8,0xff);//restart
							Write_PMU(0xf2,0xff);//sleep
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
						}
					}
					
					if(USB_Rx_Buffer[3] == 5)
					{
						USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
					}
					else
					{
						Write_PMU(0xf8,0xff);//restart
						Write_PMU(0xf2,0xff);//sleep
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
					}
				}
				else{SendError();}
				
				break;
				
			case 0x55ab://连续采集
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					
					switch(USB_Rx_Buffer[3])
					{
						case 1:
							cmd1 = 0xf4;
							cmd2 = 0x40;
							temp = 0x2099;
							break;
						case 2:
							cmd1 = 0xf3;
							cmd2 = 0x00;
							temp = 0x4069;
							break;
						case 3:
							cmd1 = 0xf4;
							cmd2 = 0x00;
							temp = 0x2099;
							break;
						case 4:
							cmd1 = 0xf3;
							cmd2 = 0x40;
							temp = 0x4069;
							SmartPhoneMod=1;
							break;
						case 5:
							cmd1 = 0xf3;
							cmd2 = 0x40;
							temp = 0x4069;
							Restart = 1;
							break;
						case 6:
							cmd1 = 0xf3;
							cmd2 = 0x40;
							temp = 0x4069;
							Rst = 1;
							break;
						default :
							cmd1 = 0xf3;
							cmd2 = 0x40;
							temp = 0x4069;
							break;
					}
					
					if(ContinueFlag == 1)
					{
						if(Restart == 1)
						{
							if(WakeUpProcess())
							{Write_PMU(cmd1,cmd2);}
							else
							{
								ContinueFlag = 0;
								break;
							}
						}
						else if(Rst == 1)
						{
							RST_PMU();
							if(!WriteRegToPMU())
							{
								TimeOutProcess();
								LED3_ON;
								break;
							}
							Write_PMU(cmd1,cmd2);
						}
						
						num = 950;
						while((--num)&&((Read_PMU(0x44)) != temp)){delay_ms(1);}//防止PMU不回0x4069的代码
						if(num==0)
						{
							TimeOutProcess();
							ContinueFlag = 0;
							SendErrorCode(2,Read_PMU(0x44));
							break;
						}

						SendOK();
						SendImageData();
						
						if(cmd1 == 0xf3)
						{
							if(Restart == 1)
							{
								if(SleepProcess()){}
								else{break;}
							}
							else if(Rst == 1)
							{
								Write_PMU(0xf8,0xff);
								//RST_PMU();
							}
							else
							{
								Write_PMU(0xf8,0xff);
								
								//GPIO_ResetBits(GPIOE,GPIO_Pin_5);
								num = 200;
								while((--num)&&((Read_PMU(0x44)) != 0x8002)){delay_ms(1);}
								if(num==0)
								{
									TimeOutProcess();
									ContinueFlag = 0;
									SendErrorCode(3,Read_PMU(0x44));
									break;
								}
								
								if(SmartPhoneMod == 1)
								{
									if((RegBE&0x01) == 0x01)
									{
										RegBE = 0x20;
									}
									else
									{
										RegBE = 0x21;
									}	
									//printf("%x\r\n",RegBE);
									Write_PMU(0xbe,RegBE);
								}
//								Write_PMU(0x88,0x01);
//								Write_PMU(0xf6,0xff);
//								delay_ms(1);
								
								Write_PMU(cmd1,cmd2);
								//GPIO_SetBits(GPIOE,GPIO_Pin_5);
							}
						}
					}
					else
					{
						PWRON_1117;
						delay_ms(14);//等待电源稳定
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(2);
					#ifdef	DEBUG
						printf(" %d \r\n",Voltage_MOD1_DET());
					#endif
						RST_PMU();
						if(!WriteRegToPMU())
						{
							TimeOutProcess();
							LED3_ON;
							break;
						}
						
						switch(USB_Rx_Buffer[2])
						{
							case 1:
								Write_PMU(0xbe,0x21);
								break;
							case 2:
								Write_PMU(0xbe,0x20);
								break;
							default :
								break;
						}
						
						if(SmartPhoneMod == 1)
						{
							RegBE = 0x21;
							Write_PMU(0xbe,RegBE);
							//printf("%x\r\n",RegBE);
						}
						
						if(!SendContinueImage(cmd1,cmd2,temp))
						{
							TimeOutProcess();
							break;
						}
						
						if(cmd1 == 0xf3)
						{
							if(Restart == 1)
							{
								if(SleepProcess()){}
								else{break;}
							}
							else if(Rst == 1)
							{
								Write_PMU(0xf8,0xff);
								RST_PMU();
							}
							else
							{
								Write_PMU(0xf8,0xff);				//restart
								//GPIO_ResetBits(GPIOE,GPIO_Pin_5);
								num = 200;
								while((--num)&&(Read_PMU(0x44)) != 0x8002){delay_ms(1);}
								if(num==0)
								{
									TimeOutProcess();
									SendErrorCode(3,Read_PMU(0x44));
									break;
								}
								if(SmartPhoneMod == 1)
								{
									if((RegBE&0x01) == 0x01)
									{
										RegBE = 0x20;
									}
									else
									{
										RegBE = 0x21;
									}	
									//printf("%x\r\n",RegBE);
									Write_PMU(0xbe,RegBE);
								}
//								Write_PMU(0x88,0x01);
//								Write_PMU(0xf6,0xff);
//								delay_ms(1);
								Write_PMU(cmd1,cmd2);
								//GPIO_SetBits(GPIOE,GPIO_Pin_5);
							}
						}
						ContinueFlag = 1;
					}
				}
				else{SendError();}
				break;
				
			case 0x55ac://cmos单帧采集
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					PWRON_1117;
					delay_ms(20);//等待电源稳定 
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
					delay_ms(2);
//				printf(" %d \r\n",Voltage_MOD1_DET());
//				printf(" %d \r\n",Voltage_MOD2_DET());
//				printf(" %d \r\n",Current_MOD1_DET());
//				printf(" %d \r\n",Current_MOD2_DET());	
					RST_PMU();
				
					if(WriteRegToCMOS())
					{
						switch(USB_Rx_Buffer[3])
						{
							case 5:
								//Write_PMU(0xf8,0xff);//restart
								Write_PMU(0xcf,0x27);
								Write_PMU(0xf2,0xff);
								
								SendOK();
								break;
							default :
								Write_PMU(0xf3,0xff);
//								delay_ms(Dtime);
//								Write_PMU(0xD2,0x28);
//								Write_PMU(0xD0,0x41);
//								Write_PMU(0xcf,0x00);
								//Write_PMU(0xDe,0x10);
								//Write_PMU(0xDf,0x10);
								num=500;
								while((--num)&&(Read_PMU(0x3f) != 0x8B0B)){delay_ms(1);}
								if(num==0)
								{
									SendErrorCode(2,Read_PMU(0x3f));
									break ;
								}
								SendOK();
								SendCMOSImageData();
//								Write_PMU(0xD2,0x20);
//								Write_PMU(0xD0,0x44);
//								Write_PMU(0xcf,0x05);
								//Write_PMU(0xDe,0x30);
								//Write_PMU(0xDf,0x30);
								break;
						}
					}
					else
					{
						if(USB_Rx_Buffer[3] == 5)
						{
							Write_PMU(0xf8,0xff);//restart
							Write_PMU(0xcf,0x27);
							Write_PMU(0xf2,0xff);//sleep
							delay_ms(1);
							Write_PMU(0xf8,0xff);//restart
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117; 
							delay_ms(5);
						}
					}
					
					if(USB_Rx_Buffer[3] == 5)
					{
						
					}
					else
					{
						Write_PMU(0xf8,0xff);//restart
						Write_PMU(0xcf,0x27);
						Write_PMU(0xf2,0xff);//sleep	
						delay_ms(1);
						Write_PMU(0xf8,0xff);//restart
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;  
						USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
					}
				}
				else{SendError();}
				
				break;	
				
			case 0x55ad://cmos连续采集
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					if(ContinueFlag == 1)
					{
						num=100;
						while((--num)&&((Read_PMU(0x3f)&0x0400) != 0x0400)){delay_ms(1);}
						if(num==0)
						{
							SendErrorCode(3,Read_PMU(0x3f));
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;  
							ContinueFlag = 0;
							break;
						}
						
						Write_PMU(0xf3,0xff);
						num=500;
						while((--num)&&(Read_PMU(0x3f) != 0x8B0B)){delay_ms(1);}
						if(num==0)
						{
							SendErrorCode(2,Read_PMU(0x3f));
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117; 
							ContinueFlag = 0;
							break ;
						}
						
						SendOK();
						SendCMOSImageData();
						Write_PMU(0xf8,0xff);
					}
					else
					{
						PWRON_1117;
						delay_ms(20);//等待电源稳定 
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(2);
					
						RST_PMU();
					
						if(WriteRegToCMOS())
						{
							Write_PMU(0xf3,0xff);
							num=500;
							while((--num)&&(Read_PMU(0x3f) != 0x8B0B)){delay_ms(1);}
							if(num==0)
							{
								SendErrorCode(2,Read_PMU(0x3f));
								IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
								PWROFF_1117; 
								break ;
							}
							SendOK();
							SendCMOSImageData();
						}
						else
						{
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;  
							USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
							break;
						}
						Write_PMU(0xf8,0xff);//restart
						
						ContinueFlag = 1;
					}
					
				}
				else{SendError();}
				
				break;	
			
			case 0x55b1: //set_frame_time
//				PWRON_1117;
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//				delay_ms(50);
				Line_H=USB_Rx_Buffer[2];
				Line_L=USB_Rx_Buffer[3];
//			
//				temp = 1+5+0x80+Line_H+Line_L;
//			
//				CS0_Low;
//				SPI1_ReadWriteByte(0xef);
//				SPI1_ReadWriteByte(0x01);
//			
//				SPI1_ReadWriteByte(0x01);
//			
//				SPI1_ReadWriteByte(0x00);
//				SPI1_ReadWriteByte(0x05);
//			
//				SPI1_ReadWriteByte(0x80);
//			
//				SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
//				SPI1_ReadWriteByte(USB_Rx_Buffer[3]);
//			
//				SPI1_ReadWriteByte((uint8_t)(temp>>8));
//				SPI1_ReadWriteByte((uint8_t)temp);
//				CS0_High;
//			
//				if(!ReadyForCMD())
//				{
//					SendErrorCode(0,0x0000);
//					break;
//				}
				
				SendOK();
				
				break;
				
		case 0x55be: //210采全图
				HANDSHAKE();
//*******************************************************************************************//
			if((Line_H!=0)||(Line_L!=0))
				{
					temp = 1+5+0x80+Line_H+Line_L;
				
					CS0_Low;
					SPI1_ReadWriteByte(0xef);
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x01);
				
					SPI1_ReadWriteByte(0x00);
					SPI1_ReadWriteByte(0x05);
				
					SPI1_ReadWriteByte(0x80);
				
					SPI1_ReadWriteByte(Line_H);
					SPI1_ReadWriteByte(Line_L);
				
					SPI1_ReadWriteByte((uint8_t)(temp>>8));
					SPI1_ReadWriteByte((uint8_t)temp);
					CS0_High;
				
					if(!ReadyForCMD())  
					{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);					
						break;
					}
					delay_ms(5);		
			   }
//*****************************************以上是设置积分时间部分****************************//		
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x03,0x02);
				temp=1+3+2;
				CS0_Low;				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				CS0_High;
					
//				delay_ms(20);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
//					PWRON_1117;
//					delay_ms(2);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(200);
					break;
				}
					
				for(num=0;num<4;num++) //4 package
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
				}

				capture_num=0;
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(5);
				
				break;
			
			case 0x559e://huawei  
				if(capture_num==0)
				{
					HANDSHAKE();

//				delay_ms(5);
				}
				capture_num++;	
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x05,0x02);
			
				switch(USB_Rx_Buffer[3])
				{
					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9
						
						break;
					case 2:
						Write_PMU(0x00,0x02);//2
						Write_PMU(0x00,0x0a);//a
					
						break;
				}	
					
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
					break;
				}
				
			
				for(num=0;num<5;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(20);
						PWRON_1117;
						delay_ms(2);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(200);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(20);
						PWRON_1117;
						delay_ms(2);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
						delay_ms(200);
						break;
					}
				}
				if(capture_num==2)
				{
				capture_num=0;
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(5);
				}
				
				break;
				
			case 0x558e://mk110 low resume into
				temp=0x01+0x04+0xf1+0x01;
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x04,0xf1);
				CS0_Low;				
				SPI1_ReadWriteByte(0x01);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				CS0_High;
			
			if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					printf("go into low resume error!");
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
//					PWRON_1117;
//					delay_ms(2);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(200);
					break;
				}
			else 
				{
					delay_ms(100);
					if(Current_MOD1_DET()<=0)
					printf("VMOD1 disable current = %d uA\r\n",0);
					else
					printf("VMOD1 disable current = %d uA\r\n",Current_MOD1_DET()*10);
				}
			SendOK();
			break;
			
			case 0x557e:// leave low consume mk110
				temp=0x01+0x04+0xf1;
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x04,0xf1);
				CS0_Low;				
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte((uint8_t)(temp>>8));//sum1
				SPI1_ReadWriteByte((uint8_t)temp);//sum2
				CS0_High;
			 
			if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					printf("leave low resume error!");
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
//					PWRON_1117;
//					delay_ms(2);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(200);
					break;
				}
			else 
			{
				delay_ms(100);
				if(Current_MOD1_DET()<=0)
					printf("leave low consume current = %d uA\r\n",0);
				else
					printf("leave low consume  current = %d uA\r\n",Current_MOD1_DET()*10);
			}
			SendOK();
				break;
			
			case 0x54ce: //module test vol
				HANDSHAKE();
				while(1)
				{
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x03,0xf3);
				Write_PMU(0x00,0xf7);
				if(!ReadyForMK_VOL())
				{
//					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(20);
					PWRON_1117;
					delay_ms(2);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
					delay_ms(20);
				}
					delay_ms(1000);
				}
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				PWROFF_1117;
				delay_ms(20);
				break;
				
			case 0x55ce:
				if(capture_num==0)
				{
					HANDSHAKE();
//*******************************************************************************************//
					if((Line_H!=0)||(Line_L!=0))
					{					
						temp = 1+5+0x80+Line_H+Line_L;
					
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
					
						SPI1_ReadWriteByte(0x80);
					
						SPI1_ReadWriteByte(Line_H);
						SPI1_ReadWriteByte(Line_L);
					
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
					
						if(!ReadyForCMD())
						{
							SendErrorCode(0,0x0000);
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							delay_ms(50);	
//							capture_num=0;
							break;
						}
						delay_ms(5);
				 	}		
//*****************************************以上是设置积分时间部分****************************//						
				}
					capture_num++;	
					Write_PMU(0xef,0x01);
					Write_PMU(0x01,0x00);
					Write_PMU(0x05,0x02);
			
				switch(USB_Rx_Buffer[3])
				{
					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9
						
						break;
					case 2:
						Write_PMU(0x00,0x02);//2
						Write_PMU(0x00,0x0a);//a
					
						break;
				}	
					
				delay_ms(20);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(50);
					capture_num=0;
//					PWRON_1117;
//					delay_ms(2);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(200);
					break;
				}
				
				for(num=0;num<5;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);
						capture_num=0;
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);
						capture_num=0;
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1p);//Power on buffer
//						delay_ms(200);
						break;
					}
				}
				if(capture_num==2)
				{
				capture_num=0;
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117;
//				delay_ms(100);
				}
				break;

			case 0x55f1:
				
				switch(USB_Rx_Buffer[2])
				{
					case 0:
						cmd1=0;
						break;
					case 1:
						cmd1 =1;//Vdd
					break;
					case 2:
						cmd1=2;//Vee
					break;
					case 3:
						cmd1=3;//Vcom
					break;
				}
				
				temp=1+4+0xf2+cmd1;
				
				CS0_Low;
				
				SPI1_ReadWriteByte(0xef);
				SPI1_ReadWriteByte(0x01);
				
				SPI1_ReadWriteByte(0x01);
				
				SPI1_ReadWriteByte(0x00);
				SPI1_ReadWriteByte(0x04);
				
				SPI1_ReadWriteByte(0xf2);
				
				SPI1_ReadWriteByte(cmd1);
				
				SPI1_ReadWriteByte((uint8_t)(temp>>8));
				SPI1_ReadWriteByte((uint8_t)(temp));
				
				CS0_High;
				
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					printf("Error");
					break;
				}
				SendOK();
				break;
				
			case 0x55fe:
				if(capture_num==0)
				{
					HANDSHAKE();

//*******************************************************************************************//
					if((Line_H!=0)||(Line_L!=0))
					{					
						temp = 1+5+0x80+Line_H+Line_L;
					
						CS0_Low;
						
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
					
						SPI1_ReadWriteByte(0x80);
				
						SPI1_ReadWriteByte(Line_H);
						SPI1_ReadWriteByte(Line_L);
					
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						
						CS0_High;
					
						if(!ReadyForCMD())
						{
							SendErrorCode(0,0x0000);
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							delay_ms(50);					
							break;
						}
						delay_ms(5);
					 }
//*****************************************以上是设置积分时间部分********************************//						
				}
				capture_num++;	
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x05,0x02);
			
				switch(USB_Rx_Buffer[3])
				{
					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9
						
						break;
					case 2:
						Write_PMU(0x00,0x02);//2
						Write_PMU(0x00,0x0a);//a
					
						break;
				}	
					
				delay_ms(20);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					capture_num=0;//2019.1.20
//					PWRON_1117;
//					delay_ms(2);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(200);
					break;
				}
				delay_ms(10);    //2018.12.18
				for(num=0;num<5;num++)
				{
					delay_ms(2);
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
				}
				if(capture_num==2)
				{
				capture_num=0;
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117; 
//				delay_ms(100);
				}			
				break;			
			case 0x55ee:
				if(capture_num==0)
				{
					HANDSHAKE();

//*******************************************************************************************//
					if((Line_H!=0)||(Line_L!=0))
					{					
						temp = 1+5+0x80+Line_H+Line_L;
					
						CS0_Low;
						
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
					
						SPI1_ReadWriteByte(0x80);
				
						SPI1_ReadWriteByte(Line_H);
						SPI1_ReadWriteByte(Line_L);
					
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						
						CS0_High;
					
						if(!ReadyForCMD())
						{
							SendErrorCode(0,0x0000);
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							delay_ms(50);					
							break;
						}
						delay_ms(5);
					 }
//*****************************************以上是设置积分时间部分****************************//						
				}
					capture_num++;	
					Write_PMU(0xef,0x01);
					Write_PMU(0x01,0x00);
					Write_PMU(0x05,0x02);
			
				switch(USB_Rx_Buffer[3])
				{
					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9
						
						break;
					case 2:
						Write_PMU(0x00,0x02);//2
						Write_PMU(0x00,0x0a);//a
					
						break;
					case 3:
						Write_PMU(0x00,0x03);//1
						Write_PMU(0x00,0x0B);//9
						
						break;
					case 4:
						Write_PMU(0x00,0x04);//2
						Write_PMU(0x00,0x0C);//a
					
						break;
					case 5:
						Write_PMU(0x00,0x05);//1
						Write_PMU(0x00,0x0D);//9
						
						break;
					case 6:
						Write_PMU(0x00,0x06);//2
						Write_PMU(0x00,0x0E);//a
					
						break;
					case 7:
						Write_PMU(0x00,0x07);//1
						Write_PMU(0x00,0x0F);//9
						
						break;
					case 8:
						Write_PMU(0x00,0x08);//2
						Write_PMU(0x00,0x10);//a	
					
						break;
					case 9:
						Write_PMU(0x00,0x09);//2
						Write_PMU(0x00,0x11);//a
					
						break;
					case 10:
						Write_PMU(0x00,0x0A);//1
						Write_PMU(0x00,0x12);//9
						
						break;
					case 11:
						Write_PMU(0x00,0x0B);//2
						Write_PMU(0x00,0x13);//a
					
						break;
					case 12:
						Write_PMU(0x00,0x0C);//1
						Write_PMU(0x00,0x14);//9
						
						break;
					case 13:
						Write_PMU(0x00,0x0D);//2
						Write_PMU(0x00,0x15);//a
					
						break;
					case 14:
						Write_PMU(0x00,0x0E);//1
						Write_PMU(0x00,0x16);//9
						
						break;
					case 15:
						Write_PMU(0x00,0x0F);//2
						Write_PMU(0x00,0x17);//a
					
						break;					
				}	
					
//				delay_ms(20);
				if(!ReadyForCMD())
				{
						SendErrorCode(0,0x0000);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);
						capture_num=0;
	//					PWRON_1117;
	//					delay_ms(2);
	//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	//					delay_ms(200);
						break;
				}
				
				for(num=0;num<5;num++)
				{
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);
						capture_num=0;
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(20);
						break;
					}
					
						SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(50);
						capture_num=0;
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(20);
						break;
					}
				}
					delay_ms(5);				
				if(capture_num==1)
				{
					capture_num=0;
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//					PWROFF_1117;
//					delay_ms(50);
				}
				break;

			case 0x552e://710 2个125
					HANDSHAKE();
					if((Line_H!=0)||(Line_L!=0))
					{
						delay_ms(5);
//*****************************************设置积分时间**************************************************//
						temp = 1+5+0x80+Line_H+Line_L;
					
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x01);
										
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
					
						SPI1_ReadWriteByte(0x80);
					
						SPI1_ReadWriteByte(Line_H);
						SPI1_ReadWriteByte(Line_L);
					
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
				
						if(!ReadyForCMD())
						{
							SendErrorCode(0,0x0000);
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							delay_ms(100);					
							break;
						}
						delay_ms(5);
					}		
//*****************************************以上是设置积分时间部分****************************//	
//			   }
//				capture_num++;	
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x05,0x02);
			
				switch(USB_Rx_Buffer[3])
				{
					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9					
						break;
					case 2:
						Write_PMU(0x00,0x02);//2
						Write_PMU(0x00,0x0a);//a				
						break;
					case 3:
						Write_PMU(0x00,0x03);//2
						Write_PMU(0x00,0x0b);//a				
						break;					 
				}	
					
					delay_ms(20);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					capture_num=0;//2019.1.20
					break;
				}
					delay_ms(10);    //2018.12.18
				for(num=0;num<5;num++)
				{
					delay_ms(2);
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
						break;
					}
				}		
				break;					
				
				
			case 0x553e://710 3个125
					HANDSHAKE();
					if((Line_H!=0)||(Line_L!=0))
					{
						delay_ms(5);
//*****************************************设置积分时间**************************************************//
						temp = 1+5+0x80+Line_H+Line_L;
					
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x01);
										
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
					
						SPI1_ReadWriteByte(0x80);
					
						SPI1_ReadWriteByte(Line_H);
						SPI1_ReadWriteByte(Line_L);
					
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
				
						if(!ReadyForCMD())
						{
							SendErrorCode(0,0x0000);
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							delay_ms(100);					
							break;
						}
						delay_ms(5);
					}		
//*****************************************以上是设置积分时间部分****************************//	
//			   }
//				capture_num++;	
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x05,0x02);
			
				switch(USB_Rx_Buffer[3])
				{
					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9						
						break;
					case 2:
						Write_PMU(0x00,0x02);//2
						Write_PMU(0x00,0x0a);//a					
						break;
//					case 3:
//						Write_PMU(0x00,0x03);//2
//						Write_PMU(0x00,0x0b);//a					
//						break;
					 
				}	
					
					delay_ms(20);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					capture_num=0;//2019.1.20
					break;
				}
					delay_ms(10);    //2018.12.18
				for(num=0;num<5;num++)
				{
					delay_ms(2);
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
						break;
					}
				}		
				break;

			case 0xea0a: //jiaozheng
				Power_MOD1_MOD2_Calibration(USB_Rx_Buffer);
				break;
			
			case 0xEB01://write gain************
				Write_Module_Gain(USB_Rx_Buffer);
				break;

			case 0xEB11://read gain to PC
				Read_Module_Gain(USB_Rx_Buffer);		
				break;
			
			case 0xec02: //KEY Detec
				Key_Detect(USB_Rx_Buffer);
				break;
				
			case 0xec03: //mk通用采图
				Capture_Full_Image(USB_Rx_Buffer);
				break;
			
			case 0xec04: //mk win通用采图
				Capture_Win_Image(USB_Rx_Buffer);
				break;
			
			case 0xEC08://normal current
				Normal_Current_Detect(USB_Rx_Buffer);
				break;
			
			case 0xEC0B://VCOM switch
				Module_SW_Vcom(USB_Rx_Buffer);
				break;
		
			case 0xEC0C://ClearFrameSet
				ClearFrameSet(USB_Rx_Buffer);
				break;
			
			case 0xEC0D://ReadClearFrameSetting
				ReadClearFrameSetting(USB_Rx_Buffer);
				break;
			
			case 0xEC0E://ReadFrameLineSetting
				ReadFrameLineSetting(USB_Rx_Buffer);
				break;
			
			case 0xEC0F://Modlue hardware setting
				ModlueHardwareSetting(USB_Rx_Buffer);
				break;
			
			case 0x55ae://
				if(capture_num==0)
				{
					HANDSHAKE();
					if((Line_H!=0)||(Line_L!=0))
					{
						delay_ms(5);
	//*****************************************设置积分时间**************************************************//
						temp = 1+5+0x80+Line_H+Line_L;
					
						CS0_Low;
						SPI1_ReadWriteByte(0xef);
						SPI1_ReadWriteByte(0x01);
					
						SPI1_ReadWriteByte(0x01);
										
						SPI1_ReadWriteByte(0x00);
						SPI1_ReadWriteByte(0x05);
					
						SPI1_ReadWriteByte(0x80);
					
						SPI1_ReadWriteByte(Line_H);
						SPI1_ReadWriteByte(Line_L);
					
						SPI1_ReadWriteByte((uint8_t)(temp>>8));
						SPI1_ReadWriteByte((uint8_t)temp);
						CS0_High;
				
						if(!ReadyForCMD())
						{
							SendErrorCode(0,0x0000);
							IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
							PWROFF_1117;
							delay_ms(100);					
							break;
						}
						delay_ms(5);
					}		
//*****************************************以上是设置积分时间部分****************************//	
			   }
				capture_num++;	
				Write_PMU(0xef,0x01);
				Write_PMU(0x01,0x00);
				Write_PMU(0x05,0x02);
			
				switch(USB_Rx_Buffer[3])
				{
					case 1:
						Write_PMU(0x00,0x01);//1
						Write_PMU(0x00,0x09);//9
						
						break;
					case 2:
						Write_PMU(0x00,0x02);//2
						Write_PMU(0x00,0x0a);//a
					
						break;
				}	
					
				delay_ms(20);
				if(!ReadyForCMD())
				{
					SendErrorCode(0,0x0000);
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					delay_ms(100);
					capture_num=0;//2019.1.20
//					PWRON_1117;
//					delay_ms(2);
//					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//					delay_ms(200);
					break;
				}
				delay_ms(10);    //2018.12.18
				for(num=0;num<5;num++)
				{
					delay_ms(2);
					if(!(ReceHead()))
					{
						SendErrorCode(num,0x0001);
						
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
					
					SPIReadData(Sdata);
					
					if(ReceEnd())
					{
						SendOK();
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,25600);
					}
					else
					{
						SendErrorCode(num,0x0002);
						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
						PWROFF_1117;
						delay_ms(100);
						capture_num=0;//2019.1.20
//						PWRON_1117;
//						delay_ms(2);
//						IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//						delay_ms(200);
						break;
					}
				}
				if(capture_num==2)
				{
				capture_num=0;
//				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
//				PWROFF_1117; 
//				delay_ms(100);
				}			
				break;			

			case 0x556b://mk610 采集窗口
				if((USB_Rx_Buffer[11] == 0xff) && (USB_Rx_Buffer[12] == 0xff))
				{	
				TouchMod = USB_Rx_Buffer[2];
				Touchmk610_win = 1;
				}
				break;		

			case 0x557b://mk610 采集窗口
				if((USB_Rx_Buffer[11] == 0xff) && (USB_Rx_Buffer[12] == 0xff))
				{	
				TouchMod = USB_Rx_Buffer[2];
				Touchmk710_win = 1;
				}
				break;	

			case 0x558b://mk610 采集窗口
				if((USB_Rx_Buffer[11] == 0xff) && (USB_Rx_Buffer[12] == 0xff))
				{	
				TouchMod = USB_Rx_Buffer[2];
				Touchmk710_win_B = 1;
				}
				break;				
				
			case 0x555b://mk510 采集窗口
				if((USB_Rx_Buffer[11] == 0xff) && (USB_Rx_Buffer[12] == 0xff))
				{	
				TouchMod = USB_Rx_Buffer[2];
				Touchmk510_win = 1;
				}
				break;	
				
			case 0x553b://mk310 采集窗口
				if((USB_Rx_Buffer[11] == 0xff) && (USB_Rx_Buffer[12] == 0xff))
				{	
				TouchMod = USB_Rx_Buffer[2];
				Touchmk310_win = 1;
				}
				break;	
				
			case 0x552b://mk210 采集窗口
				if((USB_Rx_Buffer[11] == 0xff) && (USB_Rx_Buffer[12] == 0xff))
				{
		
				TouchMod = USB_Rx_Buffer[2];
				TouchMK210_win = 1;
				}
				break;
				
			case 0x551b://mk110 采集窗口
				if((USB_Rx_Buffer[11] == 0xff) && (USB_Rx_Buffer[12] == 0xff))
				{	
				TouchMod = USB_Rx_Buffer[2];
				TouchOPPO_win = 1;
				}
				break;
			case 0x5510://Touch采集	
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					TouchMod = USB_Rx_Buffer[3];
					TouchFire = 1;
				}
				break;
			case 0x5518://Touch采集	
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					TouchMod = USB_Rx_Buffer[3];
					TouchCMOS = 1;
				}
				break;
			case 0x55af://停止采集
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{ 
					
//					Write_PMU(0xf8,0xff);
					//Write_PMU(0xcf,0x27)\\\\\\\\\\\\\
//					Write_PMU(0xf2,0xff);//sleep
					IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
					PWROFF_1117;
					LED4_OFF;
					ContinueFlag = 0;
					TouchOPPO_win=0;
					TouchMK210_win=0;
					Touchmk310_win=0;
					Touchmk510_win=0;
					Touchmk610_win=0;
					Touchmk710_win=0;
					Touchmk710_win_B=0;
					//GPIO_ResetBits(GPIOE,GPIO_Pin_5);
					//CS0_High;
					SendOK();
					USB_OTG_FlushTxFifo(&USB_OTG_dev,0);
				}		
				else{SendError();}
				break;
				
			case 0x5551: //led board 
				led0pwmval=(USB_Rx_Buffer[2]<<8)|USB_Rx_Buffer[3];
				if(led0pwmval>=0&&led0pwmval<=399)
				{	
					TIM_SetCompare1(TIM4,0);	//修改比较值，修改占空比
					TIM_SetCompare2(TIM4,(led0pwmval));	//修改比较值，修改占空比
					TIM_Cmd(TIM4, ENABLE);  //使能TIM4
//					if(led0pwmval==0)
//						TIM_Cmd(TIM4, DISABLE);  //除能TIM4
				}
				else 
				{
					TIM_SetCompare1(TIM4,0);	//修改比较值，修改占空比
//					TIM_SetCompare2(TIM4,0);	//修改比较值，修改占空比
					TIM_Cmd(TIM4, DISABLE);  //除能TIM4
				}
				 delay_ms(800);
				 ch0=tsl_chl0_read();
				 ch1=tsl_chl1_read();
				 //tsl2561_poweroff();
			//	 delay_ms(500);
				 Sdata[0]=0xf0;
				 Sdata[1]=0x00;
				 Sdata[2]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0)>>24);
				 Sdata[3]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0)>>16);
				 Sdata[4]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0)>>8);
				 Sdata[5]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0));
				 Sdata[6]=0xff;
				 Sdata[7]=0xff;
//				 SendOK();
				 while(USB_StatusDataSended==0);
//			     printf("^^^^88888888888\r\n");
				 USB_StatusDataSended = 0;
				 DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
				
				 printf(" ch0:%d \r\n",ch0);
				 printf(" ch1:%d \r\n",ch1);
				 printf("通道1光强度为:%d \r\n",CalculateLux(1,2,ch0,ch1,0));// 1,2,ch0,ch1,0
				 printf("通道1上传光强度为:%d \r\n",((uint32_t)Sdata[2]<<24)+((uint32_t)Sdata[3]<<16)+((uint32_t)Sdata[4]<<8)+Sdata[5]);
				 break;
			
			case 0x5552: //led board 
				led0pwmval=(USB_Rx_Buffer[2]<<8)|USB_Rx_Buffer[3];
				if(led0pwmval>=0&&led0pwmval<=399)
				{	
					TIM_SetCompare2(TIM4,0);	//修改比较值，修改占空比
					TIM_SetCompare1(TIM4,led0pwmval);	//修改比较值，修改占空比
					TIM_Cmd(TIM4, ENABLE);  //使能TIM4
//					if(led0pwmval==0)
//						TIM_Cmd(TIM4, DISABLE);  //除能TIM4
				}
				else 
				{
//					TIM_SetCompare1(TIM4,0);	//修改比较值，修改占空比
					TIM_SetCompare2(TIM4,0);	//修改比较值，修改占空比
					TIM_Cmd(TIM4, DISABLE);  //除能TIM4
				}
				 delay_ms(800);
				 ch0=tsl_chl0_read();
				 ch1=tsl_chl1_read();
				 //tsl2561_poweroff();
			//	 delay_ms(500);
				 Sdata[0]=0xf0;
				 Sdata[1]=0x00;
				 Sdata[2]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0)>>24);
				 Sdata[3]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0)>>16);
				 Sdata[4]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0)>>8);
				 Sdata[5]=(uint8_t)(CalculateLux(1,2,ch0,ch1,0));
				 Sdata[6]=0xff;
				 Sdata[7]=0xff;
//				 SendOK();
				 while(USB_StatusDataSended==0);
//			     printf("^^^^88888888888\r\n");
				 USB_StatusDataSended = 0;
				 DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
				
				 printf(" ch0:%d \r\n",ch0);
				 printf(" ch1:%d \r\n",ch1);
				 printf("通道2光强度为:%d \r\n",CalculateLux(1,2,ch0,ch1,0));// 1,2,ch0,ch1,0
				 printf("通道2上传光强度为:%d \r\n",((uint32_t)Sdata[2]<<24)+((uint32_t)Sdata[3]<<16)+((uint32_t)Sdata[4]<<8)+Sdata[5]);
				break;				
				
				
				
			
			/*************MCU FLASH Write&Read cmd*************/
			case 0x550a:
				WritePMUDataToMCU();
				break;
			
			case 0x550b:
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{
					ReadMCUPMUDataToPC();
				}
				else{SendError();}	
				break;
			
			case 0x550c:
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{
					CodeLength = STMFLASH_ReadWord(GAIN_INFOR_SAVE_ADDR);
					//CheckCRC = STMFLASH_ReadWord(GAIN_INFOR_SAVE_ADDR+4);
					if(CodeLength == 0xffffffff)
					{
						SendError();
					}
					else
					{
						Sdata[0] = 0xf0;
						Sdata[1] = 0x00;
						
						STMFLASH_ReadforByte(GAIN_INFOR_SAVE_ADDR,Sdata+2,12);//10----------->12
//						Sdata[2] = (uint8_t)(CodeLength>>24);
//						Sdata[3] = (uint8_t)(CodeLength>>16);
//						Sdata[4] = (uint8_t)(CodeLength>>8);
//						Sdata[5] = (uint8_t)(CodeLength);
//						
//						Sdata[6] = (uint8_t)(CheckCRC>>24);
//						Sdata[7] = (uint8_t)(CheckCRC>>16);
//						Sdata[8] = (uint8_t)(CheckCRC>>8);
//						Sdata[9] = (uint8_t)(CheckCRC);
						
						Sdata[14] = 0xff;
						Sdata[15] = 0xff;
						
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
					}
				}
				else{SendError();}
				break;	
			
			case 0x550d:
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{
					STMFLASH_ReadforByte(GAIN_INFOR_SAVE_ADDR,Sdata,4);
					CodeLength = ((Sdata[0]<<24)|(Sdata[1]<<16)|(Sdata[2]<<8)|(Sdata[3]))+1;
					SendOK();
					for(num=0;num<CodeLength;num=num+512)
					{
						//W25QXX_Read(Sdata,0x4000+i,512);
						while(USB_StatusDataSended==0);
						USB_StatusDataSended = 0;
						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,(uint8_t*)(APP_DEFAULT_ADD+num),512);
					}
					
//					if((CodeLength+1)<=0xffff)
//					{
//						while(USB_StatusDataSended==0);
//						USB_StatusDataSended = 0;
//						DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,(uint8_t*)APP_DEFAULT_ADD,CodeLength+1);
//					}
//					else
//					{
//						//num=(CodeLength+1)/0xffff;
//						for(num=0;num<((CodeLength+1)/0xffff);num++)
//						{
//							while(USB_StatusDataSended==0);
//							USB_StatusDataSended = 0;
//							DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,(uint8_t*)APP_DEFAULT_ADD,65536);
//						}
//					}
					
				}
				else{SendError();}	
				break;
			
			case 0x550e:
				if((USB_Rx_Buffer[14] == 0xff) && (USB_Rx_Buffer[15] == 0xff))
				{
					CodeLength = (USB_Rx_Buffer[2]<<24)|(USB_Rx_Buffer[3]<<16)|(USB_Rx_Buffer[4]<<8)|(USB_Rx_Buffer[5]);
					CheckCRC = (USB_Rx_Buffer[8]<<8)|(USB_Rx_Buffer[9]);
					STMFlash_EraseSect(APP_DEFAULT_ADD, CodeLength+1);
					STMFLASH_ReadforByte(FLASH_SAVE_ADDR,Sdata,500);
					for(num=4;num<16;num++)//14--------->16
					{
						Sdata[num]=USB_Rx_Buffer[num-2];
					}
					STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)Sdata,500);
					printf("MCU Gain data: %x  %x\r\n",CodeLength,CheckCRC);
					GainDataFlag=1;
					SendOK();
				}	
				break;
				
			case 0x550f:
				
				break;
				
			/***************FLASH Read cmd****************/
			
			case 0x5400:
				ReadW25QXXPMUDataToPC();
				break;
			case 0x551f://read mk110_sn
				Read_mk110_sn();
				break;
			case 0x5401:
				ReadW25QXXPCBANumToPC();
				break;
			
			case 0x5402:
				ReadW25QXXModuleNumToPC();
				break;
			
			case 0x5403:
				ReadW25QXXColorNumToPC();
				break;
			case 0x5404:
				
				break;
			
			case 0x5405:
				ReadW25QXXGAIN_AVRToPC();
				break;
			case 0x5406:
				ReadW25QXXGAIN_InforToPC();
				break;
			case 0x5407:
				SendOK();
				ReadW25QXXGAINDataToPC();
				break;
			case 0x5408:
				ReadW25QXXCMOSDataToPC();
				break;
			
			/*************FLASH Write cmd**************/
			
			case 0x5410:
				WritePMUDataToW25QXX();
				break;
			case 0x551d://write mk110_
				WriteMK110_SN();
				break;
			case 0x5411:	
				WritePCBANumToW25QXX();
				break;
			case 0x5412:	
				WriteModuleNumToW25QXX();
				break;
			case 0x5413:	
				WriteColorNumToW25QXX();
				break;
				
			case 0x5414:
				
				break;
			case 0x5415:
				WriteGAIN_AVRToW25QXX();
				break;
			
			case 0x5416:
				if((USB_Rx_Buffer[14] == 0xff) && (USB_Rx_Buffer[15] == 0xff))
				{
					CodeLength = (USB_Rx_Buffer[2]<<24)|(USB_Rx_Buffer[3]<<16)|(USB_Rx_Buffer[4]<<8)|(USB_Rx_Buffer[5]);
					CheckCRC = (USB_Rx_Buffer[8]<<8)|(USB_Rx_Buffer[9]);
					printf("Flash Gain data: %x  %x\r\n",CodeLength,CheckCRC);
					WriteGAIN_InforToW25QXX();
					STMFlash_EraseSect(APP_DEFAULT_ADD, CodeLength+1);
					SendOK();
					FlashDataFlag=1;
				}
				break;
				
			case 0x5418:
				WriteCMOSDataToW25QXX();
				//SendOK();
				break;
			
			/******************old cmd*****************/		
			case 0xf040:														//power on finger module
				
				PWRON_1117;
//				delay_ms(15);
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
				delay_ms(100);
				SendOK();											//Send success flag to USB Host
				break;
			
			case 0xf041:														//power off finger module
				
				PWROFF_1117;
				IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
				delay_ms(100);
				SendOK();											//Send success flag to USB Host
				break;
			
			case 0xf060:														//reset finger module(PMU & Touch IC)
				GPIO_ResetBits(GPIOE,RST);
				SendOK();												//Send success flag to USB Host
				break;
			
			case 0xf061:														//reset finger module(PMU & Touch IC) finish
				GPIO_SetBits(GPIOE,RST);
				SendOK();												//Send success flag to USB Host
				break;
			
			case 0xf050:														//read interrupt signal
					
				temp = Read_PMU(0x44);
				Sdata[0]=(uint8_t)(temp>>8);
				Sdata[1]=(uint8_t)temp;
				Sdata[2]=0xff;
				Sdata[3]=0xff;
				while(USB_StatusDataSended==0);
				USB_StatusDataSended = 0;
				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);		
				break;
			
			case 0xf051:
				//Read_PMU(0x01);
				if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					temp = Read_PMU(USB_Rx_Buffer[2]);
					Sdata[0]=(uint8_t)(temp>>8);
					Sdata[1]=(uint8_t)temp;
					Sdata[2]=0xff;
					Sdata[3]=0xff;
					while(USB_StatusDataSended==0);
					USB_StatusDataSended = 0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,PackSize);
				}
				else
				{SendError();}
				break;
			
			case 0xfcff:													//Instructure of PMU sending one frame data from its ram by spi interface
			
				SendImageData();
//				for(num=0;num<70;num++)
//			   {
//					SendOK();
//				}
				
				break;
			
			default :
				
				if((USB_Rx_Buffer[2] == 0xff) && (USB_Rx_Buffer[3] == 0xff))
				{
					Write_PMU(CMD->Byte[1],CMD->Byte[0]);
					SendOK();
				}
				else if((USB_Rx_Buffer[4] == 0xff) && (USB_Rx_Buffer[5] == 0xff))
				{
					Write_PMU(CMD->Byte[1],CMD->Byte[0]);
					Write_PMU(USB_Rx_Buffer[2],USB_Rx_Buffer[3]);
					SendOK();
				}
				else
				{
					SendError();
				}
				
				break;
		}
	}
}

void MOD1_MOD2_Buffer_Set(uint8_t *VoltageSetDataBuf)
{
	uint16_t Vol_Buf,Vol_Dec;
	
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)Sdata,1024);//1024*4
	
//	VMOD1=(USB_Rx_Buffer[2]<<8)|USB_Rx_Buffer[3];

//	VMOD2=(USB_Rx_Buffer[4]<<8)|USB_Rx_Buffer[5];

//	BUF=(USB_Rx_Buffer[6]<<8)|USB_Rx_Buffer[7];
	
	Vol_Buf=(VoltageSetDataBuf[2]<<8)|VoltageSetDataBuf[3];
	
	if(VMOD1_Voltage_Set(Vol_Buf,&Vol_Dec))
	{
		Sdata[0x10]=VoltageSetDataBuf[3];
		Sdata[0x11]=VoltageSetDataBuf[2];
		Sdata[0x12]=Vol_Dec;
		Sdata[0x13]=Vol_Dec>>8;
		
		VMOD2=(VoltageSetDataBuf[4]<<8)|VoltageSetDataBuf[5];
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,VMOD2,1);//MOD2 off
		delay_ms(1);
		Sdata[0x14]=VoltageSetDataBuf[5];
		Sdata[0x15]=VoltageSetDataBuf[4];
		
		BUF=(VoltageSetDataBuf[6]<<8)|VoltageSetDataBuf[7];
		IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//BUF off
		delay_ms(1);
		Sdata[0x18]=VoltageSetDataBuf[7];
		Sdata[0x19]=VoltageSetDataBuf[6];
		
		STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)Sdata,1024);
		
		SendOK();
		
	}else{/*printf("V1");*/Voltage_Send_Error(VoltageSetDataBuf,1);}
}

void Voltage_Send_Error(uint8_t *ErrorBuf,uint8_t ErrorType)
{
	uint16_t Vol_Buf;
	
	ErrorBuf[0]=0xf0;
	ErrorBuf[1]=1;
	ErrorBuf[2]=0;
	ErrorBuf[3]=0;
	ErrorBuf[4]=0;
	ErrorBuf[5]=ErrorType;
	switch(ErrorType)
	{
		case 1:
			//Vol_Buf = Voltage_MOD1_DET()*125/100;
			Vol_Buf = Voltage_MOD1_DET()*125/100;
			break;
		case 2:
			//Vol_Buf = Voltage_MOD2_DET()*125/100;
			Vol_Buf = Voltage_MOD2_DET()*125/100;
			break;
		case 3:
			//Vol_Buf = (Get_Adc_Value(&hadc1,ADC_BUFFER_CHANNEL) * 275) >> 8;
			//Vol_Buf = (Get_Buffer_Adc_Value() * 275) >> 8;
			break;
	}
	
	ErrorBuf[6]=Vol_Buf;
	ErrorBuf[7]=Vol_Buf>>8;
	ErrorBuf[8]=0xff;
	ErrorBuf[9]=0xff;
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
}



void Power_MOD1_MOD2_Calibration(uint8_t *CalibrationBuf)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x21,0x1f,1);
	delay_ms(120);
	
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)Sdata,1024);//1024*4
	//IIC2ReadInt16(PMOD1_DET_addr,VSHUNT_ADDR);IIC2ReadInt16(PMOD2_DET_addr,VSHUNT_ADDR);
	printf("%d\r\n",Current_MOD1_DET());
	printf("%d\r\n",Current_MOD2_DET()*5);

	if(CalibrationBuf[2]==1)//VMOD1 VMOD2强制校正
	{
		VMOD1_Calibration(&Sdata[0x1c]);//RX
		VMOD2_Calibration(&Sdata[0x28]);//RX
		CalibrationBuf[4]=1;
		CalibrationBuf[5]=0;
	}
	else
	{
		if(Sdata[0x1e]!=1)
		{
			VMOD1_Calibration(&Sdata[0x1c]);//RX
			VMOD2_Calibration(&Sdata[0x28]);//RX
			CalibrationBuf[4]=0;
			CalibrationBuf[5]=1;
//			printf("RX1=0x%x%x,R101=0x%x%x\r\n",DataMemry[0x35],DataMemry[0x34],DataMemry[0x39],DataMemry[0x38]);
//			printf("RX2=0x%x%x,R102=0x%x%x\r\n",DataMemry[0x45],DataMemry[0x44],DataMemry[0x49],DataMemry[0x48]);
		}
		else if(Sdata[0x1e]==1)
		{
			printf("Use old Calibration data.\r\n");
//			printf("RX1=0x%x%x,R101=0x%x%x\r\n",DataMemry[0x35],DataMemry[0x34],DataMemry[0x39],DataMemry[0x38]);
//			printf("RX2=0x%x%x,R102=0x%x%x\r\n",DataMemry[0x45],DataMemry[0x44],DataMemry[0x49],DataMemry[0x48]);
			//CalibrationBuf[1]=1;
		}
	}	
	
	if((CalibrationBuf[4]==1)||(CalibrationBuf[5]==1))//calibration flag set
	{
		Sdata[0x1e]=1;
		Sdata[0x2a]=1;
		STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)Sdata,1024);
	}
	
//	CalibrationBuf[0]=0xf0;
//	CalibrationBuf[1]=0;
//	CalibrationBuf[2]=0xff;
//	CalibrationBuf[3]=0xff;
	
	SendOK();
	
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x20,0x1f,1);
}



void Key_Detect(uint8_t *KeyDetectBuf)
{
	uint8_t KEY;
	KEY = KEY_Scan(0);
	
	KeyDetectBuf[0]=0xf0;
	KeyDetectBuf[1]=0;
	KeyDetectBuf[3]=0xff;
	KeyDetectBuf[4]=0xff;
	
	if(KEY == KEY3_PRESS)
	{
		KeyDetectBuf[2]=0;
	}
	else
	{
		KeyDetectBuf[2]=1;
	}
	
	USB_Send(KeyDetectBuf,64);
}


void Capture_Full_Image(uint8_t *CaptureFullImageBuf)
{
	uint8_t LineTimePara[2],u8Res,MKImagePart[2];
	uint8_t CountI,SensorTpye,Packet25Knum/*,Packet4Knum*/;
	uint32_t ImageLength;
	
	//ImageLength = ((CaptureFullImageBuf[5]<<8|CaptureFullImageBuf[4])*(CaptureFullImageBuf[7]<<8|CaptureFullImageBuf[6]))<<1;
	ImageLength = CaptureFullImageBuf[7]<<24|CaptureFullImageBuf[6]<<16|CaptureFullImageBuf[5]<<8|CaptureFullImageBuf[4];
	if(ImageLength > 125*1024){ImageLength = 125*1024;}
	CountI = (ImageLength+25600-1)/25600+1;
	//printf("CountI=%d\r\n",CountI);
	if((CaptureFullImageBuf[9] != 0)||(CaptureFullImageBuf[8] != 0))
	{
		LineTimePara[0]=CaptureFullImageBuf[9];//注意大端模式
		LineTimePara[1]=CaptureFullImageBuf[8];
		
		sDev_SPI_SendCmd(EN_SPI_CMD_SET_INTEGRAL_LINE,2,LineTimePara);//设置line time
		delay_ms(5);
		if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 1000,1) == 0)
		{
			CaptureFullImageBuf[0]=0xf0;//write error
			CaptureFullImageBuf[1]=1;//write error
			CaptureFullImageBuf[2]=0xff;
			CaptureFullImageBuf[3]=0xff;
			printf("Ready for linetime error!\r\n");
			return ;
		}
	}
	//else{printf("Frame Line Set ok.\r\n");}
	
	if(CaptureFullImageBuf[11]==1)
	{
		Module_Cur_Detec_EN = 1;
	}
	
	SensorTpye = CaptureFullImageBuf[2];
	if(SensorTpye==1)//MK210
	{
		sDev_SPI_SendCmd(EN_SPI_CMD_CAPTURE_MK_IMAGE,0,NULL);
	}
	else
	{
		MKImagePart[0]=0;
		MKImagePart[1]=CaptureFullImageBuf[3];
		sDev_SPI_SendCmd(EN_SPI_CMD_CAPTURE_MK_IMAGE,2,MKImagePart);
	}
	
	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 1500,1) == 0)
	{
		CaptureFullImageBuf[0]=0xf0;//write error
		CaptureFullImageBuf[1]=1;//write error
		CaptureFullImageBuf[2]=0xff;
		CaptureFullImageBuf[3]=0xff;
		printf("Ready for cmd error2!\r\n");
		return ;
	}
	else
	{
		CaptureFullImageBuf[0]=0xf0;//write ok
		CaptureFullImageBuf[1]=0;
		
		CaptureFullImageBuf[2]=ModuleCurrent;
		CaptureFullImageBuf[3]=ModuleCurrent>>8;
		CaptureFullImageBuf[4]=ModuleCurrent>>16;
		CaptureFullImageBuf[5]=ModuleCurrent>>24;
		
		CaptureFullImageBuf[6]=0xff;
		CaptureFullImageBuf[7]=0xff;
		
		
	}
	
	/*****************************25K*******************/
	
	for(Packet25Knum=1;Packet25Knum < CountI;Packet25Knum++)
	{
		if(!Receive_Packet_Head(0))
		{
			//Packet4Knum=1;
			//Packet25Knum=1;
			SendError();
			break;
		}
		
		if(Packet25Knum==1)
		{USB_Send(CaptureFullImageBuf,64);}
		else
		{SendOK();}
		
		CS0_Low;
		MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
		while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);	
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);	
		CS0_High;
		
		USB_Send(Sdata,25600);
		
		if(!Receive_Packet_End())
		{
			//Packet4Knum=1;
			//Packet25Knum=1;
			SendError();
			break;
		}
	}
	
	
	/**********************4K***********************/
//	for(Packet25Knum=1;Packet25Knum < iCount;Packet25Knum++)
//	{
//		if(!Receive_Packet_Head(0))
//		{
//			Packet4Knum=1;
//			Packet25Knum=1;
//			//SendError();
//			break;
//		}
//		
//		if((Packet25Knum > 1)&&(Packet25Knum%4 == 1)&&(Packet4Knum == 8))
//		{
//			USB_Send(Sdata,4096);
//		}
//				
//		for(Packet4Knum=1;Packet4Knum < 8;Packet4Knum++)
//		{
//			switch(Packet4Knum)
//			{
//				case 1:
//					if(Packet25Knum%4 == 1){SPI_Receive_Data_DMA(Sdata,4096,4096,1,0);}//1 or 5
//					else if(Packet25Knum%4 == 2){SPI_Receive_Data_DMA(Sdata+1024,3072,4096,1,0);}//2
//					else if(Packet25Knum%4 == 3){SPI_Receive_Data_DMA(Sdata+2048,2048,4096,1,0);}//3
//					else if(Packet25Knum%4 == 0){SPI_Receive_Data_DMA(Sdata+3072,1024,4096,1,0);}//4
//					//else if(Packet25Knum%4 == 1){SPI_Receive_Data_DMA_ReStart(Sdata+4096,4096);}
//					//CtrlSendBuf = DataBufDMA;
//					USB_Send(Sdata,4096);
//					break;
//				case 6:
//					if(Packet25Knum%4 == 1){SPI_Receive_Data_DMA(Sdata,1024,0,0,0);}
//					else if(Packet25Knum%4 == 2){SPI_Receive_Data_DMA(Sdata,2048,0,0,0);}
//					else if(Packet25Knum%4 == 3){SPI_Receive_Data_DMA(Sdata,3072,0,0,0);}
//					else if(Packet25Knum%4 == 0){SPI_Receive_Data_DMA(Sdata,4096,0,0,0);}
//					//else if(Packet25Knum%4 == 1){SPI_Receive_Data_DMA(Sdata,4096,0,0,0);}
//					//CtrlSendBuf = DataBufDMA+4096;
//					USB_Send(Sdata+4096,4096);
//					break;
//				case 7:
//					SPI_DMA_Wait();
//					//if(((SenType == 1)&&(Packet25Knum == 4))||(Packet25Knum == 5))//MK210 or last package
//					//if(Packet25Knum == (iCount-1)){UsbCtrlSendFlag=0;}
//					//CtrlSendBuf = DataBufDMA;
//					break;
//				default :
//					if((Packet4Knum%2)==0)
//					{
//						SPI_Receive_Data_DMA(Sdata,4096,0,0,0);
//						//CtrlSendBuf = DataBufDMA+4096;
//						USB_Send(Sdata+4096,4096);
//					}
//					else
//					{
//						SPI_Receive_Data_DMA(Sdata+4096,4096,0,0,0);
//						//CtrlSendBuf = DataBufDMA;
//						USB_Send(Sdata,4096);
//					}
//					break;
//			}
//		}	
//		
//		if(!Receive_Packet_End())
//		{
//			Packet4Knum=1;
//			Packet25Knum=1;
//			break;
//		}
//	}
}


void Capture_Win_Image(uint8_t *CaptureWinImageBuf)
{
	uint8_t LineTimePara[2],u8Res,MKWinImagePara[9];
	uint8_t CountI,/*SensorTpye,*/Packet25Knum/*,Packet4Knum*/;
	uint32_t ImageLength;
	
	ImageLength = CaptureWinImageBuf[7]<<24|CaptureWinImageBuf[6]<<16|CaptureWinImageBuf[5]<<8|CaptureWinImageBuf[4];
	if(ImageLength > 125*1024){ImageLength = 125*1024;}
	CountI = (ImageLength+25600-1)/25600+1;
	//printf("CountI=%d\r\n",CountI);
	if((CaptureWinImageBuf[9] != 0)||(CaptureWinImageBuf[8] != 0))
	{
		LineTimePara[0]=CaptureWinImageBuf[9];//注意大端模式
		LineTimePara[1]=CaptureWinImageBuf[8];
		
		sDev_SPI_SendCmd(EN_SPI_CMD_SET_INTEGRAL_LINE,2,LineTimePara);//设置line time
		delay_ms(5);
		if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 1000,1) == 0)
		{
//			CaptureWinImageBuf[0]=0xf0;//write error
//			CaptureWinImageBuf[1]=1;//write error
//			CaptureWinImageBuf[2]=0xff;
//			CaptureWinImageBuf[3]=0xff;
			SendError();
			printf("Ready for linetime error!\r\n");
			return ;
		}
	}
	//else{printf("Frame Line Set ok.\r\n");}
	
	MKWinImagePara[0]=CaptureWinImageBuf[2];
	
	MKWinImagePara[1]=CaptureWinImageBuf[10];
	MKWinImagePara[2]=CaptureWinImageBuf[11];
	MKWinImagePara[3]=CaptureWinImageBuf[12];
	MKWinImagePara[4]=CaptureWinImageBuf[13];
	
	if(MKWinImagePara[0]==2)
	{
		MKWinImagePara[5]=CaptureWinImageBuf[14];
		MKWinImagePara[6]=CaptureWinImageBuf[15];
		MKWinImagePara[7]=CaptureWinImageBuf[16];
		MKWinImagePara[8]=CaptureWinImageBuf[17];
	}	
	
	if(CaptureWinImageBuf[19]==1)
	{
		Module_Cur_Detec_EN = 1;
	}
	
	//SensorTpye = CaptureWinImageBuf[3];
	
	sDev_SPI_SendCmd(EN_SPI_CMD_CAPTURE_WIN_IMAGE_SIMPLIFY,9,MKWinImagePara);
	delay_ms(1);
	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 1500,1) == 0)
	{
		SendError();
		printf("Ready for cmd error2!\r\n");
		return ;
	}
	else
	{
		CaptureWinImageBuf[0]=0xf0;//write ok
		CaptureWinImageBuf[1]=0;
		
		CaptureWinImageBuf[2]=ModuleCurrent;
		CaptureWinImageBuf[3]=ModuleCurrent>>8;
		CaptureWinImageBuf[4]=ModuleCurrent>>16;
		CaptureWinImageBuf[5]=ModuleCurrent>>24;
		
		CaptureWinImageBuf[6]=0xff;
		CaptureWinImageBuf[7]=0xff;
		
		
	}
	
	
	/*****************************25K*******************/
	
	for(Packet25Knum=1;Packet25Knum < CountI;Packet25Knum++)
	{
		if(!Receive_Packet_Head(0))
		{
			//Packet4Knum=1;
			//Packet25Knum=1;
			SendError();
			break;
		}
		
		if(Packet25Knum==1)
		{USB_Send(CaptureWinImageBuf,64);}
		else
		{SendOK();}
		
		CS0_Low;
		MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
		while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);	
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);	
		CS0_High;
		
		USB_Send(Sdata,25600);
		
		if(!Receive_Packet_End())
		{
			//Packet4Knum=1;
			//Packet25Knum=1;
			SendError();
			break;
		}
	}
}


void Write_Module_Gain(uint8_t *WriteGainBuf)
{
	uint32_t CodeLength,CRC16Code,Packet4Kcount,CountI;
	uint16_t CRCBaseValue;
	uint8_t u8Res,num,GainNum,CRCInfor[9];
	
	CodeLength = WriteGainBuf[0xd]<<24|WriteGainBuf[0xc]<<16|WriteGainBuf[0xb]<<8|WriteGainBuf[0xa];
	CRC16Code = WriteGainBuf[9]<<24|WriteGainBuf[8]<<16|WriteGainBuf[7]<<8|WriteGainBuf[6];
	CRCBaseValue = WriteGainBuf[5]<<8|WriteGainBuf[4];
	GainNum = WriteGainBuf[3];

	printf("GainLength=0x%x  CRC16Gain=0x%x CRCBaseValue=0x%x GainNum=0x%x\r\n",CodeLength,CRC16Code,CRCBaseValue,GainNum);
	
	CRCInfor[0] = GainNum;//为后面CRC校验做准备
	
	CRCInfor[1] = CodeLength>>24;//大端模式
	CRCInfor[2] = CodeLength>>16;
	CRCInfor[3] = CodeLength>>8;
	CRCInfor[4] = CodeLength;
	
	CRCInfor[5] = CRC16Code>>8;//大端模式
	CRCInfor[6] = CRC16Code;
	
	CRCInfor[7] = CRCBaseValue>>8;//大端模式
	CRCInfor[8] = CRCBaseValue;
	
	Sdata[0]=0;
	Sdata[1]=WriteGainBuf[3];
	
	Sdata[2] = CodeLength>>24;//大端模式
	Sdata[3] = CodeLength>>16;
	Sdata[4] = CodeLength>>8;
	Sdata[5] = CodeLength;
	
	//sDev_SPI_SendCmd(EN_SPI_CMD_WRITE_GAIN,6,Sdata);
	sDev_SPI_SendCmd(EN_SPI_CMD_WRITE_GAIN_FOR_TEST_BOARD,6,Sdata);
	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 1000,1)==0)
	{
		SendError();
		return ;
	}
	if(u8Res != 0)
	{
		printf("result error :%x cmd = %x\r\n",u8Res,EN_SPI_CMD_WRITE_GAIN_FOR_TEST_BOARD);
		SendError();
		return ;
	}
	Sdata[0]=0xEF;
	Sdata[1]=1;
	Sdata[2]=8;//最后一包标识，0x0f代表最后一包，其他情况为8
//	Sdata[3]=0;
//	Sdata[4]=0;
	Sdata[4101]=0xaa;
	Sdata[4102]=0;		
	
//	USB_Send(WriteGainBuf);
	SendOK();
	
	Packet4Kcount = (CodeLength+4095)/4096;
	for(CountI=0;CountI<Packet4Kcount;CountI++)
	{
		for(num=0;num<8;)//取4K
		{
			if(USB_ReceivedCount>0)
			{
				//printf("1");
				USB_ReceivedCount=0;
				//USB_Receive(&Sdata[5]+num*512,512);
				memcpy(&Sdata[5]+num*512,USB_Rx_Buffer,512);
				num++;
				if(num!=8)
				{SendOK();}
			}
		}
		
		if((Packet4Kcount-CountI)==1)
		{Sdata[2]=0xf;}
		
		Sdata[3]=(uint8_t)(CountI+1)>>8;//大端模式 包号码从1开始
		Sdata[4]=(uint8_t)(CountI+1);
		
		SPI_Transmit_Data(Sdata,4103);
		if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 600,1)==0)
		{
			SendError();
			return ;//write error
		}
		if(u8Res != 0)
		{
			printf("result error :%x write gain package num = %x\r\n",u8Res,Sdata[2]);
			SendError();
			return ;
		}
		
		IWDG_ReloadCounter();
		if((Packet4Kcount-CountI)!=1)
		{
			//USB_Send(WriteGainBuf);
			SendOK();
		}
		else//last package CRC16check
		{
			sDev_SPI_SendCmd(EN_SPI_CMD_WRITE_GAIN_INFOR,9,CRCInfor);
			delay_ms(2);
			if(sDev_SPI_WaitRes(&u8Res, 2, Sdata, 600,1)==0)
			{
				/*WriteGainBuf[0]=0xf0;*/WriteGainBuf[1]=1;
				WriteGainBuf[2]=u8Res;WriteGainBuf[3]=Sdata[0];WriteGainBuf[4]=Sdata[1];
				WriteGainBuf[5]=0xff;WriteGainBuf[6]=0xff;
				printf("CRC1=0x%x,CRC2=0x%x\r\n",Sdata[0],Sdata[1]);
			}
			if(u8Res != 0)
			{
				printf("result error :%x cmd = %x\r\n",u8Res,EN_SPI_CMD_WRITE_GAIN_INFOR);
				SendError();
				return ;
			}
			SendOK();
		}
	}
}

void Read_Module_Gain(uint8_t *ReadGainBuf)
{
	uint8_t GainNumPara[6],u8Res/*,SensorTpye*/;
	uint32_t GainSize;
	uint8_t CountI,Packet25Knum/*,Packet4Knum*/;
	
	GainNumPara[0] = 0;
	GainNumPara[1] = ReadGainBuf[2];

	//SensorTpye = ReadGainBuf[3];
	
	sDev_SPI_SendCmd(EN_SPI_CMD_READ_GAIN_INFOR,1,GainNumPara+1);
	if(sDev_SPI_WaitRes(&u8Res, 8, Sdata, 1000,1)==0)
	{
		SendError();
		return ;
	}
	
	ReadGainBuf[0] = 0xf0;//大端转小端
	ReadGainBuf[1] = 0;
	
	ReadGainBuf[2] = Sdata[7];//大端转小端
	ReadGainBuf[3] = Sdata[6];
	
	ReadGainBuf[4] = Sdata[1];
	ReadGainBuf[5] = Sdata[0];
	ReadGainBuf[6] = 0;
	ReadGainBuf[7] = 0;
	
	ReadGainBuf[8] = Sdata[5];
	ReadGainBuf[9] = Sdata[4];
	ReadGainBuf[10] = Sdata[3];
	ReadGainBuf[11] = Sdata[2];
	
	GainSize = (uint32_t)(Sdata[2]<<24|Sdata[3]<<16|Sdata[4]<<8|Sdata[5]);
	CountI = (GainSize+25600-1)/25600+1;
	//printf("%d %d\r\n",GainSize,CountI);
	
	delay_ms(1);
	
	GainNumPara[2] = Sdata[2];
	GainNumPara[3] = Sdata[3];
	GainNumPara[4] = Sdata[4];
	GainNumPara[5] = Sdata[5];
	
	sDev_SPI_SendCmd(EN_SPI_CMD_READ_GAIN_FOR_TEST_BOARD,6,GainNumPara);
	delay_ms(1);
	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 1000,1)==0)
	{
		SendError();
		return ;
	}
	
	USB_Send(ReadGainBuf,64);
//	ReadGainBuf[0] = 0xf0;//大端转小端
//	ReadGainBuf[1] = 0;
	ReadGainBuf[2] = 0xff;//大端转小端
	ReadGainBuf[3] = 0xff;
	
	/*****************************25K*******************/
	
	for(Packet25Knum=1;Packet25Knum < CountI;Packet25Knum++)
	{
		if(!Receive_Packet_Head(0))
		{
			//Packet4Knum=1;
			//Packet25Knum=1;
			//SendError();
			break;
		}
		if(Packet25Knum!=1)
		{USB_Send(ReadGainBuf,64);}
		
		CS0_Low;
		MYDMA_Enable(DMA2_Stream2,25600,(uint32_t)Sdata);
		while( DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET);
		while( DMA_GetFlagStatus(DMA2_Stream2, DMA_FLAG_TCIF2) == RESET);
		DMA_Cmd(DMA2_Stream2, DISABLE);
		DMA_Cmd(DMA2_Stream3, DISABLE);	
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);	
		CS0_High;
		
		USB_Send(Sdata,25600);
		
		if(!Receive_Packet_End())
		{
			//Packet4Knum=1;
			//Packet25Knum=1;
			break;
		}
	}

}


void Normal_Current_Detect(uint8_t *NormalCurrentBuf)
{
	uint32_t Cur_Buf;
	
	I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,CURRENT_ADDR);
	Cur_Buf = I2C_ReadTwoByte(Open_I2Cx,PMOD1_DET_addr,CURRENT_ADDR)*10;
	
	NormalCurrentBuf[0] = 0xf0;
	NormalCurrentBuf[1] = 0;
	NormalCurrentBuf[2] = Cur_Buf;
	NormalCurrentBuf[3] = Cur_Buf>>8;
	NormalCurrentBuf[4] = Cur_Buf>>16;
	NormalCurrentBuf[5] = Cur_Buf>>24;
	NormalCurrentBuf[6] = 0xff;
	NormalCurrentBuf[7] = 0xff;
	
	USB_Send(NormalCurrentBuf,64);
}


void Module_SW_Vcom(uint8_t *VcomSwitchBuf)
{
	uint8_t VcomPara,u8Res;
	
	VcomSwitchBuf[0] = 0xf0;
	VcomSwitchBuf[1] = 0;
	
	VcomPara = VcomSwitchBuf[2];
	
	sDev_SPI_SendCmd(EN_SPI_CMD_MODULE_VCOM_SW,1,&VcomPara);
	delay_ms(40);
	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 500,1)==0)
	{
		VcomSwitchBuf[1] = 1;//error
		VcomSwitchBuf[4] = VcomSwitchBuf[2];//error
	}

	VcomSwitchBuf[2] = 0xff;
	VcomSwitchBuf[3] = 0xff;
	
	USB_Send(VcomSwitchBuf,64);
}


void ClearFrameSet(uint8_t *ClearFrameBuf)
{
	uint8_t u8Res;
	
	ClearFrameBuf[0] = 0xf0;
	ClearFrameBuf[1] = 0;
	
	sDev_SPI_SendCmd(EN_SPI_CMD_WRITE_CLEAR_FRAME_SET,31,ClearFrameBuf+2);
	delay_ms(5);
	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 500,1)==0)
	{
		ClearFrameBuf[1] = 1;//error
	}

	ClearFrameBuf[2] = 0xff;
	ClearFrameBuf[3] = 0xff;
	
	USB_Send(ClearFrameBuf,64);
}



void ReadClearFrameSetting(uint8_t *ReadClearFrameBuf)
{
	uint8_t u8Res/*,i*/;
	
	ReadClearFrameBuf[0] = 0xf0;
	ReadClearFrameBuf[1] = 0;
	
//	sDev_SPI_SendCmd(EN_SPI_CMD_READ_CLEAR_FRAME_SET,31,ClearFrameBuf+2);
//	delay_ms(5);
//	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 500,1)==0)
//	{
//		ClearFrameBuf[1] = 1;//error
//	}
	sDev_SPI_SendCmd(EN_SPI_CMD_RAED_CLEAR_FRAME_SET,0,NULL);
	delay_ms(1);
	if(sDev_SPI_WaitRes(&u8Res, 31, ReadClearFrameBuf+2, 1000,1)==0)
	{
		/*ReadClearFrameBuf[0]=0xf0;*/ReadClearFrameBuf[1]=1;
		ReadClearFrameBuf[2]=0xff;ReadClearFrameBuf[3]=0xff;
	}
//	for(i=0;i<38;i++)
//	{
//		printf("0x%x ",ReadClearFrameBuf[i]);
//	}
	
	USB_Send(ReadClearFrameBuf,64);
}



void ReadFrameLineSetting(uint8_t *ReadFrameLineBuf)
{
	uint8_t u8Res;
	
	ReadFrameLineBuf[0] = 0xf0;
	ReadFrameLineBuf[1] = 0;
	
	sDev_SPI_SendCmd(EN_SPI_CMD_RAED_FRAME_LINE_SET,0,NULL);
	delay_ms(1);
	if(sDev_SPI_WaitRes(&u8Res, 8, ReadFrameLineBuf+2, 1000,1)==0)
	{
		/*ReadClearFrameBuf[0]=0xf0;*/ReadFrameLineBuf[1]=1;
		ReadFrameLineBuf[2]=0xff;ReadFrameLineBuf[3]=0xff;
	}
	swap_u16_data((uint16_t*)(ReadFrameLineBuf+2),4);
	
	USB_Send(ReadFrameLineBuf,64);
}



void ModlueHardwareSetting(uint8_t *ModuleHardwareBuf)
{
	uint8_t u8Res,HardwarePara;
	
	ModuleHardwareBuf[0] = 0xf0;
	ModuleHardwareBuf[1] = 0;
	
	HardwarePara = ModuleHardwareBuf[2];
	sDev_SPI_SendCmd(EN_SPI_CMD_MODULE_HARDWARE_SET,1,&HardwarePara);
	delay_ms(20);//delay time >= 20ms is ok
	if(sDev_SPI_WaitRes(&u8Res, 0, NULL, 1000,1)==0)
	{
		/*ModuleHardwareBuf[0]=0xf0;*/ModuleHardwareBuf[1]=1;
	}
	ModuleHardwareBuf[2]=0xff;
	
	USB_Send(ModuleHardwareBuf,64);
}


uint8_t WakeUpProcess(void)
{
	uint16_t Timer=500;
	
	Read_PMU(0x01);
	Write_PMU(0xf1,WakeUpReg);
	while((--Timer)&&(Read_PMU(0x44)) != 0x8002){delay_ms(1);}
	if(Timer == 0)
	{
		TimeOutProcess();
		SendErrorCode(1,Read_PMU(0x44));
		return 0;
	}

	Write_PMU(0x9d,Reg9d2);
	Write_PMU(0x9e,Reg9e2);
	Write_PMU(0xf6,0xff);
	delay_ms(1);
	return 1;
}
uint8_t SleepProcess(void)
{
	uint16_t Timer=200;
	
	Write_PMU(0xf8,0xff);
	while((--Timer)&&(Read_PMU(0x44)) != 0x8002){delay_ms(1);}
	if(Timer==0)
	{
		TimeOutProcess();
		SendErrorCode(3,Read_PMU(0x44));
		return 0;
	}
	Write_PMU(0xf2,0xff);				//sleep
	//delay_ms(3);
	Write_PMU(0x9d,Reg9d1);
	Write_PMU(0x9e,Reg9e1);
	return 1;
}
void TimeOutProcess(void)
{
	printf(" %x \r\n",Read_PMU(0x01));
	printf(" %x %x %x %x \r\n",Read_PMU(0x46),Read_PMU(0x47),Read_PMU(0x48),Read_PMU(0x49));
	Write_PMU(0xf8,0xff);//restart
	Write_PMU(0xf2,0xff);//sleep
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	USB_OTG_FlushTxFifo(&USB_OTG_dev, 0);
}
uint8_t CodeDataCopy(uint8_t *data,uint32_t length)
{
	static uint16_t i=0;
	uint16_t j;
	for(;i<1500;)
	{
		//printf("%d\r\n",i);
		for(j=0;j<512;j++)
		{
			data[j] = USB_Rx_Buffer[j];
			
			if((i*512+j)==length)
			{
				STMFLASH_Write(APP_DEFAULT_ADD+i*512,(uint32_t*)data,j+1);
				i=0;
				return 1;
			}
		}
		STMFLASH_Write(APP_DEFAULT_ADD+i*512,(uint32_t*)data,512);
		i++;
		//printf(" %d \r\n",TT++);
		SendOK();
		return 0;
	}
	return 1;
}



uint16_t CRC16(uint8_t *Buf, uint32_t BufLen, uint16_t CRCCode)
{
	uint32_t i;

	for(i = 0 ; i < BufLen; i++)
	{
		CRCCode = (CRCCode << 8) ^ CrcCCITTTable[((CRCCode >> 8) ^ *Buf++) & 0x00FF];
	}
	return CRCCode;
}




void swap_u16_data(uint16_t *ptr,uint32_t size)
{
	uint32_t i;
	uint16_t *p = ptr;
	 
	for ( i = 0; i < size; i++)
	{
		*p = (((*p)>>8)&0x00ff)|(((*p)&0xff)<<8);
		p++;
   }
}





/*********************end*****************/





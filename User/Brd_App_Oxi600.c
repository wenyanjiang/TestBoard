
#include "brd_app_oxi600.h"
#include "stm32f4xx_hal.h"
//#include "dev_oxi600_driver.h"
#include "bsp.h"
uint16_t g_u16integraLines = 400;
extern  SPI_HandleTypeDef hspi1;

uint32_t g_u32XaoDelay = 0;
void delayMs(uint32_t n)
{
	if( n!= 0)
	HAL_Delay(n-1);
}


static int _brd_Oxi600_SpiWrite(uint8_t *dataBuf,uint32_t size)
{		
	return Dev_SPI_SendData(&hspi1, dataBuf,size,200);
}


static int _brd_Oxi600_SpiRead(uint8_t *dataBuf,uint32_t size)
{
	return 	Dev_SPI_ReceiveData(&hspi1, dataBuf,size,200);
}

static int _brd_Oxi600_SPI_ReceiveMass(uint8_t *dataBuf,uint32_t dataLen)
{
 	return Dev_SPI_ReceiveDataDMA(&hspi1,dataBuf,dataLen);
}

static int _brd_Oxi600_SPI_SendMass(uint8_t *dataBuf,uint32_t dataLen)
{
 	return Dev_SPI_SendDataContinus(&hspi1,dataBuf,dataLen);
}


void Brd_Oxi600_DrvInit(void)
{
	ST_CHNL600_EXTER_DRV stChnl600Drv;
	stChnl600Drv.delay_ms = delayMs;
	stChnl600Drv.getLocalTime = HAL_GetTick;
	stChnl600Drv.SPI_Receive = _brd_Oxi600_SpiRead;
	stChnl600Drv.SPI_Send = _brd_Oxi600_SpiWrite;
	stChnl600Drv.SPI_Receive_mass = _brd_Oxi600_SPI_ReceiveMass;
	stChnl600Drv.SPI_Send_mass = _brd_Oxi600_SPI_SendMass;
	
	Dev_Oxi600_DrvInit(&stChnl600Drv);
}


void Brd_Oxi600_SetIntegrationLine(uint16_t InteLIne)
{
	g_u16integraLines = InteLIne;
}

uint16_t Brd_Oxi600_GetIntergrationLine(void)
{
	return g_u16integraLines;
}



EN_OXI600_ERR_TYPE Brd_Oxi600_setPmuPara(ST_PRTCL_INFO rcvDataInfo, ST_PRTCL_INFO rcvCmdInfo, u32 addrOffset)
{
	EN_RESP_TYPE enRetVal = EN_RESP_SUCCESS;

	//memcpy(&g_u8Oxi600RegInitBuf[addrOffset],rcvDataInfo.pu8DataBufPointer,rcvDataInfo.u16ParaLen);
	Dev_Oxi600_setPmuPara(rcvDataInfo.pu8DataBufPointer,rcvDataInfo.u16ParaLen);

	return enRetVal;
}

EN_OXI600_ERR_TYPE Brd_Oxi600_setClrPara(ST_PRTCL_INFO rcvDataInfo, ST_PRTCL_INFO rcvCmdInfo, u32 addrOffset)
{
	EN_RESP_TYPE enRetVal = EN_RESP_SUCCESS;

	Dev_Oxi600_setCptPara(rcvDataInfo.pu8DataBufPointer,rcvDataInfo.u16ParaLen);
	if(rcvDataInfo.pu8DataBufPointer[0] == 2)
	{
		g_u32XaoDelay = 0;
	}
	else
	{
		g_u32XaoDelay =  ((uint32_t)rcvDataInfo.pu8DataBufPointer[25]<<24)|((uint32_t)rcvDataInfo.pu8DataBufPointer[26]<<16)|\
							((uint32_t)rcvDataInfo.pu8DataBufPointer[27]<<8)|((uint32_t)rcvDataInfo.pu8DataBufPointer[28]);
	}

	return enRetVal;
}


//Dev_Oxi600_CaptureWinImage(stChanl600Info,&g_pu8DataBuf[9],&u32ImgDataLen)
EN_OXI600_ERR_TYPE Brd_Oxi600_CaptureWinImg(ST_CHNL600_CPT_PARA stChnl600CptPara,uint8_t *dataBuf,uint32_t *datalen)
{
	stChnl600CptPara.integraLine = g_u16integraLines; 
	
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	
	enRetVal = Dev_Oxi600_ClrAndGetGateoff(stChnl600CptPara,dataBuf+stChnl600CptPara.CptDataSize,datalen,FALSE,1000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
	delayMs(g_u32XaoDelay);
	enRetVal = Dev_Oxi600_CaptureWinImage(stChnl600CptPara,1000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
	g_readCurrent = BSP_ReadCurrent(1);
	enRetVal = Dev_Oxi600_GetImageData(dataBuf,datalen,2000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
#if 0	
	enRetVal = Dev_Oxi600_CaptureWinImage(stChnl600CptPara,1000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
	enRetVal = Dev_Oxi600_GetImageData(dataBuf,datalen,2000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}

	//Dev_Oxi600_WinJigsaw(stChnl600CptPara.PrjType,dataBuf+stChnl600CptPara.CptDataSize,\
	//				stChnl600CptPara.W1ColStrt,dataBuf,datalen);

#endif	
	enRetVal = Dev_Oxi600_SleepROIC(stChnl600CptPara,1000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
	
	return enRetVal;
}


EN_OXI600_ERR_TYPE Brd_Oxi600_CaptureWhlImg(ST_CHNL600_CPT_PARA stChnl600CptPara,uint8_t *dataBuf,uint32_t *datalen)
{
	static u8 u8rowOfst = 0;
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	if(stChnl600CptPara.WhlImageNo == 1)
	{
		u8rowOfst = 0;
	}

	switch(stChnl600CptPara.PrjType)
	{
		case EN_PRJ_OXI600_MK720_80UM:
			stChnl600CptPara.W1RowNo = stChnl600CptPara.CptDataSize/(439*2);  
			stChnl600CptPara.W1RowStrt = u8rowOfst;
			u8rowOfst += stChnl600CptPara.W1RowNo;
			break;
			
		case EN_PRJ_OXI600_MK720_100UM:
			stChnl600CptPara.W1RowNo = stChnl600CptPara.CptDataSize/(369*2);
			stChnl600CptPara.W1RowStrt = u8rowOfst;
			u8rowOfst += stChnl600CptPara.W1RowNo;  
			break;

		case EN_PRJ_OXI600_MK810_80UM:
			stChnl600CptPara.W1RowNo = stChnl600CptPara.CptDataSize/(510*2);  
			stChnl600CptPara.W1RowStrt = u8rowOfst;
			u8rowOfst += stChnl600CptPara.W1RowNo;
			break;

		case EN_PRJ_OXI600_MK320_100UM:
			stChnl600CptPara.W1RowNo = stChnl600CptPara.CptDataSize/(493*2);  
			stChnl600CptPara.W1RowStrt = u8rowOfst;
			u8rowOfst += stChnl600CptPara.W1RowNo;
			break;

		case EN_PRJ_OXI600_MK810_80UM_04:
			stChnl600CptPara.W1RowNo = stChnl600CptPara.CptDataSize/(510*2);  
			stChnl600CptPara.W1RowStrt = u8rowOfst;
			u8rowOfst += stChnl600CptPara.W1RowNo;
			break;

			
		default :
			break;	
	}
	
	stChnl600CptPara.integraLine = g_u16integraLines; 

	enRetVal = Dev_Oxi600_ClrAndGetGateoff(stChnl600CptPara,dataBuf+stChnl600CptPara.CptDataSize,datalen,FALSE,1000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
	delayMs(g_u32XaoDelay);
	enRetVal = Dev_Oxi600_CaptureWholeRowImage(stChnl600CptPara,1000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
	g_readCurrent = BSP_ReadCurrent(1);
	enRetVal = Dev_Oxi600_GetImageData(dataBuf,datalen,2000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
#if 0	
	enRetVal = Dev_Oxi600_CaptureWinImage(stChnl600CptPara,1000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
	enRetVal = Dev_Oxi600_GetImageData(dataBuf,datalen,2000);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}

	//Dev_Oxi600_WinJigsaw(stChnl600CptPara.PrjType,dataBuf+stChnl600CptPara.CptDataSize,\
	//				stChnl600CptPara.W1ColStrt,dataBuf,datalen);

#endif	
	//enRetVal = Dev_Oxi600_SleepROIC(1000);
	//if(enRetVal != EN_OXI600_SUCCESS )
	//{
	//	return enRetVal;
	//}
	
	return enRetVal;
}




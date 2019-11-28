#include "lib_cmos.h"

////////////////////////////////////

#define DEBUG_DEV_CMOS	1
#ifdef GET_CURRENT	
	extern volatile int g_readCurrent;
	extern int BSP_ReadCurrent(uint8_t chanel);
#endif


u32 g_u32IntTimeMs;
u8 g_u8CmosRegInitBuf[] = 
{
	
	0x80,0x90,0x00,
	0x81,0x01,0x00,
	0x82,0x00,0x00,
	0x83,0x90,0x00,
	0x84,0x01,0x00,
	0x85,0x00,0x00,
	0x86,0x5b,0x00,
	0x87,0x09,0x00,
	0x88,0x00,0x00,
	0x89,0x68,0x00,
	0x8a,0x01,0x00,
	0x8b,0x00,0x00,
	0x8c,0x05,0x00,
	0x8d,0x00,0x00,
	0x8e,0x05,0x00,
	0x8f,0x00,0x00,
	0x90,0xfe,0x00,
	0x91,0x00,0x00,
	0x92,0x05,0x00,
	0x93,0x00,0x00,
	0x94,0xaf,0x00,  //// 100ms--0x271--time /// 110ms
	0x95,0x02,0x00,  ////
	0x96,0x00,0x00,
	0x97,0x68,0x00,
	0x98,0x01,0x00,
	0x99,0x00,0x00,
	0x9a,0x00,0x00,
	0x9b,0x00,0x00,
	0x9c,0x24,0x00,
	0x9d,0x00,0x00,
	0x9e,0x05,0x00,
	0x9f,0x00,0x00,
	0xa0,0x1f,0x00,
	0xa1,0x00,0x00,
	0xa2,0x25,0x00,
	0xa3,0x00,0x00,
	0xa4,0x40,0x00,
	0xa5,0x00,0x00,
	0xa6,0x15,0x00,
	0xa7,0x00,0x00,
	0xa8,0x02,0x00,
	0xa9,0x00,0x00,
	0xaa,0x7d,0x00,
	0xab,0x00,0x00,
	0xac,0x40,0x00,
	0xad,0x00,0x00,
	0xae,0xbd,0x00,
	0xaf,0x00,0x00,
	0xb0,0x40,0x00,
	0xb1,0x00,0x00,
	0xb2,0x1a,0x00,
	0xb3,0x00,0x00,
	0xb4,0x02,0x00,
	0xb5,0x00,0x00,
	0xb6,0x1a,0x00,
	0xb7,0x01,0x00,
	0xb8,0x02,0x00,
	0xb9,0x00,0x00,
	0xba,0x05,0x00,
	0xbb,0x00,0x00,
	0xc1,0x00,0x00,
	0xc2,0x01,0x00,
	0xc3,0x00,0x00,
	0xc4,0x01,0x00,
	0xc5,0x00,0x00,
	0xc6,0x03,0x00,
	0xc7,0x05,0x00,
	0xc8,0x10,0x00,
	0xc9,0x00,0x00,
	0xca,0x05,0x00,
	0xcb,0x01,0x00,
	//0xcc,0x00,0x00,
	0xcd,0x00,0x00,
	0xce,0x3f,0x00,
	0xd7,0x80,0x00,
	0xd8,0x78,0x00,
	0xd9,0x42,0x00,
	0xda,0x7a,0x00,
	0xdb,0x00,0x00,
	//0xdc,0x00,0x00,
	//0xdd,0x00,0x00,
	0xde,0x10,0x00,
	0xdf,0x10,0x0a,//delay
	0xd0,0x45,0x00,
	0xd1,0x10,0x00,
	0xd2,0x20,0x00,
	0xd3,0x0f,0x00,
	0xd4,0xf8,0x00,
	//0xd5,0x00,0x00,
	0xd6,0x24,0x00,
	0xcf,0x05,0x0a, //wakeup
	0xff,0xff,0x00,
};

/***************************************************************************************************
** Subroutine  : Dev_SPI_SendData
** Function    : Dev_SPI_CmosWrtReg
** Input       : u8 u8RegAddr; u8 u8Value
** Output      : 0
** Description : 批量写寄存器
** Date        : 2019
** ModifyRecord:
***************************************************************************************************/
oxi_cmos_status_t Dev_SPI_CmosWrtReg(u8 u8RegAddr, u8 u8Value)
{
#if DEBUG_DEV_CMOS
	DBG("W %02X = %02X\r\n",u8RegAddr, u8Value);
#endif

	Dev_Oxi600_WriteReg(u8RegAddr, u8Value);
	return OXI_Success;
}

/***************************************************************************************************
** Subroutine  : Dev_SPI_SendData、Dev_SPI_ReceiveData
** Function    : Dev_SPI_CmosRdReg
** Input       : u8 *pu8RegAddr
** Output      : 0; pu8RegAddr 会写上读到的值
** Description : 批量读寄存器
** Date        : 2019
** ModifyRecord:
***************************************************************************************************/
oxi_cmos_status_t Dev_SPI_CmosRdReg(u8 *pu8RegAddr)
{
	uint16_t val = Dev_Oxi600_ReadReg(*pu8RegAddr);
	
#if DEBUG_DEV_CMOS	
	DBG("R %02X = %02X\r\n",(*pu8RegAddr)&0x7F,(val>>8)&0xff);
#endif
	*pu8RegAddr = (val>>8)&0xff;
	return OXI_Success;
}

/***************************************************************************************************
** Subroutine  : Dev_SPI_CmosRdReg
** Function    : Dev_SPI_CmosRecvRegVal
** Input       : u8 u8RegAddr, u16 u16RegValMask, u16 u16ChkRegVal, u32 u32TimeOver
** Output      : 0:成功；其他：失败
** Description : 等一个状态
** Date        : 2019
** ModifyRecord:
***************************************************************************************************/
oxi_cmos_status_t Dev_SPI_CmosRecvRegVal(u8 u8RegAddr, u16 u16RegValMask, u16 u16ChkRegVal, u32 u32TimeOver)
{
	volatile u8 vu8RcvRegVal;
	volatile u32 vu32RecordTime = 0;
	int ret;
	u8 u8DestRegVal = (u16ChkRegVal>>8)&0xff;
	u8 u8RegValMask = (u16RegValMask>>8)&0xff;

#if DEBUG_DEV_CMOS	
	DBG("S %02X %02X %02X\r\n",u8RegAddr, u8RegValMask, u8DestRegVal);
#endif
	
	while(1)
	{
		vu8RcvRegVal = u8RegAddr;
		ret = Dev_SPI_CmosRdReg((u8*)&vu8RcvRegVal);
		//CHECK_RET_RETURN("Dev_SPI_CmosRdReg",ret,ret);
		
#ifdef GET_CURRENT
		if(u16ChkRegVal == 0x8B0B)//cmos 采图等完成
		{
			g_readCurrent = BSP_ReadCurrent(1);
#if DEBUG_DEV_CMOS
			DBG("cuurent = %d \n",g_readCurrent);
#endif
		}
#endif
		
		
	#if DEBUG_DEV_CMOS
		DBG("Reg Val:%02X\r\n",vu8RcvRegVal);
	#endif

		if((vu8RcvRegVal & u8RegValMask) == u8DestRegVal)
		{
			return OXI_Success;
		}
		
		vu32RecordTime++;
		if(vu32RecordTime%500 == 0)
		{
			IwdgFeed();
		}
		
		if(vu32RecordTime > u32TimeOver)
		{
		#if DEBUG_DEV_CMOS
			DBG("check reg %02X, mask %02X, val %02X failed\r\n" \
									,u8RegAddr,u8RegValMask,u8DestRegVal );
		#endif
			return OXI_Timeout;
		}
		delay_ms(1);
	}
	return OXI_Failed;
}


oxi_cmos_status_t Dev_Cmos_Regs_Update(void)
{
	u16 i;
	u16 u16Tmp;
	u8 u8Reg94Val,u8Reg95Val;
	int ret=0;

	for(i=0; i<4096; i +=3)
	{
		if((g_u8CmosRegInitBuf[i] == 0xff)&&(g_u8CmosRegInitBuf[i+1] == 0xff))
		{
		#if DEBUG_DEV_CMOS
			DBG("regster init finish\r\n");
		#endif
			break;
		}
		
		Dev_SPI_CmosWrtReg(g_u8CmosRegInitBuf[i], g_u8CmosRegInitBuf[i+1]);
		delay_ms(g_u8CmosRegInitBuf[i+2]);
		//DBG("[%2d] %2x %2x %2x \r\n",i/3,g_u8CmosRegInitBuf[i],g_u8CmosRegInitBuf[i+1],g_u8CmosRegInitBuf[i+2]);
	}	
	u8Reg94Val=0x94;
	u8Reg95Val=0x95;
	
	ret = Dev_SPI_CmosRdReg(&u8Reg94Val);
	//CHECK_RET_RETURN("Dev_SPI_CmosRdReg 94",ret,FALSE);
	
	ret = Dev_SPI_CmosRdReg(&u8Reg95Val);
	//CHECK_RET_RETURN("Dev_SPI_CmosRdReg 95",ret,FALSE);

#if DEBUG_DEV_CMOS
	DBG("reg 94:%02X, reg 95:%02X\r\n",u8Reg94Val,u8Reg95Val);
#endif
	u16Tmp = (u8Reg95Val<<8)|u8Reg94Val;
	DBG("tmp:%04X\r\n",u16Tmp);
	g_u32IntTimeMs = 80;//(u16Tmp>>3) + (u16Tmp>>5) - 10;
#if DEBUG_DEV_CMOS
	DBG("integral time:%d\r\n",g_u32IntTimeMs);
#endif 
	
	return OXI_Success;
}

oxi_cmos_status_t Dev_Cmos_WakeUp(void)
{
	int ret=0;
	/*wake up*/
	Dev_SPI_CmosWrtReg(0xCF, 0x05);/*it is set in init flow*/
	Dev_SPI_CmosWrtReg(0xF1, 0xFF);
	delay_ms(2);
	ret=Dev_SPI_CmosRecvRegVal(0x3F, 0x0400, 0x0400, 500);/*check wether wake up success */
	CHECK_RET_RETURN("Dev_SPI_CmosRecvRegVal err",ret,ret);
	
	return OXI_Success;
}


oxi_cmos_status_t Dev_Cmos_Restart(void)
{
	int ret=0;
	Dev_SPI_CmosWrtReg(0xF8, 0xFF);/*restart CMD*/
	delay_ms(2);

	ret=Dev_SPI_CmosRecvRegVal(0x3F, 0x0004, 0x0004, 500);/*check capture option success */
	CHECK_RET_RETURN("Dev_SPI_CmosRecvRegVal err",ret,ret);
	
	return OXI_Success;
}

oxi_cmos_status_t Dev_Cmos_GetChipID(u8* Buf)
{	
	u8 u8ChipId;
	int ret;
	u8ChipId = 0x3e;
	ret = Dev_SPI_CmosRdReg(&u8ChipId);
	//CHECK_RET_RETURN("Dev_SPI_CmosRdReg 3e",ret,FALSE);
	//SplitU16(Buf, u16ChipId);

	*Buf = u8ChipId;
	return OXI_Success;
}

/*
	aec =  v95 << 8 |  v94
	delay = (aec >> 3) + (aec >>5) - 15
*/
oxi_cmos_status_t Dev_Cmos_SetIntTime(u8* Buf)
{	
	u16 u16Tmp;
	u8 u8Reg94Val,u8Reg95Val;
	int ret=0;
	u8	u8TstReg94Val, u8TstReg95Val;
	u32 u32ReferVal,u32Fac,u32SetIntTime;/*intergral factor*/

	/*cal integral time referent value: (reg val02H~00H + reg05H~03H)*0.2 --unit: us*/
	u32ReferVal = (((u32)g_u8CmosRegInitBuf[4])<<8)|g_u8CmosRegInitBuf[1];
	u32ReferVal += (((u32)g_u8CmosRegInitBuf[13])<<8)|g_u8CmosRegInitBuf[10];
	u32ReferVal /= 5;/*unit us -- 160us*/
	u32ReferVal = 160;
	/*u32SetIntTime*1000(ms -> us) = referent vaule* reg 96H~94H*/
	u32SetIntTime = MergeU16(Buf);
#if DEBUG_DEV_CMOS
	DBG("set int time = %dMs\r\n",u32SetIntTime);
#endif
	u32SetIntTime *=1000;/*unit ms -> us*/
	u32Fac = u32SetIntTime/u32ReferVal;
	u8Reg95Val = (u8)(u32Fac>>8);
	u8Reg94Val = (u8)(u32Fac&0xFF);
#if DEBUG_DEV_CMOS
	DBG("cal reg94 val:%02X, reg95 val:%02X\r\n",u8Reg94Val,u8Reg95Val);
#endif

	ret = Dev_SPI_CmosWrtReg(0x94, u8Reg94Val);
	//CHECK_RET_RETURN("Dev_SPI_CmosRdReg 94",ret,FALSE);
	
	ret =Dev_SPI_CmosWrtReg(0x95, u8Reg95Val);
	//CHECK_RET_RETURN("Dev_SPI_CmosRdReg 95"
	
	u8TstReg94Val=0x94;
	u8TstReg95Val=0x95;
	
	ret = Dev_SPI_CmosRdReg(&u8TstReg94Val);
	//CHECK_RET_RETURN("Dev_SPI_CmosRdReg 94",ret,FALSE);
	
	ret = Dev_SPI_CmosRdReg(&u8TstReg95Val);
	//CHECK_RET_RETURN("Dev_SPI_CmosRdReg 95",ret,FALSE);

	if(((u8TstReg94Val) ==u8Reg94Val) && (u8TstReg95Val == u8Reg95Val) )
	{
		u16Tmp = (u8Reg95Val<<8)|u8Reg94Val;
		g_u32IntTimeMs = (u16Tmp>>3) + (u16Tmp>>5) - 15;
	#if DEBUG_DEV_CMOS
		DBG("register set success,int time:%d\r\n",g_u32IntTimeMs);
	#endif

		return OXI_Success;
	}
	else
	{
	#if DEBUG_DEV_CMOS
		DBG("set integral time failed %x -%x ; %x -%x\r\n",u8TstReg94Val,u8Reg94Val, u8TstReg95Val ,u8Reg95Val);
	#endif	
		return OXI_Failed;
	}
}

void SwapU16(u16 *ptr,int size)
{
     u32 i;
     u16 *p = ptr;
	 
     for ( i = 0; i < size; i++){
			*p = (((*p)>>8)&0x00ff)|(((*p)&0xff)<<8);
			p++;
   	 }
}

/*
	func describe	: Cmos capture image(without init) flow:
								step1. CMOS wake up;
								step2. send CMOS capture CMD ,then wait for 
									   capture complete of CMOS;
								step3. send get image data CMD ,then get image data;
								step4. sleep CMOS.
	input para 		: none
	output para		: image data(88x86x2, col: 88, row: 86)
	return 			: init result => TRUE -- capture success, FALSE -- capture failed
*/
oxi_cmos_status_t Dev_Cmos_Cpt(u8* ParaBuf)
{
	volatile u16 vu16RecvRegVal;
	int ret=0;
#if DEBUG_DEV_CMOS
	DBG("Cmos Capture\r\n");
#endif
	ret=Dev_Cmos_WakeUp();
	CHECK_RET_RETURN("Dev_Cmos_WakeUp err",ret,ret);
	
	IwdgFeed();

#if DEBUG_DEV_CMOS
	DBG("Cmos wake up success\r\n");
#endif

	/*send Capture image CMD*/
	Dev_SPI_CmosWrtReg(0xF3, 0xFF);
	delay_ms(g_u32IntTimeMs);
	Dev_SPI_CmosWrtReg(0xD2, 0x28);
	Dev_SPI_CmosWrtReg(0xD0, 0x40);
	Dev_SPI_CmosWrtReg(0xCF, 0x00);
	
	IwdgFeed();

	ret=Dev_SPI_CmosRecvRegVal(0x3F, 0xFFFF, 0x8B0B, 5000);/*check capture option success */
	CHECK_RET_RETURN("Dev_SPI_CmosRecvRegVal err",ret,ret);
	
#if DEBUG_DEV_CMOS
	DBG("cpt image success, image ready\r\n");
#endif
	/*Receive data*/
	Dev_SPI_CmosWrtReg(0xFC, 0xFF);
	Dev_SPI_CmosWrtReg(0xFF, 0xFF);
	Dev_Oxi600_SPI_ReceiveContinuous(ParaBuf,88*86*2);
	
#if DEBUG_DEV_CMOS
	DBG("Cmos send cpt image success\r\n");
#endif

	ret=Dev_Cmos_Restart();
	CHECK_RET_RETURN("Dev_Cmos_Restart err",ret,ret);
	
	ST_CHNL600_CPT_PARA tmp;
	ret=Dev_CMOS_SleepROIC( tmp,0);
	CHECK_RET_RETURN("Dev_CMOS_SleepROIC err",ret,ret);

	//SwapU16((u16* )(&ParaBuf[0]), 86*88);
	return OXI_Success;
}


//////////////////
/**
 * @brief  sleep ROIC disale ADC  
 * @param  timeout 
 * @retval EN_OXI600_ERR_TYPE
**/
oxi_cmos_status_t Dev_CMOS_SleepROIC(ST_CHNL600_CPT_PARA stChnl600CptPara,uint32_t timeout)
{
	Dev_SPI_CmosWrtReg(0xD0, 0x45);  ///reference closed
	Dev_SPI_CmosWrtReg(0xD2, 0x20);  ///ADC closed
	Dev_SPI_CmosWrtReg(0xCF, 0x27/*0x05*/);  ///ADC clock closed
	delay_ms(3);    

	Dev_SPI_CmosWrtReg(0xF2, 0xFF);/*cmd sleep*/
	delay_ms(2);
	return OXI_Success;
}

/**
 * @brief  set PMU parameter
 * @param   dataBuf the pointer to data buffer
 * @param   dataLen the pointer to data length, indicate effective data length
 * @retval EN_OXI600_ERR_TYPE
**/
oxi_cmos_status_t Dev_CMOS_setPmuPara(uint8_t *dataBuf, uint32_t dataLen)
{
	
#if DEBUG_DEV_CMOS
	DBG("Dev_CMOS_setPmuPara\r\n");
#endif

//	for(int i=0;i<dataLen;i+=3)
//	{
//		printf("w %2x=%2x %2x\r\n",dataBuf[i],dataBuf[i+1],dataBuf[i+2]);
//	}
	
	memcpy(g_u8CmosRegInitBuf,dataBuf,dataLen);
	return OXI_Success;
}

/**
  * @brief	start capture frame scan (Start running and reply success immediately)
  * @param	stChnl600CptPara  
  * @param	timeout 
  * @retval EN_OXI600_ERR_TYPE

  EXAMPLE:
  	 stChnl600CptPara.enCptMode = EN_CHNL600_CPT_MODE;
	 stChnl600CptPara.enCptType = EN_MODE1_CPT_WIN_IMG;
	 stChnl600CptPara.FingerNo = X;
	 stChnl600CptPara.integraLine = x;
	 stChnl600CptPara.shtOrLong = X;
	 stChnl600CptPara.PrjType = X;
	 stChnl600CptPara.W1ColStrt = X1;
	 stChnl600CptPara.W1RowStrt = Y1;
 **/
oxi_cmos_status_t Dev_CMOS_CaptureWinImage(ST_CHNL600_CPT_PARA stChnl600CptPara,uint32_t timeout)
{
	int ret=0;
	ret=Dev_CMOS_RoicRegInit();//仅为了编译通过
	CHECK_RET_RETURN("Dev_CMOS_RoicRegInit err",ret,ret);

	ret=Dev_Cmos_SetIntTime(NULL);//仅为了编译通过
	CHECK_RET_RETURN("Dev_Cmos_SetIntTime err",ret,ret);

	ret=Dev_Cmos_Cpt( NULL);//仅为了编译通过
	CHECK_RET_RETURN("Dev_Cmos_Cpt err",ret,ret);

	return OXI_Success;
}

/**
 * @brief All register writes corresponding data
 * @param none
 * @retval 1 or error number
 */
oxi_cmos_status_t Dev_CMOS_RoicRegInit(void)
{
	int ret=0;
	u8 u8ChipID;
	
//Dev_Sen_PowOn();
	Dev_Cmos_Reset(10);/*reset IC*/
//	u16ChipID = Dev_Cmos_GetChipID(NULL);
//#if DEBUG_DEV_CMOS
//	DBG("chip id:%04x\r\n",u16ChipID);
//#endif
	
	ret=Dev_Cmos_Regs_Update();/*init IC register*/
	CHECK_RET_RETURN("Dev_Cmos_Regs_Update",ret,ret);
	
	ret = Dev_Cmos_GetChipID(&u8ChipID);
	CHECK_RET_RETURN("Dev_Cmos_GetChipID",ret,ret);
	
#if DEBUG_DEV_CMOS
	DBG("chip id:%02x\r\n",u8ChipID);
#endif
	
	return OXI_Success;
}







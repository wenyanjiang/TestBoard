#include "lib_oxi600.h"

const uint8_t u8LibVer[4] = {19,0,0,2};	 /*year release_main_ver release_sub_ver debug_ver*/


/* 
 *log enable control, not include err log
 *该log为函数运行中的参数及状态便于调试，不包含代码中的错误log，错误log必打印
 */
//#define DEBUG_DEV_OXI600


/*
*	DBG api define , differnt platform has differnt implemented
*/
#if 0	//(OXI_CONFIG_PLATFORM == PLATFORM_QSEE && OXI_CONFIG_BUILD == BUILD_TEE)
	#if 1
	#include <string.h>
	#include <stdio.h>                        
	#include "qsee_log.h"    
	#define DBG(fmt,...)         qsee_printf("INFO: [%s] " fmt, __FUNCTION__,  ##__VA_ARGS__)   
	#define DBG_OPTM(fmt,...)		qsee_printf("%s(%d)-<%s>: "##fmt,__FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

	#elif LIBUSB_OPEN_FD
	#include <android/log.h>

	#define LOG_TAG                        "_OXIFP_LISTDEV_"
	#define DBG(...)            __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
	#define INFO(...)            __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
	#endif
#else 	//(stm32 pc)

#ifndef   __NO_DEBUG_MODE__
	
	#define	DBG(format, ...)		printf(format, ##__VA_ARGS__)
	#define DBG_OPTM(fmt,...)		printf("%s(%d)-<%s>: "##fmt,__FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#else
	
	#define	DBG(format, ...)		
	#define DBG_OPTM(fmt,...)		

#endif

#endif



/*
*   Internal function declaration 
*/
static EN_OXI600_ERR_TYPE _dev_Oxi600_WaitRegStatus(uint8_t RegAddr, uint16_t RegValMask, uint16_t ChkRegVal, uint32_t RetryCnt);
static void _dev_Oxi600_WriteRegContinuous(uint8_t regAddr,uint8_t *regData, uint8_t Count);
static void _dev_Oxi600_GetImageDataFromFIFO(uint8_t *DataBuf, uint32_t ImageSize);
static void _dev_Oxi600_LayoutRegInit(ST_OXI600_LAYOUT_REG_PARA stChnl600LayInfo);
static EN_OXI600_ERR_TYPE _dev_Oxi600_ClrScan(uint8_t FrameCnt, uint16_t StvCovCnt, uint16_t CpvPeriod,uint16_t u16ScanRow,uint8_t isCptGateoff,uint32_t timeout);
static EN_OXI600_ERR_TYPE _dev_Oxi600_CptScan(uint8_t FrameCnt, uint16_t InteLines ,uint32_t timeout);
static EN_OXI600_ERR_TYPE _dev_Oxi600_OutputGateoffLayOut(ST_CHNL600_CPT_PARA pstChnl600CptPara);
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK720_100umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara);
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK720_80umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara);
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK810_80umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara);
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK320_100umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara);
static EN_OXI600_ERR_TYPE _dev_Oxi600_SensorDistinguish(ST_CHNL600_CPT_PARA pstChnl600CptPara);
static EN_OXI600_ERR_TYPE _dev_Oxi600_RoicRegInit(void);
static EN_OXI600_ERR_TYPE _dev_Oxi600_ClrFrame(uint8_t prjType,Chl600_bool isCptGeteOff,uint32_t timeout);
static EN_OXI600_ERR_TYPE _dev_Oxi600_CptFrame(uint8_t prjType,uint16_t inteLine,uint32_t timeout);

static int _dev_Oxi600_WriteReg(uint8_t regAddr,uint8_t regData); /*wirte 1 byte*/
static uint16_t _dev_Oxi600_ReadReg(uint8_t RegAddr); /*read 1 byte*/
static uint32_t _dev_Oxi600_GetLocalTime( ); /*计时*/
static void _delay_ms( uint32_t n); 

uint32_t g_u32WinImgDataSize = 0;
uint8_t g_u8SwitchVcomFlag = 0;
uint8_t g_u8SwitchVcomFramCnt = 0;
uint16_t g_u16RowMax = 0;
uint8_t g_u8Vcom2vcom;
uint8_t g_u8Vcom2Vdd;
uint8_t g_u8Vcom2Vee;
uint8_t g_u8VcomStaus;
static ST_CHNL600_EXTER_DRV _g_stCh600ExterDrv;

EN_CHL600_CAPACITY g_enShtInteCapacity = EN_CAP_0_3PF;	/*short integration frame capacity*/	
EN_CHL600_CAPACITY g_enLongInteCapacity = EN_CAP_0_5PF; 	/*long integration frame capacity*/ 



ST_OXI600_CLR_PARA g_stOxi600clrpara =
{
	.bClrLagEn						= Chl600_TRUE, 	/*swtich to on/off clear lag operation*/

	.bVddEn							= Chl600_TRUE,		/*swtich Vcom to Vdd flag*/
	.u32SwVddDelay					= 10,		/*swtich Vcom voltage to Vdd delay, unit--ms*/
	.u8VddScnCnt					= 1,		/*frame scan number in VDD*/
	.u16VddPeriod					= 147,		/*CPV period*/
	.u16VddFsStvCovCnt				= 20,		/*STV cover CPV number*/
	
	.bVeeEn							= Chl600_FALSE,	/*swtich Vcom to Vee flag*/
	.u32SwVeeDelay					= 30,		/*swtich Vcom voltage to Vee delay, unit--ms*/
	.u8VeeScnCnt					= 5,			/*frame scan number in VDD*/
	.u16VeePeriod					= 30,		/*CPV period*/
	.u16VeeFsStvCovCnt				= 20,			/*STV cover CPV number*/

	.bVcomEn						= Chl600_TRUE,		/*swtich Vcom to Vcom flag*/
	.u32SwVcomDelay 				= 10,		/*swtich Vcom voltage to Vcom delay, unit--ms*/
	.u8VcomScnCnt					= 2, 		/*frame scan number in VDD*/
	.u16VcomPeriod					= 147,		/*CPV period*/
	.u16VcomFsStvCovCnt 			= 20,		/*STV cover CPV number*/	

	.u8CptImgCnt					= 3,		/*image capture counter*/
	//u16ShortIntlFramLineCnt	=			/*short integral lines*/
	.u16IntegralFramLineCnt 		= 600,		/*first frame integral frame line counter*/
	
};

const ST_OXI600_LAYOUT_REG_PARA g_stsensorLayout[] = 
{
	/*mode,colStr,border,colEnd,rowMax,colStr1/shift,colEnd1/shift,rowStr1,rowEnd1,FINGER 2,   adcMask*/
	
	{0x71, 35,    36,    465,   304,   36,   		 464, 		   6,      0,     36,464,6,0,  0xFFFE,}, /*MK720_80um*/
	{0x71, 34,	  35,	 458,	304,   99,	  		 457,		   38,	   0,	  99,457,38,0, 0xFFFF,}, /*MK720_100um*/
	{0x71, 31,	  10,	 568,	260,   38,	  		 538, 		   9,	   0,	  38,538,9,0,  0xFFFF,}, /*MK810_80um*/
	{0x71, 34,	  21,	 550,	252,   45,	   		 517,   	   1,	   0,	  45,517,1,0,  0xFFFF,}, /*MK320_100um*/
	{0x71, 31,	  10,	 568,	260,   38,	  		 538, 		   9,	   0,	  38,538,9,0,  0xFFFF,}, /*MK810_80um_ES1-3*/
};



/*
*	
*	border area start at str, include str 
*   str1 is the relative coordinate of str.
*   end1 is the relative coordinate of str1
*   [str1 end1) include str1 not include end1 when capture
*/
uint8_t g_u8oxiWriteToRegBuf[370];

uint8_t g_u8Oxi600RegInitBuf[370] = 
{						 // offset
	0xAB,  0x71,  0x00,  // 0 ADC_shift	ADC_de_set	/	ADC_mode 70:mod0;31:mod1;72:mod2 
	0xAC,  0x23,  0x00,	 // 1  ADC_col_start
	0xAD , 0x24 , 0x00,  // 2  ADC_col_border 
	0xAE , 0xd1 , 0x00,  // 3  ADC_col_end 
	0xAF , 0x01 , 0x00,  // 4  ADC_col_end //0x258(include darkline) 
	0xB8 , 0x30 , 0x00,  // 5 num_row_max_1 
	0xB9 , 0x01 , 0x00,  // 6 num_row_max_1 
	0xB0 , 0x24 , 0x00,  // 7 ADC_col_start_1 
	0xB1 , 0x00 , 0x00,  // 8 ADC_col_start_1 startX 0x0064 
	0xB2 , 0xd0 , 0x00,  // 9 ADC_col_end_1 
	0xB3 , 0x01 , 0x00 , // 10 ADC_col_end_1	endX 0x00E4 
	0xE4 , 0x64 , 0x00,  // 11 ADC_row_start_1 
	0xE5 , 0x00 , 0x00,  // 12 ADC_row_start_1	startY 0x0064 
	0xE6 , 0xE4 , 0x00,  // 13 ADC_row_end_1 
	0xE7 , 0x00 , 0x00,  // 14 ADC_row_end_1	endY 0x00E4 
	0xB4 , 0x2C , 0x00,  // 15 ADC_col_start_2 
	0xB5 , 0x01 , 0x00,  // 16 ADC_col_start_2	startX2 0x012C 
	0xB6 , 0xAC , 0x00,  // 17 ADC_col_end_2 
	0xB7 , 0x01 , 0x00,  // 18 ADC_col_end_2	endX2	0x01AC 
	0xE8 , 0x2C , 0x00,  // 19 ADC_row_start_2 
	0xE9 , 0x01 , 0x00,  // 20 ADC_row_start_2	startY2	0x012C 
	0xEA , 0xAC , 0x00,  // 21 ADC_row_end_2 
	0xEB , 0x01 , 0x00,  // 22 ADC_row_end_2	endY2	0x1AC 
	0x80 , 0x36 , 0x00,  // 23 P1 
	0x81 , 0x04 , 0x00,  // 24 P1 
	0x82 , 0xdd , 0x00,  // 25 P2
	0x83 , 0x03 , 0x00,  // 26 P2
	0x84 , 0x1e , 0x00,  // 27 P3
	0x85 , 0x03 , 0x00,  // 28 P3 
	0x86 , 0x00 , 0x00,  // 29 P3 
	0x87 , 0xfa , 0x00,  // 30 P4 
	0x88 , 0x06 , 0x00,  // 31 P4 
	0x89 , 0x1F , 0x00,  // 32 p_STV_cycle_num 
	0x8A , 0x01 , 0x00,  // 33 p_STV_high_clk 
	0x8B , 0x01 , 0x00,  // 34 p_STV_low_clk 
	0x8C , 0xC8 , 0x00,  // 35 P5 
	0x8D , 0x00 , 0x00,  // 36 P5 
	0x8E , 0x00 , 0x00,  // 37 P5  
	0x8F , 0x54 , 0x00,  // 38 P6 
	0x90 , 0x00 , 0x00,  // 39 P6 
	0x91 , 0x18 , 0x00,  // 40 P7   17
	0x92 , 0x01 , 0x00,  // 41 P7   01
	0x93 , 0x56 , 0x00,  // 42 P8   2c
	0x94 , 0x03 , 0x00,  // 43 P8		02
	0x95 , 0x46 , 0x00,  // 44 P9		45 
	0x96 , 0x00 , 0x00,  // 45 P9 	00
	0x97 , 0x38 , 0x00,  // 46 P10   88
	0x98 , 0x00 , 0x00,  // 47 P10 		00
	0x99 , 0x38 , 0x00,  // 48 P11    
	0x9A , 0x00 , 0x00,  // 49 P11 
	0x9B , 0x94 , 0x00,  // 50 P12		57
	0x9C , 0x05 , 0x00,  // 51 P12		04
	0x9D , 0xd2 , 0x00,  // 52 P13		d0
	0x9E , 0x00 , 0x00,  // 53 P13		00
	0x9F , 0x38 , 0x00,  // 54 P14 		37
	0xA0 , 0x00 , 0x00,  // 55 P14 
	0xA1 , 0x38 , 0x00,  // 56 P15 		37
	0xA2 , 0x00 , 0x00,  // 57 P15 
	0xA3 , 0xb4 , 0x00,  // 58 P16 
	0xA4 , 0x05 , 0x00,  // 59 P16 
	0xA5 , 0x90 , 0x00,  // 60 P17 
	0xA6 , 0x01 , 0x00,  // 61 P17 
	0xA7 , 0xac , 0x00,  // 62 P18		f7
	0xA8 , 0x06 , 0x00,  // 63 P18		05
	0xA9 , 0x28 , 0x00,  // 64 P19 
	0xAA , 0x01 , 0x00,  // 65 XAO_PIRST_CTRL 
	0xBA , 0xC8 , 0x00,  // 66 num_row_max_2 
	0xBB , 0x00 , 0x00,  // 67 num_row_max_2 
	0xC1 , 0x00 , 0x00,  // 68 /	/	/	/	/	/	pol_led_en	capture_forever 
	0xC2 , 0x02 , 0x00,  // 69 num_dark_led[7:4]	num_dark_led[3:0] 
	0xC3 , 0x00 , 0x00,  // 70 active_frame_h 
	0xC4 , 0x02 , 0x00,  // 71 active_frame_l 
	0xC5 , 0xFF , 0x00,  // 72 active_cap_h 
	0xC6 , 0xFF , 0x00,  // 73 active_cap_l 
	0xC7 , 0x00 , 0x00,  // 74 num_dly 
	0xC8 , 0x04 , 0x00,  // 75 value_std_fifo[15:8] 
	0xC9 , 0x00 , 0x00,  // 76 value_std_fifo[7:0] 
	0xCA , 0x05 , 0x00,  // 77 num_active_fire 
	0xCB , 0x05 , 0x00,  // 78 num_fm_dly 
	0xCC , 0x02 , 0x00,  // 79 hv_thres	half_fifo_shift 
	0xCD , 0x81 , 0x00,  // 80 D2A_REFINT_SEL 	D2A_AMP_SEL 
	0xCE , 0x08 , 0x00,  // 81 喔佮竵/	喔佮竵/	D2A_SLEEP_IRST	D2A_I_BUF 
	0xCF , 0x5F , 0x00,  // 82 D2P_VCOM2VEE  	D2P_VCOM2VDD   	D2P_VCOM2VCOM    	D2P_ENHV 	D2A_PD 	D2A_PDAD 	D2A_PDBGP  	D2A_PDCK 
	0xD0 , 0x41 , 0x00,  // 83 i2c_set	D2A_SWE     	D2A_TST   	D2A_EXREF 	D2A_EXVBG  	D2A_TSTADCB 
	0xD1 , 0x10 , 0x00,  // 84 debug_sel	D2A_TESTA 	D2A_RBG	D2A_TE 
	0xD2 , 0x4C , 0x00,  // 85 D2A_REFSEL	D2A_I_ADC 
	0xD3 , 0x78 , 0x00,  // 86 D2A_I_INT	D2A_I_REF 
	0xD4 , 0x20 , 0x00,  // 87 D2A_CSELX	/	D2A_RSELB 
	0xD5 , 0x00 , 0x00,  // 88 dly_set	p_STV_cycle_flag	p_clk_fast_flag 
	0xD6 , 0x04 , 0x00,  // 89 clk_sw	D2A_TESTBG	/	/	/	D2A_BG_TRIM 
	0xD7 , 0x00 , 0x00,  // 90 FIRE_ENABLE	D2P_PU_EN18	/	/	D2A_OSC_ITUNE	D2A_OSC_TESTEN 
	0xD8 , 0x44 , 0x00,  // 91 \\ OSC FREQUNCE 55MHZ CLK 
	0xD9 , 0x40 , 0x00,  // 92 intn_ext_sw	D2A_IREF_TRIM	D2A_BIAS_TESTEN	D2A_BIASEN 
	0xDA , 0x00 , 0x00,  // 93 /	/	/	D2A_LDO18EN 	D2A_LDO18_ITUNE 
	0xDB , 0x23 , 0x00,  // 94 \\ LDO18 
	0xDC , 0xCB , 0x00,  // 95 num_fire_cmd	num_fire_normal_frame	num_fire_stv_frame 
	0xDD , 0xA6 , 0x00,  // 96 fire_cmd2	fire_cmd1 
	0xDE , 0x30 , 0x00,  // 97 fire_cmd4	fire_cmd3 
	0xDF , 0x10 , 0x00,  // 98 fire_cmd6	fire_cmd5 
	0xEC , 0xFE , 0x00,  // 99 D2A_mask 
	0xED , 0xFF , 0x00,  // 100 D2A_mask 
	0xEE , 0x88 , 0x00,  // 101 VGG setting	VEE setting 
	0xEF , 0x88 , 0x00,  // 102 VCOM setting	I2C OSC setting
	0xFF , 0xFF , 0xFF,	 // 103
};




static void _dev_Oxi600_logErrType(char *buf,EN_OXI600_ERR_TYPE errType)
{
	DBG("_OXIFP_IC_DRV %s ,sub err type:",buf);
	switch(errType)
	{
		case EN_OXI600_SUCCESS:	
			DBG("operation success\r\n");
			break;
			
		case EN_OXI600_ERR_BASE_VAL:
			DBG("base value error\r\n");
			break;
			
		case EN_OXI600_IMAGE_SIZE_ERR:
			DBG("image size error\r\n");
			break;
			
		case EN_OXI600_REG_INIT_ERR:
			DBG("register init error\r\n");
			break;
			
		case EN_OXI600_CLR_RUN_ERR:
			DBG("clear lag operation failed\r\n");
			break;
			
		case EN_OXI600_CLR_RESTAT_ERR:	
			DBG("restart in clear lag failed\r\n");
			break;
			
		case EN_OXI600_CPT_RUN_ERR:
			DBG("capture image failed\r\n");
			break;
			
		case EN_OXI600_CPT_RESTAT_ERR:
			DBG("restart in capture image failed\r\n");
			break;
		case EN_OXI600_DATA_READY_ERR:
			DBG("wait for image data failed\r\n");
			break;
			
		case EN_OXI600_SLEEP_ROIC_ERR:
			DBG("sleep RIOC failed\r\n");
			break;
			
		case EN_OXI600_PROJECT_TYPE_ERR:
			DBG("para project id err,not exist \r\n");
			break;
	
		default:
			DBG("there is no such err type\r\n");
			break;
	}
}


/**
 * @brief Write values to specified registers
 * @param RegAddr which register address will be wirte
 * @param regData register value
 * @retval none
 */
static int _dev_Oxi600_WriteReg(uint8_t regAddr,uint8_t regData)/*wirte 1 byte*/
{
	int retVal;
	uint8_t u8regBuf[2]={0xff,0xff};
	u8regBuf[0] = regAddr;
	u8regBuf[1] = regData;
	retVal = _g_stCh600ExterDrv.SPI_Send(u8regBuf,2);
	return retVal;
}

/**
 * @brief Read values in specified registers
 * @param RegAddr which register address will be read
 * @retval Register value
 */
static uint16_t _dev_Oxi600_ReadReg(uint8_t RegAddr) /*read 1 byte*/
{
	uint16_t u16RetVal ;	
	uint8_t u8RegReceBUf[2];
	RegAddr &= 0x7F;
	u8RegReceBUf[0] = RegAddr;
	u8RegReceBUf[1] = 0xff;
	_g_stCh600ExterDrv.SPI_Send(u8RegReceBUf,2);
	
	u8RegReceBUf[0] = 0xff;				/* Just in case, MOSI need 0xff when spi read reg*/
	_g_stCh600ExterDrv.SPI_Receive(u8RegReceBUf,2);

	u16RetVal = ((uint16_t)u8RegReceBUf[0])<<8 | u8RegReceBUf[1];
	return u16RetVal;
}

/**
 * @brief SPI send mass of data
 * @param databuf 
 * @param size
 * @retval error code
 */
static int _dev_Oxi600_SpiSendMass(uint8_t *dataBuf,uint32_t size)
{
	_g_stCh600ExterDrv.SPI_Send_mass(dataBuf,size);
    return 0;
}


/**
 * @brief SPI send mass of data
 * @param databuf 
 * @param size
 * @retval error code
 */
static int _dev_Oxi600_SpiReceiveMass(uint8_t *dataBuf,uint32_t size)
{
	_g_stCh600ExterDrv.SPI_Receive_mass(dataBuf,size);
    return 0;
}

/**
 * @brief get system local time 
 * @retval current time
 */
static uint32_t _dev_Oxi600_GetLocalTime(void) /*计时*/
{
	return _g_stCh600ExterDrv.getLocalTime();
}

/**
 * @brief delay ms
 * @param number of ms
 * @retval none 
 */
static void _delay_ms(uint32_t n)
{	
	_g_stCh600ExterDrv.delay_ms(n);
    return ;
}

#if 0 /*not use now */
/**
 * @brief Write values to specified registers Continuously
 * @param RegAddr Starting address for continuous writing
 * @param regData pointer to register data buf
 * @param Count amount of data will be writing
 * @retval none
 */
static void _dev_Oxi600_WriteRegContinuous(uint8_t regAddr,uint8_t *regData, uint8_t Count)
{
	uint32_t i;
	uint8_t u8regBuf[2]={0xff,0xff};
		
	for(i = 0; i < Count; i++)
	{	
		u8regBuf[0] = regAddr+i;
		u8regBuf[1] = regData[i];
		Dev_Oxi600_WriteReg(u8regBuf[0],u8regBuf[1]);
		
	}
}

#endif

/**
 * @brief Judgment of FSM
 * @param RegAddr Should be 0x3F 
 * @param RegValMask Mask
 * @param ChkRegVal Number corresponding to the current state
 * @param RetryCnt Number of queries
 * @retval EN_OXI600_ERR_TYPE
 */
static EN_OXI600_ERR_TYPE _dev_Oxi600_WaitRegStatus(uint8_t RegAddr, uint16_t RegValMask, uint16_t ChkRegVal, uint32_t timeout)
{
	uint16_t u16RcvRegVal;
	EN_OXI600_ERR_TYPE u8Result;
	uint32_t u32entryTime,u32localTime;
	u8Result = EN_OXI600_SUCCESS;			
	u32entryTime = _dev_Oxi600_GetLocalTime();
	while(1)
	{
		u16RcvRegVal = _dev_Oxi600_ReadReg(RegAddr);
		u16RcvRegVal &=RegValMask;
		if(u16RcvRegVal == ChkRegVal)
		{
			u8Result = EN_OXI600_SUCCESS ;
			break;
		}
		_delay_ms(1);
		u32localTime = _dev_Oxi600_GetLocalTime();
		
		if(u32localTime -u32entryTime > timeout)
		{
			DBG("_OXIFP_IC_DRV wait reg status timeout,reg = %#X,mask = %#X,check val = %#X\n",RegAddr,RegValMask,ChkRegVal);
			u8Result = EN_OXI600_CHECK_STATUS_TIMEOUT ;
			break;
		}
	}
	
	return u8Result;
}


/**
 * @brief Receive amount of data from FIFO
 * @param DataBuf pointer to data buffer
 * @param ImageSize amount of data to be received from FIFO
 * @retval none
 */
static void _dev_Oxi600_GetImageDataFromFIFO(uint8_t *DataBuf, uint32_t ImageSize)
{
	ROIC_CMD_TRANSFER();
	ROIC_CMD_DUMMY();
 	_dev_Oxi600_SpiReceiveMass(DataBuf,ImageSize);
	
}

/**
 * @brief Set layout register
 * @param stChnl600LayInfo 
 * @retval none
 */
static void _dev_Oxi600_LayoutRegInit(ST_OXI600_LAYOUT_REG_PARA stChnl600LayInfo)
{
	uint16_t i;
	for(i=0; i<23; i++)
	{
		if(g_u8oxiWriteToRegBuf[3*i] == 0xAB)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = stChnl600LayInfo.u8mode;
		}

		if(g_u8oxiWriteToRegBuf[3*i] == 0xAC)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = stChnl600LayInfo.u8colStr;
		}

		if(g_u8oxiWriteToRegBuf[3*i] == 0xAD)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = stChnl600LayInfo.u8border;
		}

		if(g_u8oxiWriteToRegBuf[3*i] == 0xAE)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16colEnd & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xAF)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16colEnd >> 8) & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB8)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16rowMax & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB9)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16rowMax >> 8) & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB0)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16colStr1 & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB1)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16colStr1 >>8) & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB2)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16colEnd1 & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB3)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16colEnd1 >> 8) & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xE4)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16rowStr1 & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xE5)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16rowStr1 >> 8) & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xE6)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16rowEnd1 & 0xFF);
		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xE7)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16rowEnd1 >> 8) & 0xFF);
		}
	
		/*double finger*/
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB4)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16colStr2 & 0xFF);
	
		}	
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB5)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16colStr2 >> 8) & 0xFF);

		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB6)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16colEnd2 & 0xFF);

		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xB7)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16colEnd2 >> 8) & 0xFF);

		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xE8)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16rowStr2 & 0xFF);

		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xE9)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16rowStr2 >> 8) & 0xFF);

		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xEA)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)(stChnl600LayInfo.u16rowEnd2 & 0xFF);

		}
		
		if(g_u8oxiWriteToRegBuf[3*i] == 0xEB)
		{
			g_u8oxiWriteToRegBuf[3*i+1] = (uint8_t)((stChnl600LayInfo.u16rowEnd2 >> 8) & 0xFF);

		}

				
	}

	g_u8oxiWriteToRegBuf[99*3+1] = (uint8_t)(stChnl600LayInfo.u16adcMask & 0xFF);
	g_u8oxiWriteToRegBuf[100*3+1] = (uint8_t)(stChnl600LayInfo.u16adcMask >> 8);
}

/**
 * @brief Configure clear frame parameters and generate waveforms
 * @param FrameCnt Frame number
 * @param StvCovCnt The number of STV coverage CPV
 * @param CpvPeriod Cycle of CPV
 * @param u16ScanRow Scanning rows
 * @retval EN_OXI600_ERR_TYPE
 */
static EN_OXI600_ERR_TYPE _dev_Oxi600_ClrScan(uint8_t FrameCnt, uint16_t StvCovCnt, uint16_t CpvPeriod,uint16_t u16ScanRow,uint8_t isCptGateoff,uint32_t timeout)
{
	uint16_t u16CPV_L,u16CPV_H,u16STV_H;
	uint32_t u32StvStartTime;
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
#if 0	
	u16CPV_L = CpvPeriod /2 * 14;
	u16CPV_H = CpvPeriod /2 * 14;
	u16STV_H = CpvPeriod*StvCovCnt * 14;
	u32StvStartTime = 0x68;
#else
	ROIC_CMD_RESTART();
	ROIC_CMD_WAKEUP();

	u16STV_H =(((uint16_t)g_u8oxiWriteToRegBuf[24*3+1]<<8 | g_u8oxiWriteToRegBuf[23*3+1])+\
				((uint16_t)g_u8oxiWriteToRegBuf[26*3+1]<<8 | g_u8oxiWriteToRegBuf[25*3+1]))*StvCovCnt;
	g_u8oxiWriteToRegBuf[30*3+1] = u16STV_H & 0xff;    /*87 stv_l*/
	g_u8oxiWriteToRegBuf[31*3+1] = (u16STV_H>>8) & 0xff; /*87 stv_h*/
	g_u8oxiWriteToRegBuf[65*3+1] = 0x0b;			   /*aa pirst_H*/
	g_u8oxiWriteToRegBuf[69*3+1] = FrameCnt;		   /*c2 frame cnt*/
	g_u8oxiWriteToRegBuf[71*3+1] = isCptGateoff;	   /*c4 is write fifo*/	
	g_u8oxiWriteToRegBuf[5*3+1] = u16ScanRow & 0xff;     /*b8 row max_l*/
	g_u8oxiWriteToRegBuf[6*3+1] = (u16ScanRow>>8) & 0xff;  /*b9 row max_H*/
	enRetVal = _dev_Oxi600_RoicRegInit();
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		DBG("_OXIFP_IC_DRV clr scan reg init err,type = %d\n",enRetVal);
		return enRetVal;
	}
#endif
	//ROIC_CMD_RESTART();

	//ROIC_CMD_WAKEUP();
#if 0	
	PIRST_H();
	_dev_Oxi600_WriteRegContinuous(Oxi600_ROIC_REG_P1_CPV_LOW,(uint8_t*)&u16CPV_L,2);
	_dev_Oxi600_WriteRegContinuous(Oxi600_ROIC_REG_P2_CPV_HIGH,(uint8_t*)&u16CPV_H,2);
	_dev_Oxi600_WriteRegContinuous(Oxi600_ROIC_REG_P3_STV_START,(uint8_t*)&u32StvStartTime,3);
	_dev_Oxi600_WriteRegContinuous(Oxi600_ROIC_REG_P4_STV_HIGH,(uint8_t*)&u16STV_H,2);
	_dev_Oxi600_WriteRegContinuous(Oxi600_ROIC_REG_ROW_MAX,(uint8_t*)&u16ScanRow,2);
	Dev_Oxi600_WriteReg(Oxi600_ROIC_REG_FRAME_CNT,FrameCnt);
	Dev_Oxi600_WriteReg(Oxi600_ROIC_REG_IS_WRITE_DATA,isCptGateoff);
#endif 
	ROIC_CMD_RUN();
	if(isCptGateoff == 0) 
	{
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x0f00,0x0b00,timeout) != EN_OXI600_SUCCESS)
		{
			DBG("_OXIFP_IC_DRV clr scan no gateoff check FSM timeout\n");
			return EN_OXI600_CLR_RUN_ERR; 	
		}
	}
	else if(isCptGateoff == 1)
	{
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x0f00,0x0700,timeout) != EN_OXI600_SUCCESS)
		{
			DBG("_OXIFP_IC_DRV clr scan with gateoff check FSM timeout\n");
			return EN_OXI600_CLR_RUN_ERR;	
		}
	}
#if 0	
	ROIC_CMD_RESTART();
	if(_dev_Oxi600_WaitRegStatus(0x3f,0x0f00,0x0400,timeout) != EN_OXI600_SUCCESS)
	{
		return EN_OXI600_CLR_RESTAT_ERR;
	}
	//PIRST_L();								//   NOT WORKING ? SHOULD NOT EXSIT  IS CORRECT? 
	XAO_L_PIRST_H();
#endif
	return EN_OXI600_SUCCESS;
}

/**
 * @brief Configure clear frame parameters and generate waveforms
 * @param FrameCnt Frame number
 * @param InteLines The number of integration time
 * @param timeout 
 * @retval EN_OXI600_ERR_TYPE
 */
static EN_OXI600_ERR_TYPE _dev_Oxi600_CptScan(uint8_t FrameCnt, uint16_t InteLines,uint32_t timeout)
{
	uint16_t u16CPV_L,u16CPV_H,u16STV_H;
	uint32_t u32StvStartTime;
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;

#if 0  // wirete reg 
	u16CPV_L =(uint16_t)g_u8Oxi600RegInitBuf[24*3+1]<<8 | g_u8Oxi600RegInitBuf[23*3+1];// 523;		// P1
	u16CPV_H = (uint16_t)g_u8Oxi600RegInitBuf[26*3+1]<<8 | g_u8Oxi600RegInitBuf[25*3+1];//1549;	//P2
	u32StvStartTime =(uint32_t)g_u8Oxi600RegInitBuf[29*3+1]<<16|(uint16_t)g_u8Oxi600RegInitBuf[28*3+1]<<8 | g_u8Oxi600RegInitBuf[27*3+1];  // 462;		//P3
	u16STV_H =(uint16_t)g_u8Oxi600RegInitBuf[31*3+1]<<8 | g_u8Oxi600RegInitBuf[30*3+1];;//2074;		//P4 	
	//ROIC_CMD_WAKEUP();

	Dev_Oxi600_WriteReg(Oxi600_ROIC_REG_FRAME_CNT,FrameCnt);
	Dev_Oxi600_WriteReg(Oxi600_ROIC_REG_IS_WRITE_DATA,8);
	Dev_Oxi600_WriteReg(Oxi600_ROIC_REG_C6_ACTIVE_CAP,0xFB);
	VCOM_TO_VCOM(g_u8Vcom2vcom);
	ROIC_CMD_RUN();
#endif 	
	ROIC_CMD_RESTART();
	ROIC_CMD_WAKEUP();
	g_u8oxiWriteToRegBuf[69*3+1] = FrameCnt;		   		/*c2 frame cnt*/
	g_u8oxiWriteToRegBuf[71*3+1] = 1<<(FrameCnt-1);	          /*c4 is write fifo*/ 
	g_u8oxiWriteToRegBuf[5*3+1] = g_u16RowMax & 0xff;	 /*b8 row max_l*/
	g_u8oxiWriteToRegBuf[6*3+1] = (g_u16RowMax>>8) & 0xff; /*b9 row max_H*/
	g_u8oxiWriteToRegBuf[73*3+1] = 0xff - (1<<(FrameCnt-2));	/*c6 cap_l*/
	g_u8oxiWriteToRegBuf[66*3+1] = InteLines & 0xff;		/*ba row max2*/
	g_u8oxiWriteToRegBuf[67*3+1] = (InteLines>>8) & 0xff; /*bb row max2*/	
	g_u8oxiWriteToRegBuf[82*3+1] = g_u8VcomStaus;			/*cf*/
	//ROIC_CMD_WAKEUP();
	enRetVal= _dev_Oxi600_RoicRegInit();	
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		DBG("_OXIFP_IC_DRV cpt scan reg init err,type = %d\n",enRetVal);
	}
	//ROIC_CMD_RESTART();
	ROIC_CMD_RUN();
#if 0	
	if(IsWriteFIFO == 0)
	{
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0B00,timeout) != EN_OXI600_SUCCESS)
		{
			return EN_OXI600_CPT_RUN_ERR;
		}
		
		ROIC_CMD_RESTART();
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0400,timeout) != EN_OXI600_SUCCESS)
		{
			return EN_OXI600_CPT_RESTAT_ERR;
		}
	}
	else
	{
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x8000,0x8000,timeout) != EN_OXI600_SUCCESS)
		{
			return EN_OXI600_DATA_READY_ERR;
		}
		
	//	g_readCurrent = BSP_ReadCurrent(1);
		
		_dev_Oxi600_GetImageDataFromFIFO(dataBuf,g_u32WinImgDataSize);
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0B00,timeout) != EN_OXI600_SUCCESS)
		{
			return EN_OXI600_CPT_RUN_ERR;
		}

		ROIC_CMD_RESTART();
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0400,timeout) != EN_OXI600_SUCCESS)
		{
			return EN_OXI600_CPT_RESTAT_ERR;
		}

		XAO_L();
		ROIC_CMD_SLEEP();
		DISABLE_ROIC_ADC();
		if(_dev_Oxi600_WaitRegStatus(0x3f,0xFF00,0x7000,timeout) != EN_OXI600_SUCCESS)
		{
			return EN_OXI600_SLEEP_ROIC_ERR;
		}
			

	}
#else
		if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0700,timeout) != EN_OXI600_SUCCESS)
		{
			DBG("_OXIFP_IC_DRV cpt scan check FSM timeout\n");
			return EN_OXI600_CPT_RUN_ERR;
		}

#endif
	return EN_OXI600_SUCCESS;
}

/**
 * @brief output gateoff initialize configuration
 * @param pstChnl600CptPara 
 * @retval EN_OXI600_ERR_TYPE
**/
static EN_OXI600_ERR_TYPE _dev_Oxi600_OutputGateoffLayOut(ST_CHNL600_CPT_PARA pstChnl600CptPara)
{
	ST_OXI600_LAYOUT_REG_PARA stChnl600LayInfo;
	static uint16_t u16rowOffset;

	
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	switch(pstChnl600CptPara.PrjType)
	{
		case EN_PRJ_OXI600_MK720_80UM:
			g_u16RowMax = g_stsensorLayout[0].u16rowMax;
			break;

		case EN_PRJ_OXI600_MK720_100UM:
			g_u16RowMax = g_stsensorLayout[1].u16rowMax;
			break;

		case EN_PRJ_OXI600_MK810_80UM:
			g_u16RowMax = g_stsensorLayout[2].u16rowMax;
			break;
			
		case EN_PRJ_OXI600_MK320_100UM:
			g_u16RowMax = g_stsensorLayout[3].u16rowMax;
			break;

		case EN_PRJ_OXI600_MK810_80UM_04:
			g_u16RowMax = g_stsensorLayout[4].u16rowMax;
			break;		
			
		default:
			enRetVal = EN_OXI600_PROJECT_TYPE_ERR;
			DBG("_OXIFP_IC_DRV dev gateoff layout para in err, type: %#X\n",pstChnl600CptPara.PrjType);
			break;
	}
	stChnl600LayInfo.u8mode = 0x31;			/*reg: 0xAB*/
	stChnl600LayInfo.u8colStr = 0x04;		/*reg: 0xAC*/
	stChnl600LayInfo.u8border = 0;			/*reg: 0xAD*/		
	stChnl600LayInfo.u16colEnd = 0x258; 		/*reg: 0xAE,0xAF*/
//	stChnl600LayInfo.u16rowMax = g_u16RowMax+20;//CHNL600_MK810_80UM_ROW_MAX; 		/*reg: 0xB8,0xB9*/
	stChnl600LayInfo.u16adcMask = 0xFFFF;   /*reg: 0xEC,0xED*/
	
	stChnl600LayInfo.u16colStr1 = 0;
	stChnl600LayInfo.u16colEnd1 = 0x258;

	/*to get gateoff ,incease 18 rows when scanning ,and get the last 10 rows (8 - 18 )*/
	stChnl600LayInfo.u16rowStr1 = g_u16RowMax+8;								/*reg: 0xE4,0xE5*/
	stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + 10;

	g_u32WinImgDataSize = 600*10*2;
				
	
	_dev_Oxi600_LayoutRegInit(stChnl600LayInfo);
	
	return EN_OXI600_SUCCESS;

}



/**
 * @brief MK720_80um initialize configuration
 * @param pstChnl600CptPara 
 * @retval EN_OXI600_ERR_TYPE
**/
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK720_80umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara)
{
	ST_OXI600_LAYOUT_REG_PARA stChnl600LayInfo;
	static uint16_t u16rowOffset = 0;

#if 0
	stChnl600LayInfo.u8mode = EN_MODE1_CPT_SGL_WIN;   					 	/*reg: 0xAB*/
	stChnl600LayInfo.u8colStr = CHNL600_MK720_80UM_COL_STR;					/*reg: 0xAC*/
	stChnl600LayInfo.u8border = CHNL600_MK720_80UM_BORDER;					/*reg: 0xAD*/		
	stChnl600LayInfo.u16colEnd = CHNL600_MK720_80UM_COL_END; 				/*reg: 0xAE,0xAF*/
	stChnl600LayInfo.u16rowMax = CHNL600_MK720_80UM_ROW_MAX; 				/*reg: 0xB8,0xB9*/
	stChnl600LayInfo.u16adcMask = CHNL600_MK720_80UM_ADC_MASK;    			/*reg: 0xEC,0xED*/
	g_u16RowMax = CHNL600_MK720_80UM_ROW_MAX;
#endif		
	memcpy(&stChnl600LayInfo,&g_stsensorLayout[0],sizeof(ST_OXI600_LAYOUT_REG_PARA));
	g_u16RowMax = g_stsensorLayout[0].u16rowMax;

	if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WIN_IMG)
	{
		pstChnl600CptPara.W1ColNo = 138;
		pstChnl600CptPara.W1RowNo = 138;

		if((pstChnl600CptPara.W1ColStrt + pstChnl600CptPara.W1ColNo > 415)\
			|| (pstChnl600CptPara.W1RowStrt + pstChnl600CptPara.W1RowNo > 292))
		{
			DBG("_OXIFP_IC_DRV coordinate out of range,x=%d,y=%d\n",pstChnl600CptPara.W1ColStrt,pstChnl600CptPara.W1RowStrt);
			return EN_OXI600_COOR_ERR;
		}
		
		if(pstChnl600CptPara.W1ColStrt <= 22)
		{
			stChnl600LayInfo.u16colStr1 = g_stsensorLayout[0].u16colStr1;
		}
		else
		{
			stChnl600LayInfo.u16colStr1 = pstChnl600CptPara.W1ColStrt - 22 + g_stsensorLayout[0].u16colStr1;
		}
		stChnl600LayInfo.u16colEnd1 = stChnl600LayInfo.u16colStr1 + pstChnl600CptPara.W1ColNo;
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[0].u16rowStr1;
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;
		g_u32WinImgDataSize = (pstChnl600CptPara.W1ColNo+g_stsensorLayout[0].u8border+10)*\
									pstChnl600CptPara.W1RowNo*2;
		if(g_u32WinImgDataSize > 64*1024)
		{
			DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#if 0		
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
		#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
		#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#endif		
	}
	else if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WHLE_ROW_IMG)
	{
	//	pstChnl600CptPara.W1RowNo = pstChnl600CptPara.CptDataSize /2 /(g_stsensorLayout[0].u16colEnd1\
	//								- g_stsensorLayout[0].u16colStr1 + 1+ 10);
	
	//	if(pstChnl600CptPara.WhlImageNo == 1)
	//	{
	//		u16rowOffset = g_stsensorLayout[0].u16rowStr1;
	//	}
		
		stChnl600LayInfo.u16colStr1 = g_stsensorLayout[0].u16colStr1;						/*reg: 0xB0,0xB1*/
		stChnl600LayInfo.u16colEnd1 = g_stsensorLayout[0].u16colEnd1;						/*reg: 0xB2,0xB3*/
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[0].u16rowStr1;//u16rowOffset;											/*reg: 0xE4,0xE5*/
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;/*reg: 0xE6,0xE7*/
													
		u16rowOffset += pstChnl600CptPara.W1RowNo;
		g_u32WinImgDataSize =(stChnl600LayInfo.u16colEnd1 - stChnl600LayInfo.u16colStr1 + 1+ 10)*pstChnl600CptPara.W1RowNo*2;
		if(g_u32WinImgDataSize > 64*1024)
		{
			DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#if 0		
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
		#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
		#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#endif		
	}
	
	_dev_Oxi600_LayoutRegInit(stChnl600LayInfo);
	
	return EN_OXI600_SUCCESS;

}

/**
 * @brief MK720_100um initialize configuration
 * @param pstChnl600CptPara 
 * @retval EN_OXI600_ERR_TYPE
**/
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK720_100umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara)
{
	ST_OXI600_LAYOUT_REG_PARA stChnl600LayInfo;
	static uint16_t u16rowOffset = 0;

#if 0	
	stChnl600LayInfo.u8mode = EN_MODE1_CPT_SGL_WIN; 						/*reg: 0xAB*/
	stChnl600LayInfo.u8colStr = CHNL600_MK720_100UM_COL_STR;					/*reg: 0xAC*/
	stChnl600LayInfo.u8border = CHNL600_MK720_100UM_BORDER; 				/*reg: 0xAD*/		
	stChnl600LayInfo.u16colEnd = CHNL600_MK720_100UM_COL_END;						/*reg: 0xAE,0xAF*/
	stChnl600LayInfo.u16rowMax = CHNL600_MK720_100UM_ROW_MAX;						/*reg: 0xB8,0xB9*/
	stChnl600LayInfo.u16adcMask = CHNL600_MK720_100UM_ADC_MASK;					/*reg: 0xEC,0xED*/
	g_u16RowMax = CHNL600_MK720_100UM_ROW_MAX;
#endif	
	memcpy(&stChnl600LayInfo,&g_stsensorLayout[1],sizeof(ST_OXI600_LAYOUT_REG_PARA));
	g_u16RowMax = stChnl600LayInfo.u16rowMax;

	if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WIN_IMG)
	{
		pstChnl600CptPara.W1ColNo = 128;
		pstChnl600CptPara.W1RowNo = 128;
		
		if((pstChnl600CptPara.W1ColStrt + pstChnl600CptPara.W1ColNo > 324)\
			|| (pstChnl600CptPara.W1RowStrt + pstChnl600CptPara.W1RowNo > 228))
		{
			DBG("_OXIFP_IC_DRV coordinate out of range,x=%d,y=%d\n",pstChnl600CptPara.W1ColStrt,pstChnl600CptPara.W1RowStrt);
			return EN_OXI600_COOR_ERR;
		}
		
		stChnl600LayInfo.u16colStr1 = pstChnl600CptPara.W1ColStrt + g_stsensorLayout[1].u16colStr1;
		stChnl600LayInfo.u16colEnd1 = stChnl600LayInfo.u16colStr1 + pstChnl600CptPara.W1ColNo;
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[1].u16rowStr1;
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;
		g_u32WinImgDataSize = (g_stsensorLayout[1].u8border+10+pstChnl600CptPara.W1ColNo)*\
								pstChnl600CptPara.W1RowNo*2;
		if(g_u32WinImgDataSize > 64*1024)
		{
			DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
	#if 0
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
	#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
	#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
	#endif	
	}
	else if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WHLE_ROW_IMG)
	{
		//pstChnl600CptPara.W1RowNo = pstChnl600CptPara.CptDataSize /2 /(g_stsensorLayout[1].u16colEnd1 \
		//						- g_stsensorLayout[1].u16colStr1 + 1+ 10);

	
		//if(pstChnl600CptPara.WhlImageNo == 1)
		//{
		//	u16rowOffset = g_stsensorLayout[1].u16rowStr1;
		//}
		
		stChnl600LayInfo.u16colStr1 = g_stsensorLayout[1].u16colStr1;			/*reg: 0xB0,0xB1*/
		stChnl600LayInfo.u16colEnd1 = g_stsensorLayout[1].u16colEnd1; 			/*reg: 0xB2,0xB3*/
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[1].u16rowStr1;//u16rowOffset; 							/*reg: 0xE4,0xE5*/
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;	/*reg: 0xE6,0xE7*/
													
		u16rowOffset += pstChnl600CptPara.W1RowNo;
		g_u32WinImgDataSize = (stChnl600LayInfo.u16colEnd1 - stChnl600LayInfo.u16colStr1 + 1+ 10)*\
								(stChnl600LayInfo.u16rowEnd1 - stChnl600LayInfo.u16rowStr1)*2;
		if(g_u32WinImgDataSize > 64*1024)
		{
			DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#if 0		
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
	#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
	#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#endif		
	}
	
	_dev_Oxi600_LayoutRegInit(stChnl600LayInfo);
	
	return EN_OXI600_SUCCESS;

}

/**
 * @brief MK810_80um initialize configuration
 * @param pstChnl600CptPara 
 * @retval EN_OXI600_ERR_TYPE
**/
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK810_80umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara)
{
	ST_OXI600_LAYOUT_REG_PARA stChnl600LayInfo;
	static uint16_t u16rowOffset;

#if 0
	stChnl600LayInfo.u8mode = EN_MODE1_CPT_SGL_WIN; 				/*reg: 0xAB*/
	stChnl600LayInfo.u8colStr = CHNL600_MK810_80UM_COL_STR;			/*reg: 0xAC*/
	stChnl600LayInfo.u8border = CHNL600_MK810_80UM_BORDER;			/*reg: 0xAD*/		
	stChnl600LayInfo.u16colEnd = CHNL600_MK810_80UM_COL_END; 		/*reg: 0xAE,0xAF*/
	stChnl600LayInfo.u16rowMax = CHNL600_MK810_80UM_ROW_MAX; 		/*reg: 0xB8,0xB9*/
	stChnl600LayInfo.u16adcMask = CHNL600_MK810_80UM_ADC_MASK;    	/*reg: 0xEC,0xED*/
	g_u16RowMax = CHNL600_MK810_80UM_ROW_MAX;
#endif
	
	memcpy(&stChnl600LayInfo,&g_stsensorLayout[2],sizeof(ST_OXI600_LAYOUT_REG_PARA));
	g_u16RowMax = g_stsensorLayout[2].u16rowMax;

	if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WIN_IMG)
	{
		pstChnl600CptPara.W1ColNo = 138;
		pstChnl600CptPara.W1RowNo = 138;
		
		if((pstChnl600CptPara.W1ColStrt + pstChnl600CptPara.W1ColNo > 500)\
			|| (pstChnl600CptPara.W1RowStrt + pstChnl600CptPara.W1RowNo > 250))
		{
			DBG("_OXIFP_IC_DRV coordinate out of range,x=%d,y=%d\n",pstChnl600CptPara.W1ColStrt,pstChnl600CptPara.W1RowStrt);
			return EN_OXI600_COOR_ERR;
		}

		stChnl600LayInfo.u16colStr1 = pstChnl600CptPara.W1ColStrt + g_stsensorLayout[2].u16colStr1; /*reg: 0xB0,0xB1*/
		stChnl600LayInfo.u16colEnd1 = stChnl600LayInfo.u16colStr1 + pstChnl600CptPara.W1ColNo;	 /*reg: 0xB2,0xB3*/
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[2].u16rowStr1; /*reg: 0xE4,0xE5*/
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;	 /*reg: 0xE6,0xE7*/

		if(pstChnl600CptPara.enCptMode == EN_MODE1_CPT_SGL_WIN)
		{
			g_u32WinImgDataSize = (stChnl600LayInfo.u16colEnd1 - stChnl600LayInfo.u16colStr1 + stChnl600LayInfo.u8border)*\
										(stChnl600LayInfo.u16rowEnd1 - stChnl600LayInfo.u16rowStr1)*2;
			#if 0
			if(g_u32WinImgDataSize > 64*1024)
			{
				DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
				return EN_OXI600_IMAGE_SIZE_ERR;
			}
			#endif
		}
		else if(pstChnl600CptPara.enCptMode == EN_MODE2_CPT_DBL_WIN)
		{
			pstChnl600CptPara.W2ColNo = 138;
			pstChnl600CptPara.W2RowNo = 138;
			if((pstChnl600CptPara.W2ColStrt + pstChnl600CptPara.W2ColNo > 500)\
				|| (pstChnl600CptPara.W2RowStrt + pstChnl600CptPara.W2RowNo > 250))
			{
				DBG("_OXIFP_IC_DRV coordinate out of range,x=%d,y=%d\n",pstChnl600CptPara.W1ColStrt,pstChnl600CptPara.W1RowStrt);
				return EN_OXI600_COOR_ERR;
			}
			
			stChnl600LayInfo.u8mode = EN_MODE2_CPT_DBL_WIN;
			stChnl600LayInfo.u16colStr2 = pstChnl600CptPara.W2ColStrt + g_stsensorLayout[2].u16colStr1; /*reg: 0xB4,0xB5*/
			stChnl600LayInfo.u16colEnd2 = stChnl600LayInfo.u16colStr2 + pstChnl600CptPara.W2ColNo;	 /*reg: 0xB6,0xB7*/
			stChnl600LayInfo.u16rowStr2 = pstChnl600CptPara.W2RowStrt + g_stsensorLayout[2].u16rowStr1; /*reg: 0xE8,0xE9*/
			stChnl600LayInfo.u16rowEnd2 = stChnl600LayInfo.u16rowStr2 + pstChnl600CptPara.W2RowNo;	 /*reg: 0xEA,0xEB*/

			if(pstChnl600CptPara.W1RowStrt < pstChnl600CptPara.W2RowStrt)	/*case: y1 < y2*/
			{
				g_u32WinImgDataSize = stChnl600LayInfo.u8border*2*(pstChnl600CptPara.W2RowStrt - pstChnl600CptPara.W1RowStrt\
									+ pstChnl600CptPara.W2ColNo) + pstChnl600CptPara.W1ColNo*pstChnl600CptPara.W1RowNo*2 \
									+ pstChnl600CptPara.W2ColNo*pstChnl600CptPara.W2RowNo*2;
			}
		}
		
#if 0		
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
		#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
		#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#endif		
	}
	else if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WHLE_ROW_IMG)
	{
		//pstChnl600CptPara.W1RowNo = pstChnl600CptPara.CptDataSize /2 / (g_stsensorLayout[2].u16colEnd1\
		//							- g_stsensorLayout[2].u16colStr1 + g_stsensorLayout[2].u8border);

		//if(pstChnl600CptPara.WhlImageNo == 1)
		//{
		//	u16rowOffset = g_stsensorLayout[2].u16rowStr1;
		//}
		
		stChnl600LayInfo.u16colStr1 = g_stsensorLayout[2].u16colStr1;			/*reg: 0xB0,0xB1*/
		stChnl600LayInfo.u16colEnd1 = g_stsensorLayout[2].u16colEnd1;			/*reg: 0xB2,0xB3*/
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[2].u16rowStr1;//u16rowOffset;								/*reg: 0xE4,0xE5*/
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;	/*reg: 0xE6,0xE7*/
		
		u16rowOffset += pstChnl600CptPara.W1RowNo;
		g_u32WinImgDataSize = (stChnl600LayInfo.u16colEnd1 - stChnl600LayInfo.u16colStr1 + stChnl600LayInfo.u8border)*\
								(stChnl600LayInfo.u16rowEnd1 - stChnl600LayInfo.u16rowStr1 )*2;
		if(g_u32WinImgDataSize > 64*1024)
		{
			DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#if 0		
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
		#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
		#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
#endif		
	}
	
	_dev_Oxi600_LayoutRegInit(stChnl600LayInfo);
	
	return EN_OXI600_SUCCESS;

}


/**
 * @brief MK320_100um initialize configuration
 * @param pstChnl600CptPara 
 * @retval EN_OXI600_ERR_TYPE
**/
static EN_OXI600_ERR_TYPE _dev_Oxi600_MK320_100umSensorLayoutInit(ST_CHNL600_CPT_PARA pstChnl600CptPara)
{
	ST_OXI600_LAYOUT_REG_PARA stChnl600LayInfo;
	static uint16_t u16rowOffset;

#if 0
	stChnl600LayInfo.u8mode = EN_MODE1_CPT_SGL_WIN; 				/*reg: 0xAB*/
	stChnl600LayInfo.u8colStr = 34;//CHNL600_MK810_80UM_COL_STR;			/*reg: 0xAC*/
	stChnl600LayInfo.u8border = 21;//CHNL600_MK810_80UM_BORDER;			/*reg: 0xAD*/		
	stChnl600LayInfo.u16colEnd = 550;//CHNL600_MK810_80UM_COL_END; 		/*reg: 0xAE,0xAF*/
	stChnl600LayInfo.u16rowMax = 252;//CHNL600_MK810_80UM_ROW_MAX; 		/*reg: 0xB8,0xB9*/
	stChnl600LayInfo.u16adcMask = 0xFFFF;//CHNL600_MK810_80UM_ADC_MASK;    	/*reg: 0xEC,0xED*/
	g_u16RowMax = 252;//CHNL600_MK810_80UM_ROW_MAX;
#endif
	memcpy(&stChnl600LayInfo,&g_stsensorLayout[3],sizeof(ST_OXI600_LAYOUT_REG_PARA));
	g_u16RowMax = g_stsensorLayout[3].u16rowMax;


	if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WIN_IMG)
	{
		pstChnl600CptPara.W1ColNo = 128;
		pstChnl600CptPara.W1RowNo = 128;

		if((pstChnl600CptPara.W1ColStrt + pstChnl600CptPara.W1ColNo > 472)\
			|| (pstChnl600CptPara.W1RowStrt + pstChnl600CptPara.W1RowNo > 250))
		{
			DBG("_OXIFP_IC_DRV coordinate out of range,x=%d,y=%d\n",pstChnl600CptPara.W1ColStrt,pstChnl600CptPara.W1RowStrt);
			return EN_OXI600_COOR_ERR;
		}

		stChnl600LayInfo.u16colStr1 = pstChnl600CptPara.W1ColStrt + g_stsensorLayout[3].u16colStr1; /*reg: 0xB0,0xB1*/
		stChnl600LayInfo.u16colEnd1 = stChnl600LayInfo.u16colStr1 + pstChnl600CptPara.W1ColNo;      /*reg: 0xB2,0xB3*/
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[3].u16rowStr1; /*reg: 0xE4,0xE5*/
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;      /*reg: 0xE6,0xE7*/
		g_u32WinImgDataSize = (stChnl600LayInfo.u16colEnd1 - stChnl600LayInfo.u16colStr1 + stChnl600LayInfo.u8border)*\
							(stChnl600LayInfo.u16rowEnd1 - stChnl600LayInfo.u16rowStr1)*2;
		if(g_u32WinImgDataSize > 64*1024)
		{
			DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
	#if 0	
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
		#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
		#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
	#endif	
	}
	else if(pstChnl600CptPara.enCptType == EN_MODE1_CPT_WHLE_ROW_IMG)
	{
		//pstChnl600CptPara.W1RowNo = pstChnl600CptPara.CptDataSize /2 / (g_stsensorLayout[3].u16colEnd1 \
		//				- g_stsensorLayout[3].u16colStr1 + g_stsensorLayout[3].u8border);
		//if(pstChnl600CptPara.WhlImageNo == 1)
		//{
		//	u16rowOffset = g_stsensorLayout[3].u16rowStr1;
		//}
		
		stChnl600LayInfo.u16colStr1 = g_stsensorLayout[3].u16colStr1; /*reg: 0xB0,0xB1*/
		stChnl600LayInfo.u16colEnd1 = g_stsensorLayout[3].u16colEnd1; /*reg: 0xB2,0xB3*/
		stChnl600LayInfo.u16rowStr1 = pstChnl600CptPara.W1RowStrt + g_stsensorLayout[3].u16rowStr1;//u16rowOffset;								/*reg: 0xE4,0xE5*/
		stChnl600LayInfo.u16rowEnd1 = stChnl600LayInfo.u16rowStr1 + pstChnl600CptPara.W1RowNo;/*reg: 0xE6,0xE7*/
		
		u16rowOffset += pstChnl600CptPara.W1RowNo;
		g_u32WinImgDataSize = (stChnl600LayInfo.u16colEnd1 - stChnl600LayInfo.u16colStr1 + stChnl600LayInfo.u8border)*\
								(stChnl600LayInfo.u16rowEnd1 - stChnl600LayInfo.u16rowStr1 )*2;
		if(g_u32WinImgDataSize > 64*1024)
		{
			DBG("_OXIFP_IC_DRV cac size >64K = %d\n",g_u32WinImgDataSize);
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
	#if 0	
		if(g_u32WinImgDataSize != pstChnl600CptPara.CptDataSize)
		{
		#ifdef DEBUG_DEV_OXI600
			printf("capture whole rece size = %d , cac size =%d\n",pstChnl600CptPara.CptDataSize,g_u32WinImgDataSize);
		#endif
			return EN_OXI600_IMAGE_SIZE_ERR;
		}
	#endif	
	}
	
	_dev_Oxi600_LayoutRegInit(stChnl600LayInfo);
	
	return EN_OXI600_SUCCESS;

}

/**
 * @brief Distinguish module type and initialize configuration
 * @param pstChnl600CptPara 
 * @retval EN_OXI600_ERR_TYPE
**/
static EN_OXI600_ERR_TYPE _dev_Oxi600_SensorDistinguish(ST_CHNL600_CPT_PARA pstChnl600CptPara)
{
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	switch(pstChnl600CptPara.PrjType)
	{

		case EN_PRJ_OXI600_MK720_80UM:
			enRetVal = _dev_Oxi600_MK720_80umSensorLayoutInit(pstChnl600CptPara);
			break;

		case EN_PRJ_OXI600_MK720_100UM:
			enRetVal = _dev_Oxi600_MK720_100umSensorLayoutInit(pstChnl600CptPara);
			break;

		case EN_PRJ_OXI600_MK810_80UM:
			enRetVal = _dev_Oxi600_MK810_80umSensorLayoutInit(pstChnl600CptPara);
			break;
			
		case EN_PRJ_OXI600_MK320_100UM:
			_dev_Oxi600_MK320_100umSensorLayoutInit(pstChnl600CptPara);
			break;
		
		case EN_PRJ_OXI600_MK810_80UM_04:
			_dev_Oxi600_MK810_80umSensorLayoutInit(pstChnl600CptPara);
			break;
			
		default:
			DBG("_OXIFP_IC_DRV distinguish sensor err, type = %#X\n",pstChnl600CptPara.PrjType);
			enRetVal = EN_OXI600_PROJECT_TYPE_ERR;
			break;
	}
	if(pstChnl600CptPara.shtOrLong == 0)
	{
		g_u8oxiWriteToRegBuf[87*3+1] = g_enShtInteCapacity;		/*attention:  IC capactior*/
	}
	else if(pstChnl600CptPara.shtOrLong == 1)
	{
		g_u8oxiWriteToRegBuf[87*3+1] = g_enLongInteCapacity;		/*attention:  IC capactior*/
	}
	return enRetVal;
}



/**
 * @brief All register writes corresponding data
 * @param none
 * @retval 1 or error number
 */
static EN_OXI600_ERR_TYPE _dev_Oxi600_RoicRegInit(void)
{
	uint32_t i;
	volatile uint16_t vu16DlyTime;
	uint8_t convBuf[220];
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV capture capacity = %#X \n",g_u8oxiWriteToRegBuf[87*3+1]);
	#endif
	//ROIC_CMD_RD_ST();
	//ROIC_CMD_DUMMY();
	for(i=0; i<350; i+=3)
	{
		if((g_u8oxiWriteToRegBuf[i] == 0xff)&&(g_u8oxiWriteToRegBuf[i+1] == 0xff))
		{
			break;
		}
		convBuf[i/3*2] = g_u8oxiWriteToRegBuf[i];
		convBuf[i/3*2+1] = g_u8oxiWriteToRegBuf[i+1];
	}	
	_dev_Oxi600_SpiSendMass(convBuf,(i/3+1)*2);
	
	if(_dev_Oxi600_WaitRegStatus(0x3e, 0xFF00,  0x5E00, 100) != EN_OXI600_SUCCESS)
	{
		DBG("_OXIFP_IC_DRV ROIC reg init check FSM timeout\n");
		return EN_OXI600_REG_INIT_ERR;		// error code
	}
	
	return EN_OXI600_SUCCESS;
}


/**
 * @brief Clear frame in VDD VEE or VCOM
 * @param none
 * @retval 1 or error number
 */
static EN_OXI600_ERR_TYPE _dev_Oxi600_ClrFrame(uint8_t prjType,Chl600_bool isCptGeteOff,uint32_t timeout)
{
	EN_OXI600_ERR_TYPE enRetval = EN_OXI600_SUCCESS;
	
	ROIC_CMD_WAKEUP();			/// this  is  nesscearyy   at  the begining

	if(prjType == EN_PRJ_OXI600_MK810_80UM)
	{
		g_u8Vcom2vcom = 0xD0;
		g_u8Vcom2Vdd= 0xB0;
		g_u8Vcom2Vee= 0x70;	
	}
	else if(prjType == EN_PRJ_OXI600_MK720_80UM || prjType == EN_PRJ_OXI600_MK720_100UM ||prjType == EN_PRJ_OXI600_MK320_100UM  )
	{
		g_u8Vcom2vcom = 0x50;
		g_u8Vcom2Vdd = 0x30;
		g_u8Vcom2Vee = 0xD0;
	}
	else if(prjType == EN_PRJ_OXI600_MK810_80UM_04)
	{
		g_u8Vcom2vcom = 0xC0;
		g_u8Vcom2Vdd= 0xA0;
		g_u8Vcom2Vee= 0x60; 
	}
	else
	{
		DBG("_OXIFP_IC_DRV clr frame para prj type err, type = %#X \n",prjType);
		enRetval = EN_OXI600_PROJECT_TYPE_ERR;
		return enRetval;
	}

	if(g_u8SwitchVcomFlag == EN_VCOM_TO_VDD || g_u8SwitchVcomFlag == EN_VCOM_TO_VEE || g_u8SwitchVcomFlag == EN_VCOM_TO_VCOM)
	{
		switch(g_u8SwitchVcomFlag)
		{
			
			case EN_VCOM_TO_VDD:
				VCOM_TO_VDD(g_u8Vcom2Vdd);
				XAO_H_PIRST_H();
				g_u8VcomStaus = g_u8Vcom2Vdd;
				_delay_ms(g_stOxi600clrpara.u32SwVddDelay);
				break;

			case EN_VCOM_TO_VEE:
				VCOM_TO_VEE(g_u8Vcom2Vee);
				XAO_H_PIRST_H();
				g_u8VcomStaus = g_u8Vcom2Vee;
				_delay_ms(g_stOxi600clrpara.u32SwVeeDelay);
				break;

			case EN_VCOM_TO_VCOM:
				VCOM_TO_VCOM(g_u8Vcom2vcom);
				XAO_H_PIRST_H();
				g_u8VcomStaus = g_u8Vcom2vcom;
				_delay_ms(g_stOxi600clrpara.u32SwVcomDelay);
				break;
			default:
				g_u8VcomStaus = g_u8Vcom2vcom;
				break;
		}
		return enRetval;
	}


	if(g_stOxi600clrpara.bClrLagEn == Chl600_TRUE)
	{
		if(g_stOxi600clrpara.bVddEn == Chl600_TRUE )
		{
			VCOM_TO_VDD(g_u8Vcom2Vdd);
			XAO_H_PIRST_H();
			g_u8VcomStaus = g_u8Vcom2Vdd;
			_delay_ms(g_stOxi600clrpara.u32SwVddDelay);			
			g_u8oxiWriteToRegBuf[82*3+1] = g_u8Vcom2Vdd;
			enRetval = _dev_Oxi600_ClrScan(g_stOxi600clrpara.u8VddScnCnt,g_stOxi600clrpara.u16VddFsStvCovCnt,\
									g_stOxi600clrpara.u16VddPeriod,g_u16RowMax,0,timeout);
			if(enRetval != EN_OXI600_SUCCESS)
			{
				DBG("_OXIFP_IC_DRV vdd clr scan err, err: %d\n",enRetval);
				return enRetval;
			}
		}
		
		if(g_stOxi600clrpara.bVeeEn == Chl600_TRUE )
		{
			VCOM_TO_VEE(g_u8Vcom2Vee);
			XAO_H_PIRST_H();
			g_u8VcomStaus = g_u8Vcom2Vee;
			_delay_ms(g_stOxi600clrpara.u32SwVeeDelay);
			g_u8oxiWriteToRegBuf[82*3+1] = g_u8Vcom2Vee;
			enRetval = _dev_Oxi600_ClrScan(g_stOxi600clrpara.u8VeeScnCnt,g_stOxi600clrpara.u16VeeFsStvCovCnt,\
									g_stOxi600clrpara.u16VeePeriod,g_u16RowMax,0,timeout);
			if(enRetval != EN_OXI600_SUCCESS)
			{
				DBG("_OXIFP_IC_DRV vee clr scan err, err: %d\n",enRetval);
				return enRetval;
			}
		}
		
		if(g_stOxi600clrpara.bVcomEn == Chl600_TRUE )
		{
			VCOM_TO_VCOM(g_u8Vcom2vcom);
			XAO_H_PIRST_H();
			g_u8VcomStaus = g_u8Vcom2vcom;
			_delay_ms(g_stOxi600clrpara.u32SwVcomDelay);
			g_u8oxiWriteToRegBuf[82*3+1] = g_u8Vcom2vcom;
			enRetval = _dev_Oxi600_ClrScan(g_stOxi600clrpara.u8VcomScnCnt,g_stOxi600clrpara.u16VcomFsStvCovCnt,\
									g_stOxi600clrpara.u16VcomPeriod,g_u16RowMax+20,isCptGeteOff,timeout);	
			if(enRetval != EN_OXI600_SUCCESS)
			{
				DBG("_OXIFP_IC_DRV vcom clr scan err, err: %d\n",enRetval);
				return enRetval;
			}
		}
		else
		{
			VCOM_TO_VCOM(g_u8Vcom2vcom);
			XAO_H_PIRST_H();
			g_u8VcomStaus = g_u8Vcom2vcom;
			_delay_ms(g_stOxi600clrpara.u32SwVcomDelay);
		}

	}
	else
	{
		VCOM_TO_VCOM(g_u8Vcom2vcom);
		XAO_H_PIRST_H();
		g_u8VcomStaus = g_u8Vcom2vcom;
	}

	
	
	return enRetval;
}



/**
 * @brief star capture frame
 * @param dataBuf pointer to data buffer
 * @retval 1 or error number
**/
static EN_OXI600_ERR_TYPE _dev_Oxi600_CptFrame(uint8_t prjType,uint16_t inteLine,uint32_t timeout)
{
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	uint8_t u8cptFrameNo;
	if(g_u8SwitchVcomFlag == EN_VCOM_TO_VDD || g_u8SwitchVcomFlag == EN_VCOM_TO_VEE || g_u8SwitchVcomFlag == EN_VCOM_TO_VCOM)
	{
		u8cptFrameNo = g_u8SwitchVcomFramCnt;
	}
	else
	{
		u8cptFrameNo = g_stOxi600clrpara.u8CptImgCnt;
		if(prjType == EN_PRJ_OXI600_MK810_80UM)
		{
			g_u8Vcom2vcom = 0xD0;
		}
		else if(prjType == EN_PRJ_OXI600_MK720_80UM || prjType == EN_PRJ_OXI600_MK720_100UM ||prjType == EN_PRJ_OXI600_MK320_100UM	)
		{
			g_u8Vcom2vcom = 0x50;
		}
		else if(prjType == EN_PRJ_OXI600_MK810_80UM_04)
		{
			g_u8Vcom2vcom = 0xC0;
		}
		else
		{
			DBG("_OXIFP_IC_DRV cpt frame para prj type err, type = %#X \n",prjType);
			enRetVal = EN_OXI600_PROJECT_TYPE_ERR;
			return enRetVal;
		}
		
		g_u8VcomStaus = g_u8Vcom2vcom;
	}
#if 0	
	if(u8cptFrameNo-2 > 0)
	{
		if((enRetVal = _dev_Oxi600_CptScan(u8cptFrameNo - 2,g_u16RowMax,0,dataBuf))!=EN_OXI600_SUCCESS)
		{
			return enRetVal;
		}
	}
	
	if((enRetVal = _dev_Oxi600_CptScan(1,g_stOxi600clrpara.u16IntegralFramLineCnt,0,dataBuf)) != EN_OXI600_SUCCESS)
	{
		return enRetVal;
	}
	
	if((enRetVal = _dev_Oxi600_CptScan(1,g_u16RowMax,1,dataBuf)) != EN_OXI600_SUCCESS)
	{
		return enRetVal;
	}
#else
	if((enRetVal = _dev_Oxi600_CptScan(u8cptFrameNo,inteLine,timeout)) != EN_OXI600_SUCCESS)
	{
		return enRetVal;
	}

#endif
	
	return enRetVal;
}

/**
 * @brief  ROIC peripheral driver init
 * @param  stChnl600Drv 
 * @retval EN_OXI600_ERR_TYPE value
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_DrvInit(ST_CHNL600_EXTER_DRV *stChnl600Drv)
{
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV drv inint\n");
	#endif
	_g_stCh600ExterDrv = *stChnl600Drv;
	return EN_OXI600_SUCCESS;
}


/**
 * @brief  get image data from FIFO
 * @param  dataBuf the pointer to data buffer
 * @param  dataLen the pointer to data length, Reassign datalen indicating effective data length
 * @param  timeout 
 * @retval EN_OXI600_ERR_TYPE value
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_GetImageData(uint8_t *dataBuf,uint32_t *dataLen,uint32_t timeout)
{
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV get image data \n");
	#endif

	
	if(_dev_Oxi600_WaitRegStatus(0x3f,0x8000,0x8000,timeout) != EN_OXI600_SUCCESS)
	{
		DBG("_OXIFP_IC_DRV get image data check FSM1 timeout\n");
		return EN_OXI600_DATA_READY_ERR;
	}
	
	_dev_Oxi600_GetImageDataFromFIFO(dataBuf,g_u32WinImgDataSize);
	*dataLen = g_u32WinImgDataSize;
	if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0B00,timeout) != EN_OXI600_SUCCESS)
	{
		DBG("_OXIFP_IC_DRV get image data check FSM2 timeout\n");
		return EN_OXI600_CPT_RUN_ERR;
	}

	ROIC_CMD_RESTART();
	if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0400,timeout) != EN_OXI600_SUCCESS)
	{
		DBG("_OXIFP_IC_DRV get image data check FSM3 timeout\n");
		return EN_OXI600_CPT_RESTAT_ERR;
	}

	
	XAO_L_PIRST_H();
	return EN_OXI600_SUCCESS;

}


/**
 * @brief  clear frame and output gateoff signal(output 600 chanel signal)
 * @param  stChnl600CptPara 
 * @param  dataBuf the pointer to data buffer
 * @param  dataLen the pointer to data length, Reassign datalen indicating effective data length
 * @param  isCptGateOff is capture gateoff pixel
 * @param  timeout 
 * @retval EN_OXI600_ERR_TYPE value

 EXAMPLE: 	
 	stChnl600CptPara.PrjType = XX;
 	stChnl600CptPara.isGetGateoff = TRUE / FALSE;
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_ClrAndGetGateoff(ST_CHNL600_CPT_PARA stChnl600CptPara,uint8_t *dataBuf,uint32_t *dataLen,Chl600_bool isCptGateOff,uint32_t timeout)
{
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV clr clear frame and getgateoff, is get gateoff = %d,project type = %#X\n",isCptGateOff,stChnl600CptPara.PrjType);
	#endif

	/*!!!notice!!! because of reg 0xcf default value is 0, the HV will be enable after power on, 
	 *MK810_04 capture image data is 40, So HV off before capture .
	 */
	if(stChnl600CptPara.W1RowStrt == 0 && stChnl600CptPara.PrjType ==EN_PRJ_OXI600_MK810_80UM_04)
	{
		DISABLE_ROIC_ADC(0x1f); /*ENHV OFF , IF NOT THE SIGNAL MIGHT BE 40*/
		_delay_ms(5);
	}
	
	memcpy(g_u8oxiWriteToRegBuf,g_u8Oxi600RegInitBuf,sizeof(g_u8Oxi600RegInitBuf));

	enRetVal = _dev_Oxi600_OutputGateoffLayOut(stChnl600CptPara);
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		_dev_Oxi600_logErrType("clr farme layout",enRetVal);
		return enRetVal;
	}	
#if 0	// modiify wirte reg 
	enRetVal = _dev_Oxi600_RoicRegInit();
	if(enRetVal != EN_OXI600_SUCCESS)
	{
	#ifdef DEBUG_DEV_OXI600
			_dev_Oxi600_logErrType(enRetVal);
	#endif
		return enRetVal;
	}
#endif 	
	enRetVal = _dev_Oxi600_ClrFrame(stChnl600CptPara.PrjType,isCptGateOff,timeout);
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		_dev_Oxi600_logErrType("clr farme start",enRetVal);
		return enRetVal;
	}
	if(isCptGateOff == Chl600_TRUE)
	{
		enRetVal = Dev_Oxi600_GetImageData(dataBuf,dataLen,timeout);
		if(enRetVal != EN_OXI600_SUCCESS)
		{
			_dev_Oxi600_logErrType("clr farme get image data",enRetVal);
			return enRetVal;
		}
	}
	XAO_L_PIRST_H();
	return enRetVal;
	
}


 /**
  * @brief	start capture frame scan (Start running and reply success immediately)
  * @param	stChnl600CptPara  
  * @param	timeout 
  * @retval EN_OXI600_ERR_TYPE

  EXAMPLE:
  	 stChnl600CptPara.enCptMode = EN_CHNL600_CPT_MODE;
	 stChnl600CptPara.enCptType = EN_MODE1_CPT_WIN_IMG;
	 stChnl600CptPara.integraLine = x;
	 stChnl600CptPara.shtOrLong = X;
	 stChnl600CptPara.PrjType = X;
	 stChnl600CptPara.W1ColStrt = X1;
	 stChnl600CptPara.W1RowStrt = Y1;
 **/
EN_OXI600_ERR_TYPE Dev_Oxi600_CaptureWinImage(ST_CHNL600_CPT_PARA stChnl600CptPara,uint32_t timeout)
{

	 
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;

	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV capture window image,project type:%#X,cptmode=%#X,cpttype=%#X,x1=%d,y1=%d,x2=%d,y2=%d,shtorlong:%d,inteline:%d,\n",
			stChnl600CptPara.PrjType,stChnl600CptPara.enCptMode,stChnl600CptPara.enCptType,stChnl600CptPara.W1ColStrt,stChnl600CptPara.W1RowStrt,\
			stChnl600CptPara.W2ColStrt,stChnl600CptPara.W2RowStrt,stChnl600CptPara.shtOrLong,stChnl600CptPara.integraLine);
	#endif

	memcpy(g_u8oxiWriteToRegBuf,g_u8Oxi600RegInitBuf,sizeof(g_u8Oxi600RegInitBuf));
	
	enRetVal = _dev_Oxi600_SensorDistinguish(stChnl600CptPara);
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		_dev_Oxi600_logErrType("cpt win distinguish sensor",enRetVal);
		return enRetVal;
	}
	#if 0
	//IwdgFeed();
	enRetVal = _dev_Oxi600_RoicRegInit();
	if(enRetVal != EN_OXI600_SUCCESS)
	{		
		#ifdef DEBUG_DEV_OXI600
			_dev_Oxi600_logErrType(enRetVal);
		#endif
		return enRetVal;
	}
	#endif
	

	enRetVal = _dev_Oxi600_CptFrame(stChnl600CptPara.PrjType,stChnl600CptPara.integraLine,timeout);
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		_dev_Oxi600_logErrType("cpt win cpt frame start ",enRetVal);
		return enRetVal;
	}
	


	return enRetVal;
	
}


/**
 * @brief  capture whoel image and get data 
 * @param  stChnl600CptPara  
 * @param  dataBuf the pointer to data buffer
 * @param  dataLen the pointer to data length, Reassign datalen indicating effective data length
 * @param  timeout 
 * @retval EN_OXI600_ERR_TYPE

   EXAMPLE:
  	 stChnl600CptPara.enCptMode = 0x71;
	 stChnl600CptPara.enCptType = EN_MODE1_CPT_WHLE_ROW_IMG;
	 stChnl600CptPara.integraLine = x;
	 stChnl600CptPara.shtOrLong = X;
	 stChnl600CptPara.PrjType = X;
 	 stChnl600CptPara.XaoDelay = X;
	 stChnl600CptPara.W1RowStrt = X;		
	 stChnl600CptPara.W1RowNo = X;
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_CaptureWholeRowImage(ST_CHNL600_CPT_PARA stChnl600CptPara,uint32_t timeout)
{

	
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV capture whole image,project type:%#X,cptmode=%#X,cpttype=%#X,shtorlong:%d,inteline:%d,\
			row start:%d,rowNo:%d,xao delay:%d\n",stChnl600CptPara.PrjType,stChnl600CptPara.enCptMode,stChnl600CptPara.enCptType,\
			stChnl600CptPara.shtOrLong,stChnl600CptPara.integraLine,stChnl600CptPara.W1RowStrt,stChnl600CptPara.W1RowNo,stChnl600CptPara.XaoDelay);
	#endif

	
	memcpy(g_u8oxiWriteToRegBuf,g_u8Oxi600RegInitBuf,sizeof(g_u8Oxi600RegInitBuf));
	enRetVal = _dev_Oxi600_SensorDistinguish(stChnl600CptPara);
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		_dev_Oxi600_logErrType("cpt whole distinguis",enRetVal);	
		return enRetVal;
	}
	//IwdgFeed();
#if 0	
	enRetVal = _dev_Oxi600_RoicRegInit();
	if(enRetVal != EN_OXI600_SUCCESS)
	{
	#ifdef DEBUG_DEV_OXI600
		printf("reg init err \n");
		_dev_Oxi600_logErrType(enRetVal);
	#endif	
	
		return enRetVal;
	}
	//IwdgFeed();
	enRetVal = _dev_Oxi600_ClrFrame(stChnl600CptPara.PrjType,FALSE);	
	if(enRetVal != EN_OXI600_SUCCESS)
	{
	#ifdef DEBUG_DEV_OXI600
		printf("clr err \n");
		_dev_Oxi600_logErrType(enRetVal);
	#endif
	
		return enRetVal;
	}
#endif	

	if(stChnl600CptPara.XaoDelay != 0)
	{
		XAO_L_PIRST_H();
		_delay_ms(stChnl600CptPara.XaoDelay);
	}
	
	//IwdgFeed();
	enRetVal = _dev_Oxi600_CptFrame(stChnl600CptPara.PrjType,stChnl600CptPara.integraLine,timeout);
	if(enRetVal != EN_OXI600_SUCCESS)
	{
		_dev_Oxi600_logErrType("cpt whole cpt frame start",enRetVal);
	
		return enRetVal;
	}
#if 0	
	enRetVal = Dev_Oxi600_GetImageData(dataBuf,dataLen,timeout);
	if(enRetVal != EN_OXI600_SUCCESS )
	{
		return enRetVal;
	}
#endif
	return enRetVal;
}

/**
 * @brief  capture whoel image and get data (using in TEE)
 * @param  stChnl600CptPara  
 * @param  dataBuf the pointer to data buffer
 * @param  dataLen the pointer to data length,Reassign datalen indicating effective data length
 * @param  timeout 
 * @retval EN_OXI600_ERR_TYPE

EXAMPLE:
  stChnl600CptPara.enCptMode = 0x71;
  stChnl600CptPara.enCptType = EN_MODE1_CPT_WHLE_ROW_IMG;
  stChnl600CptPara.integraLine = x;
  stChnl600CptPara.shtOrLong = X;
  stChnl600CptPara.PrjType = X;
  stChnl600CptPara.XaoDelay = X;
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_TeeCptWholeRowImage(ST_CHNL600_CPT_PARA stChnl600CptPara,uint8_t *dataBuf,uint32_t *dataLen,uint32_t timeout)
{	
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	uint8_t i,cptWhlFrmNo,cptRowNo,cptLastRowNo;
	uint32_t rowOfset,bufOfset;
	rowOfset = 0;
	bufOfset = 0;
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV capture tee whole image\n");
	#endif	
	
	switch(stChnl600CptPara.PrjType)
	{
		case EN_PRJ_OXI600_MK720_80UM:
			cptWhlFrmNo = 4;
			cptRowNo = 73;
			cptLastRowNo = 73;
			break;
			
		case EN_PRJ_OXI600_MK720_100UM:
			cptWhlFrmNo = 3;
			cptRowNo = 87;
			cptLastRowNo = 54;
			break;
			
		case EN_PRJ_OXI600_MK810_80UM:
			cptWhlFrmNo = 4;
			cptRowNo = 63;
			cptLastRowNo = 61;
			break;
			
		case EN_PRJ_OXI600_MK320_100UM:
			cptWhlFrmNo = 4;
			cptRowNo = 63;
			cptLastRowNo = 61;
			break;
			
		case EN_PRJ_OXI600_MK810_80UM_04:
			cptWhlFrmNo = 4;
			cptRowNo = 63;
			cptLastRowNo = 61;
			break;

		default:
			DBG("_OXIFP_IC_DRV capture tee whole image para err type = %d \n",stChnl600CptPara.PrjType);
			return EN_OXI600_PARA_ERR;
			break;
		
	}
	
	for(i = 0;i < cptWhlFrmNo;i++)
	{
		if(i == cptWhlFrmNo -1)
		{
			stChnl600CptPara.W1RowStrt = rowOfset;
			stChnl600CptPara.W1RowNo = cptLastRowNo;
			rowOfset += stChnl600CptPara.W1RowNo;
		}
		else
		{
			stChnl600CptPara.W1RowStrt = rowOfset;
			stChnl600CptPara.W1RowNo = cptRowNo;
			rowOfset += cptRowNo;
		}
		
		enRetVal = Dev_Oxi600_ClrAndGetGateoff(stChnl600CptPara,dataBuf,dataLen,Chl600_FALSE,1000);
		if(enRetVal != EN_OXI600_SUCCESS )
		{
			_dev_Oxi600_logErrType("tee cpt whole clr",enRetVal);
			return enRetVal;
		}

		enRetVal = Dev_Oxi600_CaptureWholeRowImage(stChnl600CptPara,timeout);
		if(enRetVal != EN_OXI600_SUCCESS )
		{
			_dev_Oxi600_logErrType("tee cpt whole cpt start",enRetVal);
			return enRetVal;
		}
		
		enRetVal = Dev_Oxi600_GetImageData(dataBuf+bufOfset,dataLen,timeout);
		if(enRetVal != EN_OXI600_SUCCESS )
		{	
			_dev_Oxi600_logErrType("tee cpt whole getdata",enRetVal);
			return enRetVal;
		}
		bufOfset += *dataLen;
	}
	*dataLen = bufOfset;

    return enRetVal;
}


/**
 * @brief  switch Vcom to XXX and disable clear frame 
 * @param  vcomMod  Vcom to xxx  
 * @param  framNo  captrue scan frame count
 * @param  timeout 
 * @retval EN_OXI600_ERR_TYPE
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_SwitchVcomVol(EN_CHNL600_VCOM_MODE vcomMod,uint8_t framNo)
{
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV switch vcom mode = %d,frame = %#X\n",vcomMod,framNo);
	#endif

	if(vcomMod == EN_RESET_TO_DEFAULT || vcomMod == EN_VCOM_TO_VDD || \
		vcomMod == EN_VCOM_TO_VEE || vcomMod == EN_VCOM_TO_VCOM)
	{
		g_u8SwitchVcomFlag = vcomMod;
		g_u8SwitchVcomFramCnt = framNo;
	}
	else
	{
		DBG("_OXIFP_IC_DRV switch vcom para err:mod = %d,framNo = %d\n",vcomMod,framNo);
		enRetVal = EN_OXI600_PARA_ERR;
	}
	

	return enRetVal;
}

/**
 * @brief  sleep ROIC disale ADC  
 * @param  timeout 
 * @retval EN_OXI600_ERR_TYPE
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_SleepROIC(ST_CHNL600_CPT_PARA stChnl600CptPara,uint32_t timeout)
{
	EN_OXI600_ERR_TYPE enRetVal = EN_OXI600_SUCCESS;
	
#ifdef DEBUG_DEV_OXI600
	DBG("_OXIFP_IC_DRV sleep ROIC,project type = %#X \n",stChnl600CptPara.PrjType);
#endif
	ROIC_CMD_RESTART();

	ROIC_CMD_SLEEP();
	XAO_L();
	if(stChnl600CptPara.PrjType == EN_PRJ_OXI600_MK810_80UM_04)
	{
		DISABLE_ROIC_ADC(0x1f);	/*ENHV LOW LEVEL WORKING*/
	}
	else
	{
		DISABLE_ROIC_ADC(0x0f);	/*ENHV HIGH LEVEL WORKING*/
	}
	if(_dev_Oxi600_WaitRegStatus(0x3f,0x0F00,0x0000,timeout) != EN_OXI600_SUCCESS)
	{
		DBG("_OXIFP_IC_DRV sleep ROIC check FSM timeout\n");
		return EN_OXI600_SLEEP_ROIC_ERR;
	}
	return enRetVal;	
}

/**
 * @brief  set PMU parameter
 * @param   dataBuf the pointer to data buffer
 * @param   dataLen the pointer to data length, indicate effective data length
 * @retval EN_OXI600_ERR_TYPE
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_setPmuPara(uint8_t *dataBuf, uint32_t dataLen)
{
	#ifdef DEBUG_DEV_OXI600
		DBG("_OXIFP_IC_DRV set PMU paramter\n");
	#endif
	memcpy(g_u8Oxi600RegInitBuf,dataBuf,dataLen);
	return EN_OXI600_SUCCESS;
}

EN_OXI600_ERR_TYPE Dev_Oxi600_getPmuPara(uint8_t *dataBuf, uint32_t *dataLen)
{
		DBG("_OXIFP_IC_DRV get PMU paramter\n");
		for(int i = 0;i <= 103 ;i++ )
		{
			DBG("%X %X %X \n",g_u8Oxi600RegInitBuf[i*3+0],g_u8Oxi600RegInitBuf[i*3+1],g_u8Oxi600RegInitBuf[i*3+2]);
		}
		DBG("_OXIFP_IC_DRV get PMU para end\n");
		
		*dataLen = sizeof(g_u8Oxi600RegInitBuf);
		memcpy(dataBuf,g_u8Oxi600RegInitBuf,*dataLen);
		return EN_OXI600_SUCCESS;
}


/**
 * @brief  set capture flow:Vdd Vee Vcom capture counte ....
 * @param  timeout 
 * @retval 
**/
EN_OXI600_ERR_TYPE Dev_Oxi600_setCptPara(uint8_t *dataBuf, uint32_t dataLen)
{
	int i = 0;
	ST_OXI600_CLR_PARA cptPara;
	if(dataBuf[0] == 2)
	{
		#ifdef DEBUG_DEV_OXI600
			DBG("_OXIFP_IC_DRV set cpt paramter flag choose disable\n");
		#endif
		return EN_OXI600_SUCCESS;
	}
		
	cptPara.bClrLagEn = dataBuf[0];
	cptPara.bVddEn= dataBuf[1];
	cptPara.u8VddScnCnt = dataBuf[2];
	cptPara.u16VddFsStvCovCnt = ((uint16_t)dataBuf[3]<<8)|((uint16_t)dataBuf[4]);
	cptPara.u32SwVddDelay = ((uint32_t)dataBuf[5]<<24)|((uint32_t)dataBuf[6]<<16)|\
							((uint32_t)dataBuf[7]<<8)|((uint32_t)dataBuf[8]);
	
	cptPara.bVeeEn= dataBuf[9];
	cptPara.u8VeeScnCnt = dataBuf[10];
	cptPara.u16VeeFsStvCovCnt = ((uint16_t)dataBuf[11]<<8)|((uint16_t)dataBuf[12]);
	cptPara.u32SwVeeDelay = ((uint32_t)dataBuf[13]<<24)|((uint32_t)dataBuf[14]<<16)|\
							((uint32_t)dataBuf[15]<<8)|((uint32_t)dataBuf[16]);

	cptPara.bVcomEn= dataBuf[17];
	cptPara.u8VcomScnCnt = dataBuf[18];
	cptPara.u16VcomFsStvCovCnt = ((uint16_t)dataBuf[19]<<8)|((uint16_t)dataBuf[20]);
	cptPara.u32SwVcomDelay = ((uint32_t)dataBuf[21]<<24)|((uint32_t)dataBuf[22]<<16)|\
							((uint32_t)dataBuf[23]<<8)|((uint32_t)dataBuf[24]);

	cptPara.u8CptImgCnt = dataBuf[29];
	if(!(dataBuf[30] == 0xff && dataBuf[31] == 0xff))
	{
		g_enShtInteCapacity = dataBuf[30];
		g_enLongInteCapacity = dataBuf[31];
	}
	
	g_stOxi600clrpara = cptPara;

	#ifdef DEBUG_DEV_OXI600
	DBG("_OXIFP_IC_DRV set cpt paramter\n");
	DBG("clr en:%d, vdd en:%d, vdd cnt:%d, vdd cov:%d, vdd delya:%d , vee en:%d, vee cnt:%d, vee cov:%d,\
	vee delya:%d, vcom en:%d, vcom cnt:%d, vcom cov%d, vcom delya:%d,cpt cnt:%d,sht cap:%#X,long cap:%#X \n",cptPara.bClrLagEn,\
			cptPara.bVddEn,cptPara.u8VddScnCnt,cptPara.u16VddFsStvCovCnt,cptPara.u32SwVddDelay,cptPara.bVeeEn,cptPara.u8VeeScnCnt,\
			cptPara.u16VeeFsStvCovCnt,cptPara.u32SwVeeDelay,cptPara.bVcomEn,cptPara.u8VcomScnCnt,cptPara.u16VcomFsStvCovCnt,\
			cptPara.u32SwVcomDelay,cptPara.u8CptImgCnt,g_enShtInteCapacity,g_enLongInteCapacity);
	#endif


}


uint16_t Dev_Oxi600_ReadRegVal(uint8_t regAddr)
{
	return _dev_Oxi600_ReadReg(regAddr);
}

/*
* @brief  get lib version 
* @param  pointer to buffer 4 bytes 
* @retval none
*/
EN_OXI600_ERR_TYPE Dev_Oxi600_getLibVer(uint8_t* dataBuf)
{
	#ifdef DEBUG_DEV_OXI600
		DBG("get ROIC drv lib version:%d,%d,%d,%d\n",u8LibVer[0],u8LibVer[1],u8LibVer[2],u8LibVer[3]);
	#endif
	memcpy(dataBuf,u8LibVer,4);
	return EN_OXI600_SUCCESS;
}





#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "type.h"
#include "data.h"

#define BASCAL_CRC_VAL				123

/*
	the shift byte in data buffer for calculate verification value:
		package header(2 bytes) + platform type(2 bytes) + project type(1 bytes) +  sub-project type(1 byte) = 6 byte
	
*/
#define BUF_CAL_VERIFY_SHIFT		6     

#define CMD_PARA_LEN				512

#ifdef DEBUG_MODE_EN
#define PACK_HEADER					0xF000
#else
#define PACK_HEADER					0xEF01
#endif

/*platform type*/
typedef enum _EN_PFRM_TYPE
{
	EN_PFRM_STM32F407				= 0xFF01,
	EN_PFRM_STM32F412				= 0xFF02, 
	EN_PFRM_LOCK					= 0xFFFF,

}EN_PFRM_TYPE;

/*project type*/
typedef enum _EN_PRJ_TYPE
{
	EN_PRJ_G2000_MIN				= 0x01,
	EN_PRJ_G2000_01					= EN_PRJ_G2000_MIN, 
	EN_PRJ_G2000_MAX				= 0x1F,
	
	EN_PRJ_CMOS_MIN					= 0x20,
	EN_PRJ_CMOS_01 					= EN_PRJ_CMOS_MIN, 
	EN_PRJ_CMOS_MAX					= 0x3F,

	EN_PRJ_OXI600_MIN				= 0x40,
	//EN_PRJ_OXI600_MK720_80UM		= EN_PRJ_OXI600_MIN, 
	//EN_PRJ_OXI600_MK720_100UM		= EN_PRJ_OXI600_MIN + 1, 
	//EN_PRJ_OXI600_MK810_80UM		= EN_PRJ_OXI600_MIN + 2,
	//EN_PRJ_OXI600_MK320_100UM		= EN_PRJ_OXI600_MIN + 3,
	//EN_PRJ_OXI600_MK810_80UM_04	= EN_PRJ_OXI600_MIN + 4,
	EN_PRJ_OXI600_MAX				= 0x5F,

}EN_PRJ_TYPE;

typedef enum _EN_PACK_ID
{
	EN_PACK_ID_CMD       = 0x01, /*the CMD package from host send to platform*/
	EN_PACK_ID_RESP		 = 0x07, /*the respond package from platform to host*/
	EN_PACK_ID_DATA      = 0x08, /*data package that not last package*/
	EN_PACK_ID_DATA_LAST = 0x0F, /*data package that last package*/
	
}EN_PACK_ID;

typedef struct
{
	u16 u16PfrmType; /*platform type*/
	u8 u8PrjType;    /*project type*/
	u8 u8SubPrjType; /*sub-project type*/
	u8 u8Identify;   /*package identify*/
	u16 u16PackLen;  /*package length*/
	u8 u8Cmd;        /*command*/
	u8 u8Rslt;        /*respond rslt*/
	u16 u16ParaLen;  /*parameter length*/
	u8 u8CmdParaBuf[CMD_PARA_LEN]; /*the buffer for saveing CMD parameters*/
	u8* pu8DataBufPointer;/*the pointer for pointer to data buffer pointer*/
	u16 u16RcvChecksum; /*receive checksum from CMMD package*/
	
}ST_PRTCL_INFO;

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~receive CMD start~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/***************the project OXI600 CMD list start*********************************************/
typedef enum _EN_OXI600_CMD
{
	/*image data option*/
	EN_OXI600_CMD_IMG_DATA_BASE					= 0x00,
	EN_OXI600_CMD_CAPTURE_WIN_IMAGE				= EN_OXI600_CMD_IMG_DATA_BASE+0x01,
	EN_OXI600_CMD_CAPTURE_GAIN_IMAGE			= EN_OXI600_CMD_IMG_DATA_BASE+0x02, 
	EN_OXI600_CMD_IMG_DATA_MAX 					= 0x1F,

	/*capture parameters set/get option*/
	EN_OXI600_CMD_CPT_PARA_BASE					= 0x20,
	EN_OXI600_CMD_SET_INTEGRAL_LINE_CNT			= EN_OXI600_CMD_CPT_PARA_BASE,/*set intergral line*/
	EN_OXI600_CMD_GET_INTEGRAL_LINE_CNT 		= EN_OXI600_CMD_CPT_PARA_BASE+0x01,/*set intergral line*/
	EN_OXI600_CMD_SET_LINETIME					= EN_OXI600_CMD_CPT_PARA_BASE+0x02,
	EN_OXI600_CMD_GET_LINETIME					= EN_OXI600_CMD_CPT_PARA_BASE+0x03,
	EN_OXI600_CMD_SET_CLR_LAG_FLOW				= EN_OXI600_CMD_CPT_PARA_BASE+0x04, 
	EN_OXI600_CMD_GET_CLR_LAG_FLOW				= EN_OXI600_CMD_CPT_PARA_BASE+0x05, 
	EN_OXI600_CMD_SET_CPT_CNT					= EN_OXI600_CMD_CPT_PARA_BASE+0x06, 
	EN_OXI600_CMD_GET_CPT_CNT					= EN_OXI600_CMD_CPT_PARA_BASE+0x07, 
	EN_OXI600_CMD_SWITCH_VCOM_VOL				= EN_OXI600_CMD_CPT_PARA_BASE+0x08,/*swtich VCOM*/
	EN_OXI600_CMD_SET_PMU_PARA					= EN_OXI600_CMD_CPT_PARA_BASE+0x09,/*set PMU paramter*/
	EN_OXI600_CMD_RD_PMU_REG					= EN_OXI600_CMD_CPT_PARA_BASE+0x0A,/*get PMU register value*/
	EN_OXI600_CMD_LOW_POWER_MODE				= EN_OXI600_CMD_CPT_PARA_BASE+0x0B,/*sleep ROIC and turn off HV*/
	EN_OXI600_CMD_CPT_PARA_MAX 					= 0x3F,

	/*coms cmd*/
	EN_CMOS_CMD_BASE							= 0x40,
	EN_CMOS_CMD_CAPTURE_WIN_IMAGE				= EN_CMOS_CMD_BASE + 0x00,
	EN_CMOS_CMD_SET_INTEGRAL_LINE_CNT			= EN_CMOS_CMD_BASE + 0x01,
	EN_CMOS_CMD_SET_PMU_PARA					= EN_CMOS_CMD_BASE + 0x02,
	EN_CMOS_CMD_READ_REGS						= EN_CMOS_CMD_BASE + 0x03,
	EN_CMOS_CMD_WRITE_REGS						= EN_CMOS_CMD_BASE + 0x04,
	EN_CMOS_CMD_SET_CMOS_STATUS					= EN_CMOS_CMD_BASE + 0x05,
	
	
	/*test board universal peripherals*/
	EN_BSP_CMD_BASE								= 0x50,
	EN_BSP_SET_VOL								= EN_BSP_CMD_BASE+0x01,
	EN_BSP_READ_VOL 							= EN_BSP_CMD_BASE+0x02,
	EN_BSP_ELEC_CALI							= EN_BSP_CMD_BASE+0x03,
	EN_BSP_READ_CRUURENT						= EN_BSP_CMD_BASE+0x04,
	EN_BSP_POWERON_OFF							= EN_BSP_CMD_BASE+0x05,
	EN_BSP_BACKLIGHT_CTL						= EN_BSP_CMD_BASE+0x06,   
	EN_BSP_WRITE_FLASH							= EN_BSP_CMD_BASE+0x08,
	EN_BSP_READ_FLASH							= EN_BSP_CMD_BASE+0x09,
	EN_BSP_GET_FLASH_ID							= EN_BSP_CMD_BASE+0x0A,
	EN_BSP_MONITORING_KEY						= EN_BSP_CMD_BASE+0x0B,	
	EN_BSP_ERASE_FLASH							= EN_BSP_CMD_BASE+0x0C,	
	EN_BSP_GET_FW_VERSION						= EN_BSP_CMD_BASE+0x0E,
	EN_BSP_UPDATE_FW							= EN_BSP_CMD_BASE+0x0F,
	EN_BSP_CMD_MAX								= 0x6F,

	/*test communication*/
	EN_OXI600_CMD_HANDSHAKE						= 0xF0,
	
}EN_OXI600_CMD;



/***************the project cmos CMD list start*********************************************/

typedef enum _oxi_cmos_status_type_t
{
	CMOS_RESET =	0x10,
	CMOS_WAKEUP =	0x20,
	CMOS_SLEEP =	0x30,
	CMOS_RESTART=	0x40,
}oxi_cmos_status_type_t;


/******************************** package header Instances *******************************/
#define IS_PACK_HEADER_INSTANCE(INSTANCE) ((INSTANCE) == PACK_HEADER) 
								  

/******************************** platform Instances *******************************/
#define IS_PFRM_INSTANCE(INSTANCE) (((INSTANCE) == EN_PFRM_STM32F407) || \
								    ((INSTANCE) == EN_PFRM_STM32F412) || \
									((INSTANCE) == EN_PFRM_LOCK))

/******************************** project Instances *******************************/
#define IS_PRJ_INSTANCE(INSTANCE) ((((INSTANCE) >= EN_PRJ_G2000_MIN)&&((INSTANCE) <= EN_PRJ_G2000_MAX)) || \
								    (((INSTANCE) >= EN_PRJ_CMOS_MIN)&&((INSTANCE) <= EN_PRJ_CMOS_MAX)) || \
									(((INSTANCE) >= EN_PRJ_OXI600_MIN)&&((INSTANCE) <= EN_PRJ_OXI600_MAX)))

/******************************** receive from host package identify Instances *******************************/
#define IS_RCVE_PACK_ID_INSTANCE(INSTANCE) (((INSTANCE) == EN_PACK_ID_CMD) || \
								        	((INSTANCE) == EN_PACK_ID_DATA) || \
									    	((INSTANCE) == EN_PACK_ID_DATA_LAST))
									    
/******************************** prepare to send to host package identify Instances *******************************/
#define IS_SEND_PACK_ID_INSTANCE(INSTANCE) (((INSTANCE) == EN_PACK_ID_RESP) || \
								        	((INSTANCE) == EN_PACK_ID_DATA) || \
									    	((INSTANCE) == EN_PACK_ID_DATA_LAST))



/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~receive CMD end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~respond start~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

typedef enum _EN_RESP_TYPE
{
	EN_RESP_SUCCESS 						= 0x00,

	/*image capture error*/
	EN_PESP_CPT_ERR_BASE					= 0x01,
	EN_RESP_CPT_ERR							= EN_PESP_CPT_ERR_BASE,
	EN_RESP_CPT_ERR_MAX						= 0x4F,

	/*alogrithm error,only used in door lock right now*/
	EN_RESP_ALGO_ERR_BASE					= 0x50,
	EN_RESP_ALGO_ERR_MAX					= 0x8F,

	/*communication error*/
	EN_RESP_COM_ERR_BASE					= 0x90,
	EN_RESP_CMO_ERR_PACK_HEADER				= EN_RESP_COM_ERR_BASE,
	EN_RESP_CMO_ERR_PFRM_TYPE				= EN_RESP_COM_ERR_BASE+1,
	EN_RESP_CMO_ERR_PRJ_TYPE				= EN_RESP_COM_ERR_BASE+2,
	EN_RESP_CMO_ERR_PACK_ID 				= EN_RESP_COM_ERR_BASE+3,
	EN_RESP_CMO_ERR_CHECKUM 				= EN_RESP_COM_ERR_BASE+4,
	EN_RESP_CMO_ERR_INVALID_CMD 			= EN_RESP_COM_ERR_BASE+5,
	EN_RESP_CMO_ERR_INVALID_PARA 			= EN_RESP_COM_ERR_BASE+6,
	EN_RESP_CMO_ERR_RCV_TIMEOUT				= EN_RESP_COM_ERR_BASE+7,
	EN_RESP_CMO_ERR_RCV_LAST_DATA_PACK 		= EN_RESP_COM_ERR_BASE+8,
	EN_RESP_COM_ERR_MAX						= 0xFE,

	/*system error*/
	EN_RESP_SYS_ERR_BASE					= 0xD0,
	EN_RESP_SYS_ERR_MALLOC_FAIL				= EN_RESP_SYS_ERR_BASE,
	EN_RESP_SYS_FB_FUNC_RUN_ERR 			= EN_RESP_SYS_ERR_BASE+1,
	EN_RESP_SYS_ST_FLASH_OPT_ERR			= EN_RESP_SYS_ERR_BASE+2,
	EN_RESP_SYS_PERI_FLASH_NOT_EXSIT		= EN_RESP_SYS_ERR_BASE+3,
	EN_RESP_SYS_ERR_MAX						= 0xFE,
	
}EN_RESP_TYPE;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~respond end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~interface variable start~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~interface variable end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~interface function start~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void prtcl_respPreproc(ST_PRTCL_INFO* pstRespInfo, ST_PRTCL_INFO* pstRcvInfo,
								EN_PACK_ID packID, u8 rslt, u16 paraLen, u8* paraBuf );
extern EN_RESP_TYPE prtcl_rcvPackParsing(ST_PRTCL_INFO* pstRcvInfo, u8* RcvBuf);
u16 prtcl_respPackPacketization(u8* DstBuf, ST_PRTCL_INFO* pstRespInfo);
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~interface function end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/



#endif /* __PROTOCOL_H */


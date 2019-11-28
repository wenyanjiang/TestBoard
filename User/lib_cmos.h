#ifndef __LIB_CMOS_H__
#define __LIB_CMOS_H__

#include "type.h"
#include "lib_oxi600.h"
#define OXIStatusGroup_Generic  -800
typedef enum _oxi_status
{
	/* the generic status following HAL_StatusTypeDef */
	OXI_Success = 0,//为方便统一，错误改为负数
	OXI_Failed = OXIStatusGroup_Generic - 1,
	OXI_Busy = OXIStatusGroup_Generic - 2,
	OXI_Timeout = OXIStatusGroup_Generic - 3,
	/* the generic status new adding */
	OXI_OutOfRange = OXIStatusGroup_Generic - 4,
	OXI_InvalidArgument = OXIStatusGroup_Generic - 5,

	OXI_Write_CMD_Failed = OXIStatusGroup_Generic - 10,
	OXI_READ_CMD_RES_Failed = OXIStatusGroup_Generic - 11,
	OXI_RES_CRC_err = OXIStatusGroup_Generic - 12,
	OXI_RES_LEN_err = OXIStatusGroup_Generic - 13,
	OXI_RES_state_err = OXIStatusGroup_Generic - 14,
	OXI_Write_DATA_Failed = OXIStatusGroup_Generic - 15,
	OXI_READ_DATA_Failed = OXIStatusGroup_Generic - 16,
	OXI_READ_DATA_check_Failed = OXIStatusGroup_Generic - 17,

	OXI_unsupport_fun = OXIStatusGroup_Generic - 18,
}oxi_cmos_status_t;


#if 1 
/*
*   Internal function declaration 
*/
oxi_cmos_status_t Dev_Cmos_Regs_Update(void);
oxi_cmos_status_t Dev_Cmos_WakeUp(void);
oxi_cmos_status_t Dev_Cmos_Restart(void);
oxi_cmos_status_t Dev_Cmos_SetIntTime(u8* Buf);
oxi_cmos_status_t Dev_Cmos_Cpt(u8* ParaBuf);

#endif

oxi_cmos_status_t Dev_CMOS_setPmuPara(uint8_t *dataBuf, uint32_t dataLen);
oxi_cmos_status_t Dev_CMOS_SleepROIC(ST_CHNL600_CPT_PARA stChnl600CptPara,uint32_t timeout);
oxi_cmos_status_t Dev_CMOS_CaptureWinImage(ST_CHNL600_CPT_PARA stChnl600CptPara,uint32_t timeout);
oxi_cmos_status_t Dev_CMOS_RoicRegInit(void);

/*
*	Extern Funtion Define 
*   To successfully execute functions in lib, you need to provide a function definition 
*/
extern void Dev_Oxi600_WriteReg(uint8_t regAddr,uint8_t regData); /*wirte 1 byte*/
extern uint16_t Dev_Oxi600_ReadReg(uint8_t RegAddr); /*read 1 byte*/
extern uint32_t Dev_Oxi600_GetLocalTime(void); /*计时*/
extern void delay_ms(uint32_t n); 
extern void Dev_Oxi600_SPI_ReceiveContinuous(uint8_t *dataBuf,uint32_t dataLen);	/*receive buf*/
extern void Dev_Oxi600_SPI_SendContinuous(uint8_t *dataBuf,uint32_t dataLen);  /*write buf */

void Dev_Cmos_Reset(u16 KpLowTim);

#endif

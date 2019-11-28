#ifndef __BRD_APP_OXI600_H
#define __BRD_APP_OXI600_H

#include "type.h"
#include "stm32f4xx_hal.h"
#include "protocol.h"
#include "lib_oxi600.h"
#include "lib_oxi600_jigsaw.h"


extern uint8_t g_u8Oxi600RegInitBuf[370];
extern  SPI_HandleTypeDef hspi1;

void Brd_Oxi600_DrvInit(void);
void Brd_Oxi600_SetIntegrationLine(uint16_t InteLIne);
uint16_t Brd_Oxi600_GetIntergrationLine(void);
EN_OXI600_ERR_TYPE Brd_Oxi600_setPmuPara(ST_PRTCL_INFO rcvDataInfo, ST_PRTCL_INFO rcvCmdInfo, u32 addrOffset);
EN_OXI600_ERR_TYPE Brd_Oxi600_setClrPara(ST_PRTCL_INFO rcvDataInfo, ST_PRTCL_INFO rcvCmdInfo, u32 addrOffset);
EN_OXI600_ERR_TYPE Brd_Oxi600_CaptureWinImg(ST_CHNL600_CPT_PARA stChnl600CptPara,uint8_t *dataBuf,uint32_t *datalen);


#endif


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */
#include "type.h"

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */
  
/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */ 

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */
/*enum: platform(as test board) USB interface status type*/
typedef enum _EN_CDC_HS_STAT_TYPE
{
	EN_USB_STAT_RCV_CMD				= 0x01,
	EN_USB_STAT_RCV_DATA 			= 0x02,

}EN_CDC_HS_STAT_TYPE;


/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */
#define USB_RX_BUF_DFALT_LEN		2048
#define USB_TX_BUF_MAX_LEN			2048
/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern uint8_t* g_pu8UsbRcvDataBuf;
extern uint8_t g_u8UsbCmdRcvBuf[USB_RX_BUF_DFALT_LEN];
extern uint8_t g_u8UsbSendBuf[USB_TX_BUF_MAX_LEN];
extern uint32_t g_u32UsbRcvLen;
extern volatile uint32 g_UsbRcvEpPackCnt;

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */
extern uint8_t CDC_Transmit_HS_Timeout(uint8_t* Buf, uint16_t Len, u32 timeoutMs);
extern u32 CDC_GetRxLen_Hs(void);
extern void CDC_ClrRxLen_Hs(void);
extern bool CDC_SetRxStat_Hs(EN_CDC_HS_STAT_TYPE enStats);
extern EN_CDC_HS_STAT_TYPE CDC_GetRxStat_Hs(void);

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

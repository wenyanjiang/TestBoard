#include <string.h>
#include <stdio.h>

#include "app_oxi.h"
#include "app_usb.h"

#include "dev_usb.h"

#include "heap_5.h"

#include "usbd_cdc_if.h"


void App_Oxi_TestCodeRun(void)
{
	volatile static u32 su32tick =0, su32FlowCnt = 0;
	volatile static bool sbFlagLedOn = TRUE;
	static bool sbFlagIncres = TRUE;
	static bool sbFlagWhileLedOn = TRUE;
	static bool sbFlagFstRun = TRUE;

	HAL_Delay(1);
	su32tick++;
	if(su32tick% 500 == 0)
	{
	#if 0
		//×¢ÒâHAL_Delay(1)ÊÇ2ms
		static int su32tick_xx=0;

		printf("WhileLedOn -- %d,Incres -- %d, tick -- %d\r\n",sbFlagWhileLedOn,sbFlagIncres,su32tick_xx);
		if(sbFlagWhileLedOn == TRUE)
		{
			pwm_led_set(1,su32tick_xx);
			pwm_led_set(2,0);
		}
		else if(sbFlagWhileLedOn == FALSE)
		{
			pwm_led_set(2,su32tick_xx);
			pwm_led_set(1,0);
		}

		if(sbFlagFstRun == FALSE)
		{
			if((su32tick_xx == 100)||(su32tick_xx == 0))
			{
				sbFlagIncres = !sbFlagIncres;
				su32FlowCnt++;
			}
		}

		sbFlagFstRun = FALSE;

		if(su32FlowCnt == 2)
		{
			su32FlowCnt = 0;
			sbFlagWhileLedOn = !sbFlagWhileLedOn;
			sbFlagFstRun  = TRUE;
			return ;
		}

		if(sbFlagIncres == TRUE)
		{
			su32tick_xx++;
		}
		else if(sbFlagIncres == FALSE)
		{
			su32tick_xx--;
		}
	#endif	
		
		if(sbFlagLedOn == TRUE)
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
			sbFlagLedOn = FALSE;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);			
			sbFlagLedOn = TRUE;
		}
	}
}


void App_Oxi_TestUSBCommunication(void)
{
	static bool sbFlagFrstRun = TRUE;
	u32 i;
	u8 u8UsbSendBuf[] = {0xf0, 0x00 ,0xff, 0xff};
	u8 UsbComResult;

	if(sbFlagFrstRun == TRUE)
	{
		sbFlagFrstRun = FALSE;
		Dev_Usb_remallocRxBuf(70*1024);
		DBG("malloc 70k for usb rx buffer,then MCU free ram:%d\r\n",xPortGetFreeHeapSize());
	}
	
	if((g_u32UsbRcvLen != 0)||(g_UsbRcvEpPackCnt != 0))
	{
		DBG("rcv en pack cnt:%d,u32RcvLen = %d\r\n",g_UsbRcvEpPackCnt,g_u32UsbRcvLen);
		g_UsbRcvEpPackCnt = 0;
		for(i=0; i<g_u32UsbRcvLen; i++ )
		{
			DBG("%02X ", g_u8UsbCmdRcvBuf[i]);
		}
		DBG("\r\n");
		g_u32UsbRcvLen = 0;
		UsbComResult = CDC_Transmit_HS(u8UsbSendBuf, 64);
		DBG("result:%d\r\n",UsbComResult);
		
	}

}


void App_Oxi_Task(void)
{
	App_Usb_intfcProc();
}


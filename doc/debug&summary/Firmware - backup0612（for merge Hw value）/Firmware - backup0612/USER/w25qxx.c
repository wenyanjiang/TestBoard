#include "w25qxx.h" 
//#include "spi.h"
#include "delay.h"	   
//#include "usart.h"	

 
u16 W25QXX_TYPE=W25Q128;	//Ĭ����W25Q128

extern uint8_t Sdata[];
//extern uint8_t WakeUpReg;
//extern uint8_t Reg9d1,Reg9d2;
//extern uint8_t Reg9e1,Reg9e2;
extern __IO uint16_t VMOD1,VMOD2,BUF;
extern  uint8_t USB_Rx_Buffer[];
extern  __IO uint8_t USB_StatusDataSended;
extern __ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q128
//����Ϊ16M�ֽ�,����128��Block,4096��Sector 
													 
//��ʼ��SPI FLASH��IO��
//void W25QXX_Init(void)
//{ 
//	SPI1_SetSpeed(SPI_BaudRatePrescaler_4);		//����Ϊ21Mʱ��,����ģʽ 
//	W25QXX_TYPE=W25QXX_ReadID();	//��ȡFLASH ID.
//}  
uint8_t Flash_Judgment(void)
{
	uint8_t FlashON;
	PWRON_1117;
	delay_ms(2);	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	switch(W25QXX_ReadID())
	{
		case 0xE010:
			printf("Flash is BY25Q10A,memery size is 128KB.\r\n");
			FlashON = 1;
			break;
		case 0xEF11:
			printf("Flash is W25Q20EW,memery size is 256KB.\r\n");
			FlashON = 1;
			break;
		case 0xE012:
			printf("Flash is BY25Q40A,memery size is 1024KB.\r\n");
			FlashON = 1;
			break;
		case 0xC812:
			printf("Flash is GD25Q40C,memery size is 1024KB.\r\n");
			FlashON = 1;
			break;
		case 0x0b16:
			printf("Flash is XT25Q64BDSIGT,memery size is 8M.\r\n");
			FlashON = 1;
			break;
		default :
//			printf("0x%x\r\n",W25QXX_ReadID());
			printf("No Flash!\r\n");
			FlashON = 0;
			break;
	}
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
	return FlashON;
}
//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00

u8 W25QXX_ReadSR(void)   
{  
	u8 byte=0;   
	CS_FLASH_Low;                            //ʹ������   
	SPI1_ReadWriteByte(W25X_ReadStatusReg);    //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI1_ReadWriteByte(0Xff);             //��ȡһ���ֽ�  
	CS_FLASH_High;                            //ȡ��Ƭѡ     
	return byte;   
} 
//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(u8 sr)   
{   
	CS_FLASH_Low;                            //ʹ������   
	SPI1_ReadWriteByte(W25X_WriteStatusReg);   //����дȡ״̬�Ĵ�������    
	SPI1_ReadWriteByte(sr);               //д��һ���ֽ�  
	CS_FLASH_High;                            //ȡ��Ƭѡ     	      
}   
//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteEnable);      //����дʹ��  
	CS_FLASH_High;                            //ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_WriteDisable);     //����д��ָֹ��    
	CS_FLASH_High;                            //ȡ��Ƭѡ     	      
} 		
//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	CS_FLASH_Low;				    
	SPI1_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	 			   
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI1_ReadWriteByte(0xFF);	 
	CS_FLASH_High;				    
	return Temp;
}   		    
//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i;   										    
	CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReadData);         //���Ͷ�ȡ����   
    SPI1_ReadWriteByte((u8)((ReadAddr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPI1_ReadWriteByte((u8)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=SPI1_ReadWriteByte(0XFF);   //ѭ������  
    }
	CS_FLASH_High;  				    	      
}  
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
    W25QXX_Write_Enable();                  //SET WEL 
	CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PageProgram);      //����дҳ����   
    SPI1_ReadWriteByte((u8)((WriteAddr)>>16)); //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((WriteAddr)>>8));   
    SPI1_ReadWriteByte((u8)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//ѭ��д��  
	CS_FLASH_High;                            //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   //�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
u8 W25QXX_BUFFER[4096];		 
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u32 NumByteToWrite)   
{ 
	u32 secpos;
	u16 secoff;
	u16 secremain;	   
 	u16 i;    
	u8 * W25QXX_BUF;	  
   	W25QXX_BUF=W25QXX_BUFFER;	     
 	secpos=WriteAddr/4096;//������ַ  
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
	while(1) 
	{	
		//CS0_High;
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(W25QXX_BUF[secoff+i]!=0xFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			//CS0_High;
			W25QXX_Erase_Sector(secpos);//�����������
			for(i=0;i<secremain;i++)	   //����
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			//CS0_High;
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������  

		}
		else 
		{
			//CS0_High;
			W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);
		}//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumByteToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff=0;//ƫ��λ��Ϊ0 	 

		   	pBuffer+=secremain;  //ָ��ƫ��
			WriteAddr+=secremain;//д��ַƫ��	   
		   	NumByteToWrite-=secremain;				//�ֽ����ݼ�
			if(NumByteToWrite>4096)secremain=4096;	//��һ����������д����
			else secremain=NumByteToWrite;			//��һ����������д����
		}	 
	};	 
}
//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ChipErase);        //����Ƭ��������  
	CS_FLASH_High;                            //ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				   //�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(u32 Dst_Addr)   
{  
	//����falsh�������,������   
 	//printf("fe:%x\r\n",Dst_Addr);	  
    Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
    CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_SectorErase);      //������������ָ�� 
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>16));  //����24bit��ַ    
    SPI1_ReadWriteByte((u8)((Dst_Addr)>>8));   
    SPI1_ReadWriteByte((u8)Dst_Addr);  
	 CS_FLASH_High;                            //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				   //�ȴ��������
}  
//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
}  
//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  	CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PowerDown);        //���͵�������  
	CS_FLASH_High;                            //ȡ��Ƭѡ     	      
    delay_us(3);                               //�ȴ�TPD  
}   
//����
void W25QXX_WAKEUP(void)   
{  
  	CS_FLASH_Low;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
	CS_FLASH_High;                            //ȡ��Ƭѡ     	      
    delay_us(3);                               //�ȴ�TRES1
} 

void ReadMCUPMUDataToPC(void)
{
	Sdata[0]=0xf0;
	Sdata[1]=0x00;
	
	STMFLASH_ReadforByte(GAIN_PARAMETER_SAVE_ADDR,Sdata+2,126);
		
	Sdata[128] = 0xff;
	Sdata[129] = 0xff;
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
}
//void ReadMCUGAIN_InforToPC(void)
//{
//	
//}

void WritePMUDataToMCU(void)
{
	uint16_t i;
	STMFLASH_ReadforByte(FLASH_SAVE_ADDR,Sdata,372);
	for(i=0;i<126;i++)
	{
		Sdata[i+372]=USB_Rx_Buffer[i+2];
	}
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)Sdata,500);
	
	SendOK();
}

void ReadW25QXXCMOSDataToPC(void)
{
	uint8_t temp=0;
	uint16_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata,0,256);
	
	for(i=0;i<256;i=i+2)
	{
		if((Sdata[i]==0xff)&&(Sdata[i+1]==0xff))
		{
			break;
		}
		else
		{
			temp += Sdata[i];
			temp += Sdata[i+1];
		}
	}
	if(temp == Sdata[255])
	{
		Sdata[0]=0xf0;
		Sdata[1]=0x00;
		
		W25QXX_Read(Sdata+2,0,i);
	
		Sdata[i+2] = 0xff;
		Sdata[i+3] = 0xff;
		
		while(USB_StatusDataSended==0);
		USB_StatusDataSended = 0;
		DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
	}
	else{SendError();}
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
}
void ReadW25QXXPMUDataToPC(void)
{
	uint8_t i,temp=0;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata,0,256);
	
	for(i=0;i<122;i++)
	{
		temp += Sdata[i];
	}
	if(temp == Sdata[255])
	{
		W25QXX_Read(Sdata,0x100,128);
		temp = 0;
		for(i=0;i<4;i++)
		{
			temp += Sdata[i];
		}
		
		if(temp == Sdata[127])
		{
			Sdata[0]=0xf0;
			Sdata[1]=0x00;
			W25QXX_Read(Sdata+2,0,122);
			W25QXX_Read(Sdata+124,0x100,4);
		
			Sdata[128] = 0xff;
			Sdata[129] = 0xff;
			while(USB_StatusDataSended==0);
			USB_StatusDataSended = 0;
			DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
		}
		else{SendError();}
	}
	else{SendError();}
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
}
void ReadW25QXXPCBANumToPC(void)
{
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata+2,0x1000,64);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);	
	Sdata[0] = 0xf0;
	Sdata[1] = 0x00;
	Sdata[66] = 0xff;
	Sdata[67] = 0xff;
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
}
void ReadW25QXXModuleNumToPC(void)
{
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata+2,0x1000+64,64);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
	Sdata[0] = 0xf0;
	Sdata[1] = 0x00;
	Sdata[66] = 0xff;
	Sdata[67] = 0xff;
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
}
void ReadW25QXXColorNumToPC(void)
{
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata+2,0x1000+128,2);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
	Sdata[0] = 0xf0;
	Sdata[1] = 0x00;
	Sdata[4] = 0xff;
	Sdata[5] = 0xff;
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
}
void ReadW25QXXGAIN_AVRToPC(void)
{
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
//	W25QXX_Read(Sdata+2,0x2000,2);//���ģʽ
	
	W25QXX_Read(Sdata+2,0x2001,1);//С��ģʽ
	W25QXX_Read(Sdata+3,0x2000,1);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
	Sdata[0] = 0xf0;
	Sdata[1] = 0x00;
	Sdata[4] = 0xff;
	Sdata[5] = 0xff;
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
}
void ReadW25QXXGAIN_InforToPC(void)
{
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
//	W25QXX_Read(Sdata+2,0x3000,12);//10----->12   С��ģʽ
	
	//��ԭ���Ĵ��ģʽ�ĳ�С��ģʽ
	W25QXX_Read(Sdata+2,0x3003,1);		//length
	W25QXX_Read(Sdata+3,0x3002,1);
	W25QXX_Read(Sdata+4,0x3001,1);
	W25QXX_Read(Sdata+5,0x3000,1);
	
	W25QXX_Read(Sdata+6,0x3007,1);		//CRC16
	W25QXX_Read(Sdata+7,0x3006,1);
	W25QXX_Read(Sdata+8,0x3005,1);
	W25QXX_Read(Sdata+9,0x3004,1);
	
	W25QXX_Read(Sdata+10,0x3008,4);		//others
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
	
	Sdata[0] = 0xf0;
	Sdata[1] = 0x00;
	Sdata[14] = 0xff;
	Sdata[15] = 0xff;
	while(USB_StatusDataSended==0);
	USB_StatusDataSended = 0;
	DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
}
void ReadW25QXXGAINDataToPC(void)
{
	uint32_t i,length;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	W25QXX_Read(Sdata,0x3000,4);
//	length = ((Sdata[0]<<24)|(Sdata[1]<<16)|(Sdata[2]<<8)|(Sdata[3]))+1;//С��ģʽ
	length = ((Sdata[3]<<24)|(Sdata[2]<<16)|(Sdata[1]<<8)|(Sdata[0]))+1;//���ģʽ
	
//	while(USB_StatusDataSended==0);
//	USB_StatusDataSended = 0;
	for(i=0;i<length;i=i+512)
	{
		W25QXX_Read(Sdata,0x4000+i,512);
		while(USB_StatusDataSended==0);
		USB_StatusDataSended = 0;
		DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,Sdata,512);
	}
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(50);
}
void Read_mk110_sn()
{
//	PWRON_1117;
////	delay_ms(2);
//	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//	delay_ms(60);
	HANDSHAKE();
	
	CS0_Low;
	SPI1_ReadWriteByte(0xef);
	SPI1_ReadWriteByte(0x01);
	SPI1_ReadWriteByte(0x01);
	SPI1_ReadWriteByte(0x00);
	SPI1_ReadWriteByte(0x04);
	SPI1_ReadWriteByte(0x09);
	SPI1_ReadWriteByte(USB_Rx_Buffer[2]);
	SPI1_ReadWriteByte((uint8_t)((0x0e+USB_Rx_Buffer[2])>>8));
	SPI1_ReadWriteByte((uint8_t)(0x0e+USB_Rx_Buffer[2]));
	CS0_High;
	delay_ms(10);    //2018.12.27
	if(!ReadyFor_module_sn_cmd())
		{
			SendErrorCode(0,0x0000);
			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
			PWROFF_1117;
			delay_ms(100);
			return ;
		}
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	delay_ms(100);		
}
void WriteMK110_SN()
{	
	uint8_t i=0;
	uint16_t checksum=0;
//	if(USB_Rx_Buffer[67]==0xff)
///	{
	Sdata[0]=0xef;
	Sdata[1]=0x01;
	Sdata[2]=0x01;
	Sdata[3]=0x00;
	Sdata[4]=0x44;
	Sdata[5]=0x08;
	Sdata[6]=USB_Rx_Buffer[2];
	for(i=0;i<64;i++)
	{
		Sdata[7+i]=USB_Rx_Buffer[3+i];
		checksum=checksum+USB_Rx_Buffer[3+i];
	}
	checksum=checksum+0x01+0x44+0x08+Sdata[6];
	Sdata[71]=(uint8_t)(checksum>>8);
	Sdata[72]=(uint8_t)(checksum);
//	PWRON_1117;
//	//delay_ms(2);
//	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
//	delay_ms(60);
	HANDSHAKE();
	
	CS0_Low;
	for(i=0;i<73;i++)
		{
			SPI1_ReadWriteByte(Sdata[i]);
		}
			CS0_High;
			delay_ms(10);    //2018.12.27		
//	delay_ms(20);	
	if(!ReadyForCMD())
		{
			SendErrorCode(0,0x0000);
			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
			PWROFF_1117;
			delay_ms(100);				
			return ;
		}
			SendOK();
			IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
			PWROFF_1117;
			delay_ms(100);	
//	}
	
}
void WritePMUDataToW25QXX(void)
{
	uint8_t temp=0;
	uint16_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Erase_Sector(0);//sector0�ڲ������4096
	for(i=0;i<256;i++)
	{
		Sdata[i] = 0xFF;
	}
	for(i=0;i<122;i++)
	{
		Sdata[i]=USB_Rx_Buffer[i+2];

		temp += USB_Rx_Buffer[i+2];
	}
	Sdata[255] = temp;
	W25QXX_Write_Page(Sdata,0,256);//д��Ĵ�������
	
	temp = 0;
	temp += USB_Rx_Buffer[124];temp += USB_Rx_Buffer[125];
	temp += USB_Rx_Buffer[126];temp += USB_Rx_Buffer[127];
	for(i=0;i<256;i++)
	{
		Sdata[i] = 0xFF;
	}
	Sdata[0] = USB_Rx_Buffer[124];
	Sdata[1] = USB_Rx_Buffer[125];
	Sdata[2] = USB_Rx_Buffer[126];
	Sdata[3] = USB_Rx_Buffer[127];
	Sdata[127] = temp;
	W25QXX_Write_Page(Sdata,0x100,128);//д������Ĵ�������
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	SendOK();
}
void WriteCMOSDataToW25QXX(void)
{
	uint8_t temp=0;
	uint16_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Erase_Sector(0);//sector0�ڲ������4096
	for(i=0;i<256;i++)
	{
		Sdata[i] = 0xFF;
	}
	for(i=0;i<256;i=i+2)
	{
		if((USB_Rx_Buffer[i+2] == 0xff)&&(USB_Rx_Buffer[i+3] == 0xff))
		{
			break;
		}
		else
		{
			Sdata[i]=USB_Rx_Buffer[i+2];
			Sdata[i+1]=USB_Rx_Buffer[i+3];

			temp += USB_Rx_Buffer[i+2];
			temp += USB_Rx_Buffer[i+3];
		}
		
	}
	Sdata[255] = temp;
	W25QXX_Write_Page(Sdata,0,256);//д��Ĵ�������
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	SendOK();
}
void WritePCBANumToW25QXX(void)
{
	uint8_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata,0x1000,130);
	W25QXX_Erase_Sector(1);//sector1�ڲ������4096
	for(i=0;i<64;i++)
	{
		Sdata[i]=USB_Rx_Buffer[i+2];
	}
	
	W25QXX_Write_Page(Sdata,0x1000,130);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	SendOK();
}
void WriteModuleNumToW25QXX(void)
{
	uint8_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata,0x1000,130);
	W25QXX_Erase_Sector(1);//sector1�ڲ������4096
	for(i=0;i<64;i++)
	{
		Sdata[64+i]=USB_Rx_Buffer[i+2];
	}
	
	W25QXX_Write_Page(Sdata,0x1000,130);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	SendOK();
}
void WriteColorNumToW25QXX(void)
{
	uint8_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	
	W25QXX_Read(Sdata,0x1000,130);
	W25QXX_Erase_Sector(1);//sector1�ڲ������4096
	for(i=0;i<2;i++)
	{
		Sdata[128+i]=USB_Rx_Buffer[i+2];
	}
	
	W25QXX_Write_Page(Sdata,0x1000,130);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	SendOK();
}
void WriteGAIN_AVRToW25QXX(void)
{
//	uint8_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	W25QXX_Erase_Sector(2);//sector2�ڲ������4096
//	for(i=0;i<2;i++)//���ģʽ
//	{
//		Sdata[i]=USB_Rx_Buffer[i+2];
//	}
	Sdata[0]=USB_Rx_Buffer[3];//��ԭ���Ĵ��ģʽ�ĳ�С��ģʽ
	Sdata[1]=USB_Rx_Buffer[2];
	
	W25QXX_Write_Page(Sdata,0x2000,2);
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
	SendOK();
}
void WriteGAIN_InforToW25QXX(void)
{
//	uint8_t i;
	
	PWRON_1117;
	delay_ms(2);
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x25,BUF,1);//Power on buffer
	delay_ms(1);
	W25QXX_Erase_Sector(3);//sector2�ڲ������4096
//	for(i=0;i<12;i++)//10---->12//���ģʽ
//	{
//		Sdata[i]=USB_Rx_Buffer[i+2];
//	}
	//��ԭ���Ĵ��ģʽ�ĳ�С��ģʽ
	Sdata[0]=USB_Rx_Buffer[5];		//length
	Sdata[1]=USB_Rx_Buffer[4];
	Sdata[2]=USB_Rx_Buffer[3];
	Sdata[3]=USB_Rx_Buffer[2];
	
	Sdata[4]=USB_Rx_Buffer[9];		//CRC16
	Sdata[5]=USB_Rx_Buffer[8];
	Sdata[6]=USB_Rx_Buffer[7];
	Sdata[7]=USB_Rx_Buffer[6];
	
	Sdata[8]=USB_Rx_Buffer[10];	//GAINͼ����+�������		
	Sdata[9]=USB_Rx_Buffer[11];
	
	Sdata[10]=USB_Rx_Buffer[12];	//darkline��־
	Sdata[11]=USB_Rx_Buffer[13];
	
	W25QXX_Write_Page(Sdata,0x3000,12);//10----------->12
	
	IIC_WriteByte(Open_I2Cx,RT9367_addr,0x24,BUF,1);//Power off buffer
	PWROFF_1117;
//	SendOK();
}


























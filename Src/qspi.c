#include "qspi.h"


//QSPI�����ڴ�ӳ��ģʽ��ִ��QSPI����ر�ǰ�ᣬΪ�˼���������ļ���
//����GPIO�����⣬�������������������üĴ�����ʽ��
void QSPI_Enable_Memmapmode(void)
{
	u32 tempreg=0; 
	vu32 *data_reg=&QUADSPI->DR;
	
	RCC->AHB4ENR|=1<<1;    		//ʹ��PORTBʱ��	 
  RCC->AHB4ENR|=1<<6;				//ʹ��PORTGʱ��	 
	RCC->AHB4ENR|=1<<5;    		//ʹ��PORTFʱ��	   
	RCC->AHB3ENR|=1<<14;   		//QSPIʱ��ʹ��
	GPIO_Set(GPIOB,1<<2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PB2���ù������	
	GPIO_Set(GPIOG,1<<6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PG6���ù������
	
	GPIO_Set(GPIOF,0X1<<6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9���ù������
	GPIO_Set(GPIOF,0X1<<7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9���ù������	
	GPIO_Set(GPIOF,0X1<<8,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9���ù������	
	GPIO_Set(GPIOF,0X1<<9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9���ù������	
	
  GPIO_AF_Set(GPIOB,2,9);		//PB2,AF9
 	GPIO_AF_Set(GPIOG,6,10);	//PG6,AF10
	
 	GPIO_AF_Set(GPIOF,6,9);		//PF6,AF9 
 	GPIO_AF_Set(GPIOF,7,9);		//PF7,AF9
 	GPIO_AF_Set(GPIOF,8,10);	//PF8,AF10
 	GPIO_AF_Set(GPIOF,9,10);	//PF9,AF10
	
	//QSPI���ã��ο�QSPIʵ���QSPI_Init����
	RCC->AHB3RSTR|=1<<14;			//��λQSPI
	RCC->AHB3RSTR&=~(1<<14);		//ֹͣ��λQSPI
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->CR=0X01000310;			//����CR�Ĵ���,��Щֵ��ô���ģ���ο�QSPIʵ��/��H750�ο��ֲ�Ĵ�����������
	QUADSPI->DCR=0X00180401;		//����DCR�Ĵ���
	QUADSPI->CR|=1<<0;				//ʹ��QSPI 

	//ע��:QSPI QEλ��ʹ�ܣ���QSPI��д�㷨���棬���Ѿ�������
	//����,������Բ�������QEλ��������Ҫ�����QEλ��1�Ĵ���
	//����,�������ͨ������������,ֱ����¼���ⲿQSPI FLASH,�ǲ����õ�
	//�����ֱ����¼���ⲿQSPI FLASHҲ������,����Ҫ���������QEλ��1�Ĵ���
	
//	//����QEλ����ʹ��д����
//	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
//	QUADSPI->CCR=0X00000106;		//����0X06ָ�ʹ��д
//	while((QUADSPI->SR&(1<<1))==0);	//�ȴ�ָ������
//	QUADSPI->FCR|=1<<1;				//���������ɱ�־λ 	
//	//����0x31��ʹ��QE
//	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
//	QUADSPI->CCR=0X00000131;		//����0X31ָ�QE��λ
//	while((QUADSPI->SR&(1<<1))==0);	//�ȴ�ָ������
//	QUADSPI->FCR|=1<<1;				//���������ɱ�־λ 	
	
	
	//W25QXX����QPIģʽ��0X38ָ�
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->CCR=0X00000138;		//����0X38ָ�W25QXX����QPIģʽ
	while((QUADSPI->SR&(1<<1))==0);	//�ȴ�ָ������
	QUADSPI->FCR|=1<<1;				//���������ɱ�־λ 	

	//W25QXXдʹ�ܣ�0X06ָ�
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->CCR=0X00000106;		//����0X06ָ�W25QXXдʹ��
	while((QUADSPI->SR&(1<<1))==0);	//�ȴ�ָ������
	QUADSPI->FCR|=1<<1;				//���������ɱ�־λ 
	
	//W25QXX����QPI��ض�������0XC0��
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->CCR=0X030003C0;		//����0XC0ָ�W25QXX����������
	QUADSPI->DLR=0;
	while((QUADSPI->SR&(1<<2))==0);	//�ȴ�FTF
	*(vu8 *)data_reg=3<<4;			//����P4&P5=11,8��dummy clocks,104M
	QUADSPI->CR|=1<<2;				//��ֹ���� 
	while((QUADSPI->SR&(1<<1))==0);	//�ȴ����ݷ������
	QUADSPI->FCR|=1<<1;				//���������ɱ�־λ  
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 	 

	//MemroyMap ģʽ����
	while(QUADSPI->SR&(1<<5));		//�ȴ�BUSYλ���� 
	QUADSPI->ABR=0;					//�����ֽ�����Ϊ0��ʵ���Ͼ���W25Q 0XEBָ���,M0~M7=0
	tempreg=0XEB;					//INSTRUCTION[7:0]=0XEB,����0XEBָ�Fast Read QUAD I/O��
	tempreg|=3<<8;					//IMODE[1:0]=3,���ߴ���ָ��
	tempreg|=3<<10;					//ADDRESS[1:0]=3,���ߴ����ַ
	tempreg|=2<<12;					//ADSIZE[1:0]=2,24λ��ַ����-----------------------------------//
	tempreg|=3<<14;					//ABMODE[1:0]=3,���ߴ��佻���ֽ�
	tempreg|=0<<16;					//ABSIZE[1:0]=0,8λ�����ֽ�(M0~M7)
	tempreg|=6<<18;					//DCYC[4:0]=6,6��dummy����
	tempreg|=3<<24;					//DMODE[1:0]=3,���ߴ�������
	tempreg|=3<<26;					//FMODE[1:0]=3,�ڴ�ӳ��ģʽ
	QUADSPI->CCR=tempreg;			//����CCR�Ĵ���
	
	//����QSPI FLASH�ռ��MPU����
	SCB->SHCSR&=~(1<<16);			//��ֹMemManage 
	MPU->CTRL&=~(1<<0);				//��ֹMPU
	MPU->RNR=0;						//���ñ���������Ϊ0(1~7���Ը������ڴ���)
	MPU->RBAR=0X90000000;			//����ַΪ0X9000 000,��QSPI����ʼ��ַ
	MPU->RASR=0X0303002D;			//������ر�������(��ֹ����,����cache,������),���MPUʵ��Ľ���
	MPU->CTRL=(1<<2)|(1<<0);		//ʹ��PRIVDEFENA,ʹ��MPU 
	SCB->SHCSR|=1<<16;				//ʹ��MemManage
}


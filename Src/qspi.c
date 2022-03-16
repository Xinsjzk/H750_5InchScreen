#include "qspi.h"


//QSPI进入内存映射模式（执行QSPI代码必备前提，为了减少引入的文件，
//除了GPIO驱动外，其他的外设驱动均采用寄存器形式）
void QSPI_Enable_Memmapmode(void)
{
	u32 tempreg=0; 
	vu32 *data_reg=&QUADSPI->DR;
	
	RCC->AHB4ENR|=1<<1;    		//使能PORTB时钟	 
  RCC->AHB4ENR|=1<<6;				//使能PORTG时钟	 
	RCC->AHB4ENR|=1<<5;    		//使能PORTF时钟	   
	RCC->AHB3ENR|=1<<14;   		//QSPI时钟使能
	GPIO_Set(GPIOB,1<<2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PB2复用功能输出	
	GPIO_Set(GPIOG,1<<6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PG6复用功能输出
	
	GPIO_Set(GPIOF,0X1<<6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9复用功能输出
	GPIO_Set(GPIOF,0X1<<7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9复用功能输出	
	GPIO_Set(GPIOF,0X1<<8,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9复用功能输出	
	GPIO_Set(GPIOF,0X1<<9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_HIGH,GPIO_PUPD_PU);	//PF6~9复用功能输出	
	
  GPIO_AF_Set(GPIOB,2,9);		//PB2,AF9
 	GPIO_AF_Set(GPIOG,6,10);	//PG6,AF10
	
 	GPIO_AF_Set(GPIOF,6,9);		//PF6,AF9 
 	GPIO_AF_Set(GPIOF,7,9);		//PF7,AF9
 	GPIO_AF_Set(GPIOF,8,10);	//PF8,AF10
 	GPIO_AF_Set(GPIOF,9,10);	//PF9,AF10
	
	//QSPI设置，参考QSPI实验的QSPI_Init函数
	RCC->AHB3RSTR|=1<<14;			//复位QSPI
	RCC->AHB3RSTR&=~(1<<14);		//停止复位QSPI
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CR=0X01000310;			//设置CR寄存器,这些值怎么来的，请参考QSPI实验/看H750参考手册寄存器描述分析
	QUADSPI->DCR=0X00180401;		//设置DCR寄存器
	QUADSPI->CR|=1<<0;				//使能QSPI 

	//注意:QSPI QE位的使能，在QSPI烧写算法里面，就已经设置了
	//所以,这里可以不用设置QE位，否则需要加入对QE位置1的代码
	//不过,代码必须通过仿真器下载,直接烧录到外部QSPI FLASH,是不可用的
	//如果想直接烧录到外部QSPI FLASH也可以用,则需要在这里添加QE位置1的代码
	
//	//设置QE位，先使能写功能
//	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
//	QUADSPI->CCR=0X00000106;		//发送0X06指令，使能写
//	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
//	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 	
//	//发送0x31，使能QE
//	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
//	QUADSPI->CCR=0X00000131;		//发送0X31指令，QE置位
//	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
//	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 	
	
	
	//W25QXX进入QPI模式（0X38指令）
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=0X00000138;		//发送0X38指令，W25QXX进入QPI模式
	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 	

	//W25QXX写使能（0X06指令）
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=0X00000106;		//发送0X06指令，W25QXX写使能
	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 
	
	//W25QXX设置QPI相关读参数（0XC0）
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=0X030003C0;		//发送0XC0指令，W25QXX读参数设置
	QUADSPI->DLR=0;
	while((QUADSPI->SR&(1<<2))==0);	//等待FTF
	*(vu8 *)data_reg=3<<4;			//设置P4&P5=11,8个dummy clocks,104M
	QUADSPI->CR|=1<<2;				//终止传输 
	while((QUADSPI->SR&(1<<1))==0);	//等待数据发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位  
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 	 

	//MemroyMap 模式设置
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->ABR=0;					//交替字节设置为0，实际上就是W25Q 0XEB指令的,M0~M7=0
	tempreg=0XEB;					//INSTRUCTION[7:0]=0XEB,发送0XEB指令（Fast Read QUAD I/O）
	tempreg|=3<<8;					//IMODE[1:0]=3,四线传输指令
	tempreg|=3<<10;					//ADDRESS[1:0]=3,四线传输地址
	tempreg|=2<<12;					//ADSIZE[1:0]=2,24位地址长度-----------------------------------//
	tempreg|=3<<14;					//ABMODE[1:0]=3,四线传输交替字节
	tempreg|=0<<16;					//ABSIZE[1:0]=0,8位交替字节(M0~M7)
	tempreg|=6<<18;					//DCYC[4:0]=6,6个dummy周期
	tempreg|=3<<24;					//DMODE[1:0]=3,四线传输数据
	tempreg|=3<<26;					//FMODE[1:0]=3,内存映射模式
	QUADSPI->CCR=tempreg;			//设置CCR寄存器
	
	//设置QSPI FLASH空间的MPU保护
	SCB->SHCSR&=~(1<<16);			//禁止MemManage 
	MPU->CTRL&=~(1<<0);				//禁止MPU
	MPU->RNR=0;						//设置保护区域编号为0(1~7可以给其他内存用)
	MPU->RBAR=0X90000000;			//基地址为0X9000 000,即QSPI的起始地址
	MPU->RASR=0X0303002D;			//设置相关保护参数(禁止共用,允许cache,允许缓冲),详见MPU实验的解析
	MPU->CTRL=(1<<2)|(1<<0);		//使能PRIVDEFENA,使能MPU 
	SCB->SHCSR|=1<<16;				//使能MemManage
}


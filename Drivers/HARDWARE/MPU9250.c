#include "MPU9250.h"
#include "spi.h"
#include "main.h"
#include "dma.h"
uint8_t Mag_adjust[3] = { 0 };
volatile uint8_t DataReceive[23] __attribute__((section("DMABuff"))) = {0};
const uint8_t DataSend[23] = { MPU9250_REG_ACCEL_XOUT_H | 0x80 };
uint32_t SampleInterLast = 0;
uint32_t SampleInter = 0;
extern int SPI_REQUEST_Count;
volatile uint8_t FlagDMASuccess = 0;
int ErrorCode = 0;

static void I2C_Mag_Setting()
{
	uint8_t WritaData[3];
	WritaData[0] = AK8963_ADDR | I2C_READFLAG;
	WritaData[1] = AK8963_REG_ST1;
	WritaData[2] = 0x88;  // 设置数据长度为8，设置使能位
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_SLV0_ADDR, WritaData, 3);
	HAL_Delay(1);
}
static void SPI_ModifyMPU9250Reg(uint8_t reg, uint8_t CLEARMASK, uint8_t SETMASK)
{
	uint8_t RegPreVal;
	SPI_ReadMPU9250Reg(reg, &RegPreVal, 1);
	RegPreVal = ((RegPreVal) & (~CLEARMASK)) | SETMASK;
	SPI_WriteMPU9250Reg(reg, &RegPreVal, 1);
}
static void I2C_Mag_SLV4_Write(uint8_t reg, uint8_t val)
{
	uint8_t WriteData[3];
	WriteData[0] = AK8963_ADDR;
	WriteData[1] = reg;
	WriteData[2] = val;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_SLV4_ADDR, WriteData, 3); //设置SLAVE4
	WriteData[0] = 0x80;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_SLV4_CTRL, WriteData, 1); //使能SLAVE4
	HAL_Delay(1);
}
static uint8_t I2C_Mag_SLV4_Read(uint8_t AK8963Reg)
{
	uint8_t WritaData[2];
	uint8_t ReadData;
	WritaData[0] = AK8963_ADDR | I2C_READFLAG;
	WritaData[1] = AK8963Reg;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_SLV4_ADDR, WritaData, 2);
	WritaData[0] = 0x80;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_SLV4_CTRL, WritaData, 1);
	HAL_Delay(10);
	SPI_ReadMPU9250Reg(MPU9250_REG_I2C_SLV4_DI, &ReadData, 1);
	return ReadData;

}
int MPU9250_Init(void)
{
	uint8_t WriteData;

	WriteData = (1 << 0);
	SPI_ModifyMPU9250Reg(MPU9250_REG_PWR_MGMT_1, 0x7, WriteData); //设置时钟源
	WriteData = 0x20;
	SPI_ModifyMPU9250Reg(MPU9250_REG_USER_CTRL, WriteData, WriteData); //设置I2C Master
	WriteData = 0x4d;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_MST_CTRL, &WriteData, 1); // I2C 400kHZ,同时等待I2C数据全部读取完成后才输出中断
	I2C_Mag_SLV4_Write(AK8963_REG_CNTL1, 0x0); //设置AK8963掉电
	WriteData = 0x80;
	SPI_WriteMPU9250Reg(MPU9250_REG_PWR_MGMT_1, &WriteData, 1);  //Reset MPU9250
	HAL_Delay(10);
	I2C_Mag_SLV4_Write(AK8963_REG_CNTL2, 0x01);
	WriteData = (1 << 0);
	SPI_ModifyMPU9250Reg(MPU9250_REG_PWR_MGMT_1, 0x7, WriteData); //设置时钟源

	SPI_ReadMPU9250Reg(MPU9250_REG_WHO_AM_I, &WriteData, 1);
	if (WriteData != 0x71)
		return 5;
	WriteData = 0x01;
	SPI_WriteMPU9250Reg(MPU9250_REG_CONFIG, &WriteData, 1); //设置陀螺仪低通滤波器的值为1
	/*************Init ACCELEROMETER*******/
	WriteData = (1 << 3);
	SPI_WriteMPU9250Reg(MPU9250_REG_ACCEL_CONFIG, &WriteData, 1); //设置量程为+-4g
	WriteData = 0x7;
	SPI_WriteMPU9250Reg(MPU9250_REG_ACCEL_CONFIG2, &WriteData, 1); //设置低通滤波器值为7，主要是为了降低延迟
	/*************Init GYROSCOPE**********/
	WriteData = (0x3 << 3);
	SPI_WriteMPU9250Reg(MPU9250_REG_GYRO_CONFIG, &WriteData, 1); //设置陀螺仪量程为+-2000 dps

	WriteData = (1 << 3);
	SPI_ModifyMPU9250Reg(MPU9250_REG_PWR_MGMT_1, WriteData, 0); //开始温度传感器
	WriteData = 0x10;
	SPI_WriteMPU9250Reg(MPU9250_REG_INT_PIN_CFG, &WriteData, 1); //配置中断电气特性
	WriteData = 0x01;
	SPI_WriteMPU9250Reg(MPU9250_REG_INT_ENABLE, &WriteData, 1); //使能数据准备完成中断
	WriteData = 0x20;
	SPI_ModifyMPU9250Reg(MPU9250_REG_USER_CTRL, WriteData, WriteData); //设置I2C Master
	WriteData = 0x4d;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_MST_CTRL, &WriteData, 1); // I2C 400kHZ,同时等待I2C数据全部读取完成后才输出中断

	/*************Init MAG*****************/

	WriteData = I2C_Mag_SLV4_Read(AK8963_REG_WIA);
	if (WriteData != 0x48)
		return 2;

	I2C_Mag_SLV4_Write(AK8963_REG_CNTL1, 0x00); //设置AK8963掉电
	HAL_Delay(200);
	I2C_Mag_SLV4_Write(AK8963_REG_CNTL1, 0x0F); //设置AK8963模式为fuse ROM模式
	HAL_Delay(200);
	Mag_adjust[0] = I2C_Mag_SLV4_Read(AK8963_REG_ASAX); //得出AK8963三轴磁力计的校正值
	Mag_adjust[1] = I2C_Mag_SLV4_Read(AK8963_REG_ASAY);
	Mag_adjust[2] = I2C_Mag_SLV4_Read(AK8963_REG_ASAZ);
	I2C_Mag_SLV4_Write(AK8963_REG_CNTL1, 0x00); //设置AK8963掉电
	HAL_Delay(200);
	I2C_Mag_SLV4_Write(AK8963_REG_CNTL1, 0x16); //设置AK8963模式为continue 2模式
	HAL_Delay(200);
	WriteData = 0x9;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_SLV4_CTRL, &WriteData, 1); //让MPU9250每10次采样才读取AK8963一次
	WriteData = 0x1;
	SPI_WriteMPU9250Reg(MPU9250_REG_I2C_MST_DELAY_CTRL, &WriteData, 1); //开启I2C Master延迟
	I2C_Mag_Setting(); //设置连续读取
	MODIFY_REG(hspi1.Instance->CFG1, 0x7 << 28, SPI_BAUDRATEPRESCALER_16);  //提高SPI速度
	return 0;
}
/* 配置PA3作为MPU9250中断的输入引脚 配置DMA1 Stream0<-->SPI1 RX  DMA1 Stream1<-->SPI1 TX*/
void MPU9250_IntConfig(void)
{
	GPIO_InitTypeDef GInit;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GInit.Alternate = 0;
	GInit.Mode = GPIO_MODE_IT_RISING;
	GInit.Pin = GPIO_PIN_3;
	GInit.Pull = GPIO_PULLDOWN;
	GInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GInit);

	/*配置DMA传输地址*/
	DMA1_Stream0->M0AR = (uint32_t)DataReceive;
	DMA1_Stream0->PAR = (uint32_t)&SPI1->RXDR;
	DMA1_Stream0->NDTR = 23;
	DMA1_Stream1->M0AR = (uint32_t)DataSend;
	DMA1_Stream1->PAR = (uint32_t)&SPI1->TXDR;
	DMA1_Stream1->NDTR = 23;

	/*开启DMA1 Stream0 的TC中断*/
	MODIFY_REG(DMA1_Stream0->CR, DMA_IT_TC, DMA_IT_TC);
	HAL_NVIC_SetPriority(EXTI3_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}
void EXTI3_IRQHandler(void)
{
	/* 	SPI_ReadMPU9250Reg(MPU9250_REG_ACCEL_XOUT_H, DataOut, 22);
		EXTI->PR1 |= 0x1<<3;
		SPI_REQUEST_Count++;
		SampleInter = TIM3->CNT - SampleInterLast;
		SampleInterLast = TIM3->CNT; */
	EXTI->PR1 |= (0x1 << 3);   //!@！注意，若是把这一行放在最后，则会重复进入中断，奇怪的芯片
	if (!FlagDMASuccess)         //DMA传输由于某种原因没有成功，所以需要重新失能DMA SPI1
	{
		ErrorCode = SPI_REQUEST_Count;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;
		DMA1_Stream1->CR &= ~DMA_SxCR_EN;
		DMA1->LIFCR |= (0x1 << 5 | 0x1 << 11);  //清理DMA1 Stream0 与 Stream1 的TC中断位
		SPI1->CR1 &= ~SPI_CR1_SPE;
		SPI1->CFG1 &= ~(0x3 << 14);  //清除SPI TXDMA RXDMA位
	}
	DMA1_Stream0->NDTR = 23;    //给DMA设置传输的数据量
	DMA1_Stream1->NDTR = 23;

	SPI1->CFG1 |= 0x1 << 14;		//使能SPI RX Request 参考Reference Manual 

	DMA1_Stream0->CR |= DMA_SxCR_EN;  //使能DMA1 DMA2相应的通道
	DMA1_Stream1->CR |= DMA_SxCR_EN;

	SPI1->CFG1 |= (0x1 << 15);    //使能SPI TX Request
	SPI1->CR2 = 23;              //设置SPI传输的数量
	SPI1->CR1 |= SPI_CR1_SPE;   //使能SPI
	SPI1->CR1 |= SPI_CR1_CSTART;  //开启传输

	SPI_REQUEST_Count++;      //计数器
	FlagDMASuccess = 0;       //清除成功位
	SampleInter = TIM3->CNT - SampleInterLast;   //计时器
	SampleInterLast = TIM3->CNT;

}

void DMA1_Stream0_IRQHandler(void)
{
	DMA1->LIFCR |= (0x1 << 5 | 0x1 << 11);  //清理DMA1 Stream0 与 Stream1 的TC中断位

	DMA1_Stream0->CR &= ~DMA_SxCR_EN;    //失能DMA1 DMA2
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;

	if (!HAL_IS_BIT_SET(SPI1->SR, SPI_IT_EOT))   //要是SPI没有传输完成，输出错误
		ErrorCode = -1;
	//while (!HAL_IS_BIT_SET(SPI1->SR, SPI_IT_EOT));
	SPI1->CR1 &= ~SPI_CR1_SPE;                    //失能SPI
	SPI1->CFG1 &= ~(0x3 << 14);										//失能SPI DMA Request
	SET_BIT(SPI1->IFCR, SPI_IFCR_EOTC | SPI_IFCR_TXTFC);  //清除SPI EOT TXTF 位

	if ((DMA1_Stream0->CR & DMA_SxCR_EN) || (DMA1_Stream1->CR & DMA_SxCR_EN))  //要是DMA没有被失能，输出错误
		ErrorCode = -2;
	
	FlagDMASuccess = 1;                                        //标志位，标记在此次传输中，DMA成功发送接收完成，并被失能
}

void SPI_WriteMPU9250Reg(uint8_t RegAdd, const uint8_t* WriteData, uint32_t size)
{

	/* Set the number of data at current transfer */
	hspi1.Instance->CR2 = size + 1;

	/* Enable SPI peripheral */
	__HAL_SPI_ENABLE(&hspi1);

	/* Master transfer start */
	SET_BIT(hspi1.Instance->CR1, SPI_CR1_CSTART);

	*((__IO uint8_t*) & hspi1.Instance->TXDR) = RegAdd;

	while (size > 0)
	{
		if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXP))
		{
			*((__IO uint8_t*) & hspi1.Instance->TXDR) = *WriteData;
			WriteData += 1;
			size--;
		}
	}

	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_EOT))
		;
	__HAL_SPI_DISABLE(&hspi1);
	__HAL_SPI_CLEAR_EOTFLAG(&hspi1);
	__HAL_SPI_CLEAR_TXTFFLAG(&hspi1);
	__HAL_SPI_CLEAR_OVRFLAG(&hspi1);

}

void SPI_ReadMPU9250Reg(uint8_t RegAdd, uint8_t* ReadData, uint32_t ReadSize)
{
	uint32_t SendSize = ReadSize;
	hspi1.Instance->CR2 = ReadSize + 1;
	__HAL_SPI_ENABLE(&hspi1);
	/* Master transfer start */
	SET_BIT(hspi1.Instance->CR1, SPI_CR1_CSTART);

	*((__IO uint8_t*) & hspi1.Instance->TXDR) = (uint8_t)(RegAdd | 0x80);
	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXP))
		;

	*ReadData = *((__IO uint8_t*) & hspi1.Instance->RXDR);

	while ((SendSize > 0) || (ReadSize > 0))
	{
		/* Check the TXP flag */
		if ((__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXP)) && (SendSize > 0))
		{
			*((__IO uint8_t*) & hspi1.Instance->TXDR) = (uint8_t)0x01;
			SendSize--;
			__DSB();
			__ISB();
		}

		/* Check the RXP flag */
		if ((__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXP)) && (ReadSize > 0))
		{
			*ReadData = *((__IO uint8_t*) & hspi1.Instance->RXDR);
			ReadSize--;
			ReadData += 1;
			__DSB();
			__ISB();
		}
	}

	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_EOT))
		;
	__HAL_SPI_DISABLE(&hspi1);
	__HAL_SPI_CLEAR_EOTFLAG(&hspi1);
	__HAL_SPI_CLEAR_TXTFFLAG(&hspi1);

}
/*
使用资源 DMA1_Stream0 DMA1_Stream1 SPI1 EXTI3 GPIO端口 PA4 PA5 PA6 PA7 PA3 
*/
int MPU9250_TopInit(void)
{
	int Flag;
	MX_DMA_Init();
	MX_SPI1_Init();
	Flag = MPU9250_Init();
	MPU9250_IntConfig();
	return Flag;
}
/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/*
 *@Note 
 ADC使用DMA采样例程：
 ADC通道1(PA1),规则组通道通过DMA获取 ADC连续1024次转换数据。
 
*/

#include "debug.h"
#include "stdio.h"
#include "string.h"

/* Global Variable */
u16 TxBuf[1024];
s16 Calibrattion_Val = 0;
extern int Fulse_Width;

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define TxSize1    (size(TxBuffer1))
#define TxSize2    (size(TxBuffer2))
#define size(a)    (sizeof(a) / sizeof(*(a)))

/* Global Variable */
u8 TxBuffer1[] = "*Buffer1 Send from USART2 to USART3 using DMA!"; /* Send by UART2 */
u8 TxBuffer2[] = "#Buffer2 Send from USART3 to USART2 using DMA!"; /* Send by UART3 */
u8 RxBuffer1[TxSize1] = {0};                                       /* USART2 Using  */
u8 RxBuffer2[TxSize2] = {0};                                       /* USART3 Using  */

u8 TxCnt1 = 0, RxCnt1 = 0;
u8 TxCnt2 = 0, RxCnt2 = 0;

u8 Rxfinish1 = 0, Rxfinish2 = 0;

#define N 8 //频率采样数组长度
#define n 2 //去除频率采样数组左右旁值个数
unsigned int Frequence_Ave;//有效取样数组平均频率（长度=N-2n)
unsigned int Frequence_Arr[50];//频率采样数组
extern unsigned int TIM1_CAPTURE_VAL;//捕获信号频率脉冲宽度


int Fre1;int* Probe_Fre1=&Fre1;
int Fre2;int* Probe_Fre2=&Fre2;
int Fre3;int* Probe_Fre3=&Fre3;
float Probe1_Moisture;float* String_Probe1_Moisture=&Probe1_Moisture;
float Probe2_Moisture;float* String_Probe2_Moisture=&Probe2_Moisture;
float Probe3_Moisture;float* String_Probe3_Moisture=&Probe3_Moisture;

float Power;float* String_Power=&Power;
float Waterlevel;float* String_Waterlevel=&Waterlevel;


int Frequense=0;


char p1[100];//将3个水分传感器探头、1个水位传感器探头和电源电压的采样值整合成一个字符串
char p2[100];//将数据流名称“height"与采样值整合成一个字符串
char p3[100];//将P2长度的16进制转换成字符串
char p4[100];//将P2转换成16进制
char p5[20]="0300";//0x03是使用数据六3来同步
char p6[100];//往Topic推送的报文
char p7[100];//发送数据的总体AT指令字符串

int lengString1;
int lengString2;
int lengString3;



int Probe1_Dry_Fre=11732;
int Probe2_Dry_Fre=11765;

int Probe3_Dry_Fre=11931;

int Probe1_Wet_Fre=10030;
int Probe2_Wet_Fre=9797;

int Probe3_Wet_Fre=9978;

float SoilDryMoisture=4.0;
float SoilWetMoisture=46.0;

float Probe_k1;
float Probe_k2;
float Probe_k3;
float Probe_k4;

TestStatus TransferStatus1 = FAILED;
TestStatus TransferStatus2 = FAILED;


/*********************************************************************
 * @fn      Buffercmp
 *
 * @brief   Compares two buffers
 *
 * @param   Buf1,Buf2 - buffers to be compared
 *          BufferLength - buffer's length
 *
 * @return  PASSED - Buf1 identical to Buf
 *          FAILED - Buf1 differs from Buf2
 */
//TestStatus Buffercmp(uint8_t *Buf1, uint8_t *Buf2, uint16_t BufLength)
//{
//    while(BufLength--)
//    {
//        if(*Buf1 != *Buf2)
//        {
//            return FAILED;
//        }
//        Buf1++;
//        Buf2++;
//    }
//    return PASSED;
//}


//将ASCII转换成16进制
void string2hexString(char* input, char* output)
{
    int loop;
    int i;

    i=0;
    loop=0;

    while(input[loop] != '\0')
    {
        sprintf((char*)(output+i),"%02X", input[loop]);
        loop+=1;
        i+=2;
    }
    output[i++] = '\0';
}



void Probe_Init()//Ì½Í·4²ÉÑù
{

    GPIO_InitTypeDef Choose_InitStruct1;
    GPIO_InitTypeDef Choose_InitStruct2;
    GPIO_InitTypeDef Choose_InitStruct3;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);//GPIOÊ±ÖÓÊ¹ÄÜ£¬PA7



    Choose_InitStruct1.GPIO_Pin=GPIO_Pin_4;
    Choose_InitStruct1.GPIO_Speed=GPIO_Speed_50MHz;
    Choose_InitStruct1.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init( GPIOE, &Choose_InitStruct1);



    Choose_InitStruct2.GPIO_Pin=GPIO_Pin_3;
    Choose_InitStruct2.GPIO_Speed=GPIO_Speed_50MHz;
    Choose_InitStruct2.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init( GPIOE, &Choose_InitStruct2);



    Choose_InitStruct3.GPIO_Pin=GPIO_Pin_2;
    Choose_InitStruct3.GPIO_Speed=GPIO_Speed_50MHz;
    Choose_InitStruct3.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init( GPIOE, &Choose_InitStruct3);

}

void Probe1Control()//水分传感器探头1频率信号采样
{

    GPIO_ResetBits( GPIOE, GPIO_Pin_4);//低电平
    GPIO_ResetBits( GPIOE, GPIO_Pin_3);//低电平
    GPIO_ResetBits( GPIOE, GPIO_Pin_2);//低电平
}

void Probe2Control()//水分传感器探头2频率信号采样
{
    GPIO_SetBits( GPIOE, GPIO_Pin_4);//高电平
    GPIO_ResetBits( GPIOE, GPIO_Pin_3);//低电平
    GPIO_ResetBits( GPIOE, GPIO_Pin_2);//低电平
 }

void Probe3Control()//水分传感器探头3频率信号采样
{
     GPIO_ResetBits( GPIOE, GPIO_Pin_4);//低电平
     GPIO_SetBits( GPIOE, GPIO_Pin_3);//高电平
     GPIO_ResetBits( GPIOE, GPIO_Pin_2);//低电平
}

void ProbeStop()//关闭水分传感器探头采样
{
     GPIO_SetBits( GPIOE, GPIO_Pin_4);//低电平
     GPIO_SetBits( GPIOE, GPIO_Pin_3);//高电平
     GPIO_SetBits( GPIOE, GPIO_Pin_2);//低电平
}
/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);


    /* USART3 TX-->B.10  RX-->B.11 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure);

//    DMA_Cmd(DMA1_Channel7, ENABLE); /* USART2 Tx */
//    DMA_Cmd(DMA1_Channel6, ENABLE); /* USART2 Rx */
    DMA_Cmd(DMA1_Channel2, ENABLE); /* USART3 Tx */
    DMA_Cmd(DMA1_Channel3, ENABLE); /* USART3 Rx */


    USART_Cmd(USART3, ENABLE);
}


/*********************************************************************
 * @fn      DMA_INIT
 *
 * @brief   Configures the DMA for USART2 & USART3.
 *
 * @return  none
 */
void DMA_INIT(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DATAR); /* USART2->DATAR:0x40004804 */
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer2;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = TxSize2;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer2;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = TxSize1;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
}



/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void ADC_Function_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure={0};
	ADC_InitTypeDef ADC_InitStructure1={0};
	GPIO_InitTypeDef GPIO_InitStructure={0};
	GPIO_InitTypeDef GPIO_InitStructure1={0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure1);

	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_DeInit(ADC1);
	ADC_InitStructure1.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure1.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure1.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure1.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure1.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure1.ADC_NbrOfChannel = 0;
    ADC_Init(ADC1, &ADC_InitStructure1);


	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	ADC_BufferCmd(ADC1, DISABLE);   //disable buffer
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	Calibrattion_Val = Get_CalibrationValue(ADC1);	
	
	ADC_BufferCmd(ADC1, ENABLE);   //enable buffer
}

/*********************************************************************
 * @fn      Get_ADC_Val
 *
 * @brief   Returns ADCx conversion result data.
 *
 * @param   ch - ADC channel.
 *            ADC_Channel_0 - ADC Channel0 selected.
 *            ADC_Channel_1 - ADC Channel1 selected.
 *            ADC_Channel_2 - ADC Channel2 selected.
 *            ADC_Channel_3 - ADC Channel3 selected.
 *            ADC_Channel_4 - ADC Channel4 selected.
 *            ADC_Channel_5 - ADC Channel5 selected.
 *            ADC_Channel_6 - ADC Channel6 selected.
 *            ADC_Channel_7 - ADC Channel7 selected.
 *            ADC_Channel_8 - ADC Channel8 selected.
 *            ADC_Channel_9 - ADC Channel9 selected.
 *            ADC_Channel_10 - ADC Channel10 selected.
 *            ADC_Channel_11 - ADC Channel11 selected.
 *            ADC_Channel_12 - ADC Channel12 selected.
 *            ADC_Channel_13 - ADC Channel13 selected.
 *            ADC_Channel_14 - ADC Channel14 selected.
 *            ADC_Channel_15 - ADC Channel15 selected.
 *            ADC_Channel_16 - ADC Channel16 selected.
 *            ADC_Channel_17 - ADC Channel17 selected.
 *
 * @return  none
 */
u16 Get_ADC_Val(u8 ch)
{
	u16 val;

	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	val = ADC_GetConversionValue(ADC1);

	return val;
}

/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init( DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
	DMA_InitTypeDef DMA_InitStructure={0};

	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

	DMA_DeInit(DMA_CHx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
	DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = bufsize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init( DMA_CHx, &DMA_InitStructure );
}

/*********************************************************************
 * @fn      Get_ConversionVal
 *
 * @brief   Get Conversion Value.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal(s16 val)
{
	if((val+Calibrattion_Val)<0) return 0;
	if((Calibrattion_Val+val)>4095||val==4095) return 4095;
	return (val+Calibrattion_Val);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
u16 Get_ConversionVal_Waterlevel(void)
{
    DMA_Tx_Init( DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)TxBuf, 1024 );
    DMA_Cmd( DMA1_Channel1, ENABLE );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    Delay_Ms(50);
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    if((TxBuf[0]+Calibrattion_Val)<0) return 0;
    if((Calibrattion_Val+TxBuf[0])>4095||TxBuf[0]==4095) return 4095;
    return (TxBuf[0]+Calibrattion_Val);
}

u16 Get_ConversionVal_Power(void)
{
    DMA_Tx_Init( DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)TxBuf, 1024 );
    DMA_Cmd( DMA1_Channel1, ENABLE );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5 );
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    Delay_Ms(50);
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    if((TxBuf[0]+Calibrattion_Val)<0) return 0;
    if((Calibrattion_Val+TxBuf[0])>4095||TxBuf[0]==4095) return 4095;
    return (TxBuf[0]+Calibrattion_Val);
}

void Input_Capture_Init( u16 arr, u16 psc )
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    TIM_ICInitTypeDef TIM_ICInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits( GPIOA, GPIO_Pin_8 );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter =  0x00;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_PWMIConfig( TIM1, &TIM_ICInitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig( TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE );

    TIM_SelectInputTrigger( TIM1, TIM_TS_TI1FP1 );
    TIM_SelectSlaveMode( TIM1, TIM_SlaveMode_Reset );
    TIM_SelectMasterSlaveMode( TIM1, TIM_MasterSlaveMode_Enable );
    TIM_Cmd( TIM1, ENABLE );
}

//频率测量，并进行中值滤波
int Get_Frequence( )
{

  unsigned int i = 0;
  unsigned int j = 0;
  unsigned int f_temp = 0;
  unsigned int Frequence_All = 0;

/*******************************************输入频率信号采样，长度为N****************************************************/
    for(i=0;i<N;i++)
    {
      Delay_Ms(20);
      Frequence_Arr[i]  = 72000000/(TIM1_CAPTURE_VAL);
    }

/**********************************************冒泡排序****************************************************/
    for(j = 0; j < N; j++)
    {
     for(i = 0; i < N-j; i++)
      {
       if(Frequence_Arr[i] > Frequence_Arr[i+1] )
        {
         f_temp = Frequence_Arr[i] ;
         Frequence_Arr[i] =  Frequence_Arr[i+1];
         Frequence_Arr[i+1] = f_temp;
        }
      }
    }

/**********************************************求取采样数组有效部分，去除两旁较大值和较小值*****************************************************/
    for(i=n;i<N-n;i++)
    {
     Frequence_All=Frequence_All + Frequence_Arr[i];
    }

    Frequence_Ave = (Frequence_All/(N-2*n));//求取有效采样数组平均频率


return Frequence_Ave;

}


int main(void)
{
//	u16 i;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Probe_Init();//水分传感器探头初始化
	Delay_Init();
	ADC_Function_Init();
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	DMA_INIT();
	USARTx_CFG(); /* USART2 & USART3 INIT */
	USART_DMACmd(USART3, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
	while(DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) /* Wait until USART3 TX DMA1 Transfer Complete */
    {
    }
//    while(DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET) /* Wait until USART3 RX DMA1 Transfer Complete */
//    {
//    }
//    TransferStatus1 = Buffercmp(TxBuffer1, RxBuffer2, TxSize1);
//    if(TransferStatus1)
//    {
//        printf("Send Success!\r\n");
//    }
//    else
//    {
//        printf("Send Fail!\r\n");
//    }
    printf("RxBuffer2:%s\r\n", RxBuffer2);
    Input_Capture_Init( 0xFFFF, 72-1 );

    printf("CalibrattionValue:%d\n", Calibrattion_Val);
  
//	DMA_Tx_Init( DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)TxBuf, 1024 );
//	DMA_Cmd( DMA1_Channel1, ENABLE );
//
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5 );
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//  Delay_Ms(50);
//	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
//
//	for(i=0; i<1024; i++)
//	{
////		printf( "%04d\r\n", Get_ConversionVal(TxBuf[i]));
//	    printf( "%04d\r\n", Get_ConversionVal_PA0());
//	    printf( "%04d\r\n", Get_ConversionVal_PA1());
//		Delay_Ms(5000);
//	}

	while(1)
	{
        Probe1Control();//采样水分传感器探头1频率信号
        Delay_Ms(2000);
        Frequense = Get_Frequence( );
        Fre1=Frequense;
        Probe_k1 = (SoilWetMoisture  - SoilDryMoisture) /(Probe1_Dry_Fre - Probe1_Wet_Fre);
        Probe1_Moisture = SoilDryMoisture + (Probe1_Dry_Fre -Fre1)*Probe_k1;

        Probe2Control();//采样水分传感器探头2频率信号
        Delay_Ms(2000);
        Frequense=Get_Frequence( );
        Fre2=Frequense;
        Probe_k2 = (SoilWetMoisture  - SoilDryMoisture) /(Probe2_Dry_Fre - Probe2_Wet_Fre);
        Probe2_Moisture = SoilDryMoisture + (Probe2_Dry_Fre -Fre2)*Probe_k2;

        Probe3Control(); //采样水分传感器探头3频率信号
        Delay_Ms(2000);
        Frequense= Get_Frequence( );
        Fre3=Frequense;
        Probe_k3 = (SoilWetMoisture  - SoilDryMoisture) /(Probe3_Dry_Fre - Probe3_Wet_Fre);
        Probe3_Moisture = SoilDryMoisture + (Probe3_Dry_Fre -Fre3)*Probe_k3;

        ProbeStop();//关闭水分传感器探头采样

        printf("AT+ZIPCALL=1");
        Delay_Ms(2000);


        printf("AT+ZMQNEW=1,183.230.40.39,6002,12000,2048,2");
        Delay_Ms(2000);

        printf("AT+ZMQCON=1,4,1028645212,1000,1,0,488621,shanty");
        Delay_Ms(2000);


        sprintf(p1,"%2.1f%2.1f%2.1f%2.1f%1.2f",*String_Probe1_Moisture,*String_Probe2_Moisture,
                *String_Probe3_Moisture,*String_Waterlevel,*String_Power);
        sprintf(p2,"{\"height\":\"%s\"}",p1);
        lengString1=strlen(p2);
        sprintf(p3,"%x",lengString1);
        lengString2=strlen(p2);
        string2hexString(p2, p4);
        sprintf(p6,"%s%s%s",p5,p3,p4);
        lengString3=strlen(p6);
        sprintf(p7,"AT+ZMQPUB=1,$dp,1,0,0,%d,%s",lengString3,p6);
        Delay_Ms(2000);
        printf("%s\r\n",p7);
        Delay_Ms(2000);


//	    printf( "%05d\r\n", Get_ConversionVal_PA0());
//	    Delay_Ms(1000);
//	    printf( "%05d\r\n", Get_ConversionVal_PA1());
//	    Delay_Ms(1000);
//	    printf( "%05d\r\n", Fulse_Width);

	}
}









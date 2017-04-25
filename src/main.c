//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
#include "stm32f10x.h"

static u8 fac_us = 0;  //us延时倍乘数
static u16 fac_ms = 0; //ms

/**************************************************
Name: delay_init
功能：应用程序延时函数初始化，包括为滴答时钟准备相关数值
参数：u8 SYSCLK
返回：None
***************************************************/
void delay_init(u8 SYSCLK)
{
  SysTick->CTRL &= 0xfffffffb; //设置滴答时钟的时钟源：bit 2清空，选择外部时钟HCLK/8
  fac_us = SYSCLK / 8;
  fac_ms = (u16)fac_us * 1000;
}
/**************************************************
名字：delay_ms
共能：应用程序ms级别的延时实现
参数：u16 nms
返回：None
***************************************************/
void delay_ms(u16 nms)
{
  u32 temp;
  SysTick->LOAD = (u32)nms * fac_ms; //时间加载
  SysTick->VAL = 0x00;               //清空计数器
  SysTick->CTRL = 0x01;              //开始倒数
  do
  {
    temp = SysTick->CTRL;
  } while (temp & 0x01 && !(temp & (1 << 16))); //等待时间到达
  SysTick->CTRL = 0x00;                         //关闭计数器
  SysTick->VAL = 0X00;                          //清空计数器
}

int main(int argc, char *argv[])
{
  // 初始化时钟，默认72MHz
  SystemInit();

  delay_init(72);

  // 打开时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  // 串口
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  // 中断
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  // 初始化GPIO
  GPIO_InitTypeDef GPIO_InitStructure;

  //初始化GPIOA
  //USART1 Tx(PA.09)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //USART1 Rx(PA.10)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // PA0~PA7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 初始化GPIOC
  // PC0~PC15
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 初始化GPIOD
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // 中断端口PA11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //USART1配置
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //PPP外部中断线
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Infinite loop
  while (1)
  {
    delay_ms(100);
    GPIO_SetBits(GPIOD, GPIO_Pin_2);

    delay_ms(100);
    // 设置低
    GPIO_ResetBits(GPIOD, GPIO_Pin_2);
  }
}


void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line11); //清除标志
        
        asm("nop");asm("nop");
        uint16_t dat = GPIO_ReadInputData(GPIOC);
        USART_SendData(USART1, dat >> 8);
        USART_SendData(USART1, dat & 0xFF);
    }
}

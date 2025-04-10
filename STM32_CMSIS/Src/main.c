

#include <stdio.h>
#include "main.h" 
#include "stm32f10x.h"    
#include <time.h>
#include <RFID.h>
#include <stdlib.h> 
#include "delay.h" 
#include <string.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "lcd16x2.h"
#include "keypad4x4-scanning.h"
#define KEYPAD_NO_KEY 0xFF
#include <string.h>
/**
  * @brief  	Main program.
  * @param  	None
  * @retval 	None
  */
  
/*----------------------------------------------------------------------------
					MAIN Function
 *----------------------------------------------------------------------------*/
int main(void)
{    
	/*--------------------------------------------------------*/
	/* Configure the system clocks */
	DelayInit();
	RCC_Configuration(); 
	/**/ 
	NVIC_Configuration(); 
	GPIO_Configuration(); 
	initUART2();	  
	//----RFID-----  
	TM_MFRC522_Init();  
  LCD_Init();

LEDInit();
  while (1)
	{   
			//Delay(10);  
			/* Toggle pins */
			GPIO_WriteBit(GPIOC, GPIO_Pin_3, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_3)));
			GPIO_WriteBit(GPIOC, GPIO_Pin_4, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_4)));
			GPIO_WriteBit(GPIOC, GPIO_Pin_5, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_5)));    
			//-----------------------------RFID Analayzer------------------------------------------
		  if (TM_MFRC522_Check(CardID) == MI_OK) 
			{
				sprintf(bufferRFID, "[%02x-%02x-%02x-%02x-%02x]", CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
				
				//ham kiem dung the
			  if (TM_MFRC522_Compare(CardID, MyID) == MI_OK) 
				{
					
					LCD_Gotoxy(0,1);
					LCD_Puts("Lam On Dung Di Troi");
					sprintf(bufferRFID, "[%s voi Card ID=%02x-%02x-%02x-%02x-%02x]", dataUDP_permit, CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
				
				}
				else
				{
					
					LCD_Gotoxy(0,1);
					LCD_Puts("The Sai, Quet Lai");
					sprintf(bufferRFID, "[%s voi Card ID=%02x-%02x-%02x-%02x-%02x]", dataUDP_deny, CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
				 	
				}
				
			}  
			Send_string_uart("Waiting for RFID Card...!\n\r");
			LCD_Gotoxy(0,0);
			LCD_Puts("Xin Moi Quet The!");
         
			Delay(10);  
	}
	 
 
} /*--------------End Main------------------*/

/*
*************************************************************************************************************************************
*							  								LOCAL FUNCTIONS															*
*************************************************************************************************************************************
*/
int counterChar(const char* logData)
{
		int t=0;
		const char* x = logData;
		//x = logData;
	  
		while (*x)
		 {
			 *x++;
			 t++;
		 }
		 
	  return t; 
}
 
/**
  * @brief  	Configures the different system clocks.
  * @param  	None
  * @retval 	None
  */
void RCC_Configuration(void)
{
	/* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
	initialize the PLL and update the SystemFrequency variable. */
	//SystemInit();

	/* Enable GPIOA B C E and AFIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
	/**/
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
}

/**
  * @brief  	Inserts a delay time with resolution is 10 milisecond..
  * @param  	nCount: specifies the delay time length.
  * @retval 	None
  */
void Delay(__IO uint32_t num)
{
	__IO uint32_t index = 0;

	/* default system clock is 72MHz */
	for(index = (720000 * num); index != 0; index--)
	{
	}
}
 
/**
  * @brief  	Make Delay using Loop CPU 
  * @param  	uint32_t Delay in milisecond
  * @retval 	None
  */
void Delay_us(uint32_t us){
	 uint32_t temp;
	 temp = us * 6; // cho 24 MHZ
	 while(temp > 0)
	 {
		 temp --;
	 }
 }
 
void unless_loop()
{
  while(1)
	 {
		 Delay(20);
	 }
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
  
 
 /*----------------------------------------------------------------------------
  SendChar
  Write character to Serial Port.
 *----------------------------------------------------------------------------*/
int Send_int_uart(int ch)  {

  while (!(USARTz->SR & USART_FLAG_TXE));
  USARTz->DR = (ch & 0x1FF);

  return (ch);
}


/*----------------------------------------------------------------------------
  GetKey
  Read character to Serial Port.
 *----------------------------------------------------------------------------*/
int GetKeyt (void)  {

  while (!(USARTz->SR & USART_FLAG_RXNE));

  return ((int)(USARTz->DR & 0x1FF));
}
/*----------------------------------------------------------------------------
  Send string
 *----------------------------------------------------------------------------*/
void Send_string_uart(const char *str)
{
    while (*str)
    {
        while(USART_GetFlagStatus(USARTz, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTz, *str++);
    }
}
/*----------------------------------------------------------------------------
  Send char
 *----------------------------------------------------------------------------*/
void Send_char_uart(char str)
{
   
        while(USART_GetFlagStatus(USARTz, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTz, str);
    
}
///////////
 
/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  //NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USART3 Interrupt */
  //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 // NVIC_Init(&NVIC_InitStructure);
    
  /*Enable the TIMER 2 Interrupt*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
}

void GPIO_Configuration(void)
{
   	/*------------------------------------------------------------*/
		/*--------------------------------------------------------*/   
		/* Configure  PC.3, PC.4, PC.5 as output push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 ;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void TIM_Configuration(void)
{

 	/*Config for Timer 2*/

	/* Time base configuration */

	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* TIMER 2 enable */
 	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	
}
 
void initUART2(void)
{
  	/* 
	  *	configure USART 2 pins	
	  */	
   
	/* Enable UART clock  */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
	/*CONFIG REMAP FOR USART2 --> PA2, PA3*/

	/* Configure PA2 for USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PA3 for USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  /* Enable USART2 Receive and Transmit interrupts */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);
	/* Enable USART */
	USART_Cmd(USART2, ENABLE);	

}
 
 

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
 


#endif

int ComparePassword(const char *inputPassword, const char *correctPassword)
{
    return strcmp(inputPassword, correctPassword);
}
void DisplayCorrectPasswordLED()
{
    // B?t LED ? PB8
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    // T?t LED ? PB9
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
}

// Hàm hi?n th? LED khi m?t kh?u sai
void DisplayIncorrectPasswordLED()
{
    // B?t LED ? PC9
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    // T?t LED ? PC8
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}


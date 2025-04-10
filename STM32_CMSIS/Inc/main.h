/**
  ******************************************************************************
  * @file    	GPIO/JTAG_Remap/main.h
  * @author  	ARMVN Application Team
  * @version 	V1.0.0
  * @date    	01/30/2010
  * @brief   	Header file for main.c module.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 ARMVietNam</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif 

/*
*************************************************************************************************************************************
*															INCLUDED FILES															*
*************************************************************************************************************************************
*/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"	  
#include "stm32f10x_usart.h"
#include "misc.h"
#include "time.h"

void RCC_Configuration(void);
void Delay(__IO uint32_t nCount);

/* Private define ------------------------------------------------------------*/  
#define PSTR(s) s  
#define USARTz    USART2 
#define TxBufferSize2  (countof(TxBuffer2) - 1)
#define TxBufferSize3   (countof(TxBuffer3) - 1)
#define RxBufferSize2   TxBufferSize3
#define RxBufferSize3   TxBufferSize2
/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus; 

typedef unsigned long	ULONG;
typedef unsigned long	DWORD;

/* Private variables ---------------------------------------------------------*/
char c; 
char greeting[] = "Hello cac ban!";  
// please modify the following two lines. mac and ip have to be unique
// in your local area network. You can not have the same numbers in
// two devices:  
unsigned char mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x24}; //MAC cua STM32
unsigned char PC[6] = 	 {0xb8,0x2a,0x72,0xf1,0xb8,0xd8}; //MAC cua PC
unsigned char myip[4] =  {192,168,1,25}; //0xc0,0xa8,0x01,0x19           
unsigned char PC_ip[4] = {192,168,1,95}; //0xc0,0xa8,0x01,0x5f
// base url (you can put a DNS name instead of an IP addr. if you have
// a DNS server (baseurl must end in "/"):
//char baseurl[]="http://192.168.1.25/";
unsigned int myport = 100; // listen port for tcp/www (max range 1-254)   
unsigned int myudpport =1200; // listen port for udp
// how did I get the mac addr? Translate the first 3 numbers into ascii is: TUX __ lockup in ASCII TUX is  
#define BUFFER_SIZE 1500//400
unsigned char buf[BUFFER_SIZE+1]; 
unsigned char bufRemeber[BUFFER_SIZE+1];
// the password string (only the first 5 char checked), (only a-z,0-9,_ characters):
char password[]="123456"; // must not be longer than 9 char  
int counter = 8;
__IO uint32_t index1 = 0;  

//
unsigned char UDP_pactkage_sample[200] = 
   { 
		  0xb8,0x2a,0x72,0xf1,0xb8,0xd8,   //MAC DST 
		  0x54,0x55,0x58,0x10,0x00,0x24,   //MAC SCR  11
			0x08,0x00, 											 //UDP - 12 13
			0x45,  //14 IP_P
			0x00, //15
			0x00,0x29,  //16 17 
			0x7c,0x61, //18 19
			0x40,0x00, // 20 21
			0x40, //22 
			0x11, //23
			0x3a,0x9a, //24 25
			0xc0,0xa8,0x01,0x19, 	//IP SCR  //29
			0xc0,0xa8,0x01,0x5f, 	//IP DST  //33
			0x04,0xb0,  //34 35
			0x00,0xb0,  //36 37
			0x00,0x15,  //38 39
			0x93,0xd5,  //40 41
			0x06,0x44,0x50,0x20,0x58,0x69,0x6e,0x20,0x43,0x68,0x61,0x6f,0x21,  //DATA  // 54
			0x00,0x00,0x00,0x00,0x00}; //59 
			
unsigned char TCP_pactkage_sample_1[200] = 
   {	0xb8,0x2a,0x72,0xf1,0xb8,0xd8,   //MAC DST
			0x54,0x55,0x58,0x10,0x00,0x24,   //MAC SRC  11
			0x08,0x00, 											 //UDP - 12 13
			0x45,  //14 IP_P
			0x00, //15
			0x00,0x35,  //16 17 
			0x70,0xc7, //18 19
			0x40,0x00, // 20 21
			0x40, //22 
			0x06, //23
			0x46,0x33, //24 25
			0xc0,0xa8,0x01,0x19, 	//IP SRC  //29
			0xc0,0xa8,0x01,0x5f, 	//IP DST  //33
			0x00,0x64,  //34 35 //PORT SRC
			0x21,0xce,  //36 37 //PORT DST
			0x00,0x00,  //38 39
			0x0a,0x01,  //40 41
		  0x28,0x9a,0x3a,0xb1,//42 43 44 45
		  0x50,//46
		  0x18,//47  //Flags
		  0x28,0xb8,//48 49
		  0xe2,0xbd,//50 51
			0x00,0x00,//52 53
			0x55,0x44,0x50,0x20,0x58,0x69,0x6e,0x20,0x43,0x68,0x61,0x6f,0x21}; //54 55 56 57 58 59 60 61 62 63 64 65	66	
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char c; 
  
uint8_t TxBuffer2[] = "USART Interrupt Example: USART2 -> USART3 using Interrupt";
uint8_t TxBuffer3[] = "USART Interrupt Example: USART3 -> USART2 using Interrupt";
uint8_t RxBuffer2[RxBufferSize2];
uint8_t RxBuffer3[RxBufferSize3];
 __IO uint8_t TxCounter2 = 0x00;
 __IO uint8_t TxCounter3 = 0x00;
 __IO uint8_t RxCounter2 = 0x00;
 __IO uint8_t RxCounter3 = 0x00;

/* For TIMER */
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
			
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void TIM_Configuration(void);
void initUART2(void);
void I2C(void);
void Delay(uint32_t);

int  GetKey(void);
int  Send_int_uart(int);
void Send_string_uart(const char*);
void Send_char_uart(char); 
int counterChar(const char* logData);
void netAnalyzer(void);
void EXTI5_Config(void);
/* Private function prototypes -----------------------------------------------*/
void EXTI0_Config(void);
void EXTI9_5_Config(void);

unsigned char bufferRFID[30];
unsigned char CardID[5];
unsigned char MyID[5] = {0x82, 0x1f, 0x2f, 0x00, 0xb2};	//My card on my keys
	
int randomNumber1;
int randomNumber2;

int len; 
int k;
int counter;

char cmdval;
unsigned char status;
	
unsigned int plen;
unsigned int dat_p;
unsigned char i=0;
unsigned char cmd_pos=0;
unsigned char cmd;
unsigned char payloadlen=0;
char str[30] = "UDP Xin Chao!";
char dataUDP_deny[30] =   "Co canh bao dot nhap du lieu!";
char dataUDP_permit[30] = "Truy cap thiet bi hop le!";
	
void unless_loop(void);
 /*
*************************************************************************************************************************************
*							  						   		GLOBAL FUNCTION PROTOTYPES												*
*************************************************************************************************************************************
*/




#ifdef __cplusplus
}
#endif


#endif /* __MAIN_H */

/**
  * @}
  */

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 ARMVietNam *****END OF FILE****/

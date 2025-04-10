#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "lcd16x2.h"
#include "delay.h"

int main(void)
{
    // Delay initialization
    DelayInit();
    // LCD initialization
    LCD_Init();
        LCD_Gotoxy(2,0); // ghi lenh 
        LCD_Puts("KY THUAT VI XU"); // ghi ra man hinh 
        LCD_Gotoxy(0,1);
        LCD_Puts("LY - PTIT");
	      DelayMs(500);
}

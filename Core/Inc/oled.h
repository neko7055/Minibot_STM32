#ifndef __OLED_H
#define __OLED_H			  	 
#include "main.h"

#define OLED_RST_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define OLED_RST_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)

#define OLED_RS_Clr() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define OLED_RS_Set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

#define OLED_SCLK_Clr()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define OLED_SCLK_Set()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) 

#define OLED_SDIN_Clr()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define OLED_SDIN_Set()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)

#define OLED_CMD  0	
#define OLED_DATA 1	

void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const char *p);	 
#endif  
	 

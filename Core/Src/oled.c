#include "oled.h"
#include "oledfont.h"  	
		   
uint8_t OLED_GRAM[128][8];	 
void OLED_Refresh_Gram(void)
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);
		OLED_WR_Byte (0x00,OLED_CMD);
		OLED_WR_Byte (0x10,OLED_CMD);
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{	
	uint8_t i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 

void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);
	OLED_WR_Byte(0X14,OLED_CMD);
	OLED_WR_Byte(0XAF,OLED_CMD);
}
   
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);
	OLED_WR_Byte(0X10,OLED_CMD);
	OLED_WR_Byte(0XAE,OLED_CMD);
}		   			 
  
void OLED_Clear(void)  
{  
	uint8_t i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();
}
		   
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{      			    
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' ';			   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];
		else temp=oled_asc2_1608[chr][t];                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}

uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 

void OLED_ShowString(uint8_t x,uint8_t y,const char *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	   

void OLED_Init(void)
{ 	
	OLED_RST_Clr();
	HAL_Delay(100);
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD); 
	OLED_WR_Byte(0xD5,OLED_CMD); 
	OLED_WR_Byte(80,OLED_CMD);  
	OLED_WR_Byte(0xA8,OLED_CMD); 
	OLED_WR_Byte(0X3F,OLED_CMD); 
	OLED_WR_Byte(0xD3,OLED_CMD);
	OLED_WR_Byte(0X00,OLED_CMD); 

	OLED_WR_Byte(0x40,OLED_CMD); 
													    
	OLED_WR_Byte(0x8D,OLED_CMD); 
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_WR_Byte(0x20,OLED_CMD);
	OLED_WR_Byte(0x02,OLED_CMD); 
	OLED_WR_Byte(0xA1,OLED_CMD); 
	OLED_WR_Byte(0xC0,OLED_CMD); 
	OLED_WR_Byte(0xDA,OLED_CMD); 
	OLED_WR_Byte(0x12,OLED_CMD);
		 
	OLED_WR_Byte(0x81,OLED_CMD);
	OLED_WR_Byte(0xEF,OLED_CMD); 
	OLED_WR_Byte(0xD9,OLED_CMD);
	OLED_WR_Byte(0xf1,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);
	OLED_WR_Byte(0x30,OLED_CMD);

	OLED_WR_Byte(0xA4,OLED_CMD);
	OLED_WR_Byte(0xA6,OLED_CMD);    						   
	OLED_WR_Byte(0xAF,OLED_CMD);
	OLED_Clear();
}  





#ifndef __LCD_H
#define __LCD_H
#include "stdlib.h"
#include "main.h"

typedef struct
{
	uint16_t width;
	uint16_t height;
	uint16_t id;
	uint8_t  dir;
	uint16_t wramcmd;
	uint16_t setxcmd;
	uint16_t setycmd;
	uint8_t  xoffset;
	uint8_t	 yoffset;
}_lcd_dev;

_lcd_dev lcddev;

//extern SPI_HandleTypeDef hspi1;
extern uint8_t lcd_flag;
/////////////////////////////////////ÓÃ»§ÅäÖÃÇø///////////////////////////////////
#define USE_HORIZONTAL  	 1

//////////////////////////////////////////////////////////////////////////////////
//¶¨ÒåLCDµÄ³ß´ç
#define LCD_W 135
#define LCD_H 240

#define GPIO_TYPE  	GPIOB
#define LCD_CS   	8
#define LCD_RS   	9
#define LCD_RST   	13


#define	LCD_CS_SET  GPIO_TYPE->BSRR=1<<LCD_CS
#define	LCD_RS_SET	GPIO_TYPE->BSRR=1<<LCD_RS
#define	LCD_RST_SET	GPIOC->BSRR=1<<LCD_RST

#define	LCD_CS_CLR  GPIO_TYPE->BRR=1<<LCD_CS
#define	LCD_RS_CLR	GPIO_TYPE->BRR=1<<LCD_RS
#define	LCD_RST_CLR	GPIOC->BRR=1<<LCD_RST


#define WHITE       	0xFFFF
#define BLACK      		0x0000
#define BLUE       		0x001F
#define BRED        	0XF81F
#define GRED 			0XFFE0
#define GBLUE			0X07FF
#define RED         	0xF800
#define MAGENTA     	0xF81F
#define GREEN       	0x07E0
#define CYAN        	0x7FFF
#define YELLOW      	0xFFE0
#define BROWN 			0XBC40
#define BRRED 			0XFC07

#define GRAY0           0xe71c// 0b1110011100011100
#define GRAY1           0xc618// 0b1100011000011000
#define GRAY2           0xa514// 0b1010010100010100
#define GRAY3           0x8410// 0b1000010000010000
#define GRAY4           0x630c// 0b0110001100001100
#define GRAY5           0x4208// 0b0100001000001000
#define GRAY6           0x2104// 0b0010000100000100


#define DARKBLUE      	0X01CF
#define LIGHTBLUE      	0X7D7C
#define GRAYBLUE       	0X5458
#define LIGHTGREEN     	0X841F
#define LIGHTGRAY     	0XEF5B
#define LGRAY 			0XC618
#define LGRAYBLUE      	0XA651
#define LBBLUE          0X2B12


uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;
uint16_t DeviceCode;

void  SPIv_WriteData(uint8_t Data)
{
	/*
	unsigned char i=0;
	for(i=8;i>0;i--)
	{
	  if(Data&0x80) HAL_GPIO_WritePin(spi_data_GPIO_Port, spi_data_Pin,GPIO_PIN_SET);
      else HAL_GPIO_WritePin(spi_data_GPIO_Port, spi_data_Pin,GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(spi_clk_GPIO_Port, spi_clk_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(spi_clk_GPIO_Port, spi_clk_Pin,GPIO_PIN_RESET);
      Data<<=1;
	}
	*/
}

void SPI_WriteByte(uint8_t Byte)
{
	while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
	LL_SPI_TransmitData8(SPI1, Byte);
	//(void) SPI1->DR; //fake Rx read;
	while (LL_SPI_IsActiveFlag_BSY(SPI1)){}
}

/*****************************************************************************
 * @name       :void LCD_WR_REG(u8 data)
 * @date       :2018-08-09
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(uint8_t data)
{
	LCD_CS_CLR;
	LCD_RS_CLR;
	SPI_WriteByte(data);
	LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u8 data)
 * @date       :2018-08-09
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(uint8_t data)
{
	LCD_CS_CLR;
	LCD_RS_SET;
	SPI_WriteByte(data);
	LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(u16 Data)
 * @date       :2018-08-09
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/
void Lcd_WriteData_16Bit(uint16_t Data)
{
   LCD_CS_CLR;
   LCD_RS_SET;
   SPIv_WriteByte(SPI1,Data>>8);
   SPIv_WriteByte(SPI1,Data);
   LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(uint16_t x,uint16_t y)
 * @date       :2018-08-09
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/
void LCD_Pixel(uint16_t x,uint16_t y,uint16_t color)
{
#if(USE_HORIZONTAL)
	LCD_SetCursor(x,y);
#else
	LCD_SetCursor(y,x);
#endif
	LCD_CS_CLR;
	LCD_RS_SET;
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_16BIT );
	while(!(SPI1->SR & SPI_SR_TXE));
	LL_SPI_TransmitData16(SPI1, color);
	while (SPI1->SR & SPI_SR_BSY);
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );
	LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_Clear(uint16_t Color)
 * @date       :2018-08-09
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/
void LCD_Clear(uint16_t Color)
{
	unsigned int i,m;
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
	LCD_CS_CLR;
	LCD_RS_SET;
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_16BIT );
	for(i=0;i<lcddev.height;i++)
	{
		for(m=0;m<lcddev.width;m++)
		{
		    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
		    LL_SPI_TransmitData16 (SPI1, Color);
		    //(void) SPI1->DR; //fake Rx read;
		    while (LL_SPI_IsActiveFlag_BSY(SPI1)){}
		}
	}
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );
	LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void LCD_RESET(void)
{
	LCD_RST_CLR;
	HAL_Delay(20);
	LCD_RST_SET;
	HAL_Delay(20);
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void LCD_Init(void)
{
//	SPI2_Init(); //Ó²¼şSPI2³õÊ¼»¯
//	LCD_GPIOInit();//LCD GPIO³õÊ¼»¯
 	LCD_RESET();
//************* ST7789V³õÊ¼»¯**********//
	HAL_Delay(120);
	LCD_WR_REG(0x11);     //Sleep out
	HAL_Delay(120);

	LCD_WR_REG(0x36);	//MADCTL	-  MY MX MV ML RGB MH -  -
	LCD_WR_DATA(0x00);	//          0  0  0  0  0   0  0  0  0

	LCD_WR_REG(0x3A);	//COLMOD
	LCD_WR_DATA(0x05);  //0x05( 65K Color)

	LCD_WR_REG(0x21);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x33);
	LCD_WR_DATA(0x33);

	LCD_WR_REG(0xB7);
	LCD_WR_DATA(0x23);

	LCD_WR_REG(0xBB);
	LCD_WR_DATA(0x22);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA(0x2C);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA(0x13);

	LCD_WR_REG(0xC4);
	LCD_WR_DATA(0x20);

	LCD_WR_REG(0xC6);
	LCD_WR_DATA(0x0F);

	LCD_WR_REG(0xD0);
	LCD_WR_DATA(0xA4);
	LCD_WR_DATA(0xA1);

	LCD_WR_REG(0xD6);
	LCD_WR_DATA(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0x70);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x27);
	LCD_WR_DATA(0x2E);
	LCD_WR_DATA(0x34);
	LCD_WR_DATA(0x46);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x25);
	LCD_WR_DATA(0x2A);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA(0x70);
	LCD_WR_DATA(0x04);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x42);
	LCD_WR_DATA(0x42);
	LCD_WR_DATA(0x38);
	LCD_WR_DATA(0x14);
	LCD_WR_DATA(0x14);
	LCD_WR_DATA(0x27);
	LCD_WR_DATA(0x2C);

	LCD_WR_REG(0x29);     //Display on

	LCD_WR_REG(0x33);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0);


	LCD_WR_REG(0x11);     //Sleep out

	LCD_direction(USE_HORIZONTAL);//ÉèÖÃLCDÏÔÊ¾·½Ïò
	LCD_Clear(BLUE);
}

/*****************************************************************************
 * @name       :void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
 * @date       :2018-08-09
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{
	LCD_WR_REG(lcddev.setxcmd);
	LCD_WR_DATA((xStar+lcddev.xoffset)>>8);
	LCD_WR_DATA(xStar+lcddev.xoffset);
	LCD_WR_DATA((xEnd+lcddev.xoffset)>>8);
	LCD_WR_DATA(xEnd+lcddev.xoffset);

	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA((yStar+lcddev.yoffset)>>8);
	LCD_WR_DATA(yStar+lcddev.yoffset);
	LCD_WR_DATA((yEnd+lcddev.yoffset)>>8);
	LCD_WR_DATA(yEnd+lcddev.yoffset);

	LCD_WriteRAM_Prepare();	//¿ªÊ¼Ğ´ÈëGRAM
}

/*****************************************************************************
 * @name       :void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
 * @date       :2018-08-09
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);
}

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/
void LCD_direction(uint8_t direction)
{
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
	switch(direction){
		case 0:
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;
			lcddev.xoffset=52;
			lcddev.yoffset=40;
			LCD_WriteReg(0x36,0);//BGR==1,MY==0,MX==0,MV==0
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			lcddev.xoffset=40;
			lcddev.yoffset=53;
			LCD_WriteReg(0x36,(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;
		case 2:
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;
      		lcddev.xoffset=53;
			lcddev.yoffset=40;
			LCD_WriteReg(0x36,(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			lcddev.xoffset=40;
			lcddev.yoffset=52;
			LCD_WriteReg(0x36,(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
		break;
		default:break;
	}
}

void Send_DMA_Data16(uint16_t* buff, uint16_t dataSize)
{
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);

    LL_SPI_Disable(SPI1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, dataSize);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)buff, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);

    while (!lcd_flag) {}
    lcd_flag = 0;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Disable(SPI1);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
}

void Send_DMA_Data8(uint8_t* buff, uint16_t dataSize)
{

    LL_SPI_Disable(SPI1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, dataSize);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)buff, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);

    while (!lcd_flag) {}
    lcd_flag = 0;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Disable(SPI1);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);
}

void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{
    uint16_t tbuf[w];
    LCD_SetWindows(x1, y1, (uint16_t) (x1 + w - 1), (uint16_t) (y1 + h - 1));
    LCD_CS_CLR;
    LCD_RS_SET;

    for (int x = w ; x >= 0; x--) tbuf[x] = color;
    for (y1 = h; y1 > 0; y1--) Send_DMA_Data16(tbuf,w);

    LCD_CS_SET;
}

#endif

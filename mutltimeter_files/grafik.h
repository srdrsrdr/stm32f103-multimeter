#ifndef INC_GRAFIK_H_
#define INC_GRAFIK_H_

#include "Fonts/Font16.h"
#include "font.h"
#include "lcd.h"

//---------------------Buton Durumu---------------------
#define		Basili			1
#define		BasiliDegil		0
//------------------------------------------------------

//---------------------   Hizalama  ---------------------
#define		SolaDayali			1
#define		SagaDayali			2
#define		UsteDayali			3
#define		AltaDayali			4
#define		Ortala				0
//------------------------------------------------------

#define 	wordWrap 			1

uint8_t	*wTbl;
const uint8_t	*pointerTbl[1];
uint8_t fontHeigh = 16;


void TFT_Set_Address(uint16_t PX1,uint16_t PY1,uint16_t PX2,uint16_t PY2)
{
  Write_Command_Data(68,(PX2 << 8) + PX1 );  //Column address start2
  Write_Command_Data(69,PY1);      //Column address start1
  Write_Command_Data(70,PY2);  //Column address end2
  Write_Command_Data(78,PX1);      //Column address end1
  Write_Command_Data(79,PY1);  //Row address start2
  Write_Command(34);
}

void TFT_fill_rect(uint16_t y1,uint16_t x1,uint16_t y2,uint16_t x2,uint16_t color)
{
	LCD_fillRect(x1,y1,x2-x1, y2-y1, color);
}

void LCD_Line(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	int x,y,addx,addy,dx,dy;
	long P;
	uint16_t i;

		if(x2>x1) dx = x2-x1; else dx = x1-x2;
		if(y2>y1) dy = y2-y1; else dy = y1-y2;

	  x = x1;
	  y = y1;

	   if(x1 > x2)
	   {
		   addx = -1;
	   }
	   else
	   {
		   addx = 1;
	   }

	   if(y1 > y2)
	   {
		  addy = -1;
	   }
	   else
	   {
		  addy = 1;
	   }


	 if(dx >= dy)
	 {

	  P = (2*dy) - dx;

	   for(i = 1; i <= (dx +1); i++)
	   {

		 LCD_Pixel(x,y,color);

		 if(P < 0)
		 {
			 P = P + (2*dy);
			 x = (x + addx);
		 }
		 else
		 {
			P = P+(2*dy) - (2*dx);
			x = x + addx;
			y = y + addy;
		 }
		}
	  }
	  else
	  {
		P = (2*dx) - dy;

		for(i = 1; i <= (dy +1); i++)
		{

		 LCD_Pixel(x,y,color);

		 if(P<0)
		 {
		   P = P + (2*dx);
		   y = y + addy;
		 }
		 else
		 {
			P = P + (2*dx) - (2*dy);
			x = x + addx;
			y = y + addy;
		 }
		}
	   }
}

void Line_H(uint16_t x1,uint16_t y,uint16_t x2,uint16_t color)
{
	LCD_fillRect(x1,y,x2-x1,1,color);
}

void Line_V(uint16_t x_pos,uint16_t y1,char y2,uint16_t color)
{
	LCD_fillRect(x_pos,y1,1,y2-y1,color);
}

void Line2_H(uint16_t x1,uint16_t y_pos,uint16_t x2,uint16_t color)
{
	LCD_fillRect(x1,y_pos,x2-x1,2,color);
}

void Line2_V(uint16_t x_pos,uint16_t y1,char y2,uint16_t color)
{
	LCD_fillRect(x_pos,y1,2,y2-y1,color);
}

void Rectangle(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	Line_H(x1, y1, x2, color);
	Line_H(x1, y2, x2, color);
	Line_V(x1, y1, y2, color);
	Line_V(x2, y1, y2, color);
}

void Fill_Circle(int X,int Y,int Radius,int color)
{
   int a, b, P;
   a = 0;
   b = Radius;
   P = 1 - Radius;

   do
   {
      LCD_Line(X-a, Y+b, X+a, Y+b, color);
      LCD_Line(X-a, Y-b, X+a, Y-b, color);
      LCD_Line(X-b, Y+a, X+b, Y+a, color);
      LCD_Line(X-b, Y-a, X+b, Y-a, color);
      if(P < 0)
         P+= 3 + 2*a++;
      else
         P+= 5 + 2*(a++ - b--);
    } while(a <= b);
}

void Circle(int X,int Y,int Radius,int renk)
{
   int f;
   int ddF_x;
   int ddF_y;
   int x;
   int y;

   f = 1 - Radius;
   ddF_x = 0;
   ddF_y = -2 * Radius;
   x = 0;
   y = Radius;

   LCD_Pixel(X, Y + Radius, renk);
   LCD_Pixel(X, Y - Radius, renk);
   LCD_Pixel(X + Radius, Y, renk);
   LCD_Pixel(X - Radius, Y, renk);

   while (x < y)
   {   if (f >= 0)
      {   y--;
         ddF_y += 2;
         f += ddF_y;
      }

      x++;
      ddF_x += 2;
      f += ddF_x + 1;

      LCD_Pixel(X + x, Y + y, renk);
      LCD_Pixel(X - x, Y + y, renk);
      LCD_Pixel(X + x, Y - y, renk);
      LCD_Pixel(X - x, Y - y, renk);
      LCD_Pixel(X + y, Y + x, renk);
      LCD_Pixel(X - y, Y + x, renk);
      LCD_Pixel(X + y, Y - x, renk);
      LCD_Pixel(X - y, Y - x, renk);
   }
}

void TFT_Char8x8(char C,uint16_t x,uint16_t y,uint16_t Fcolor,uint16_t Bcolor)
{
	uint8_t mask,tmp;
	uint16_t Cptrfont;
	Cptrfont = (C-32)*8;

	LCD_SetWindows(x,y,x+7,y+7);

	LCD_CS_CLR;
	LCD_RS_SET;
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_16BIT );
	for(uint8_t i=0;i<8;i++)
	{
		tmp = FONT_8x8[Cptrfont];
		Cptrfont++;
		mask=0x80;
		for(uint8_t si=0;si<8;si++)
		{
			while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
			if(tmp & mask) 		LL_SPI_TransmitData16 (SPI1, Fcolor);
			else				LL_SPI_TransmitData16 (SPI1, Bcolor);
			mask>>=1;
			while (LL_SPI_IsActiveFlag_BSY(SPI1)){}
		}
	}
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );
	LCD_CS_SET;
}

void TFT_Char16x16(char C,uint16_t x,uint16_t y,uint16_t Fcolor,uint16_t Bcolor)
{
	uint8_t mask,tmp;
	uint16_t Cptrfont;
	Cptrfont = (C-32)*32;

	LCD_SetWindows(x,y,x+15,y+15);

	LCD_CS_CLR;
	LCD_RS_SET;
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_16BIT );
	for(uint8_t i=0;i<32;i++)
	{
		tmp = FONT_16x16[Cptrfont];
		Cptrfont++;
		mask=0x80;

		do{
			while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
			if(tmp & mask) 		LL_SPI_TransmitData16 (SPI1, Fcolor);
			else				LL_SPI_TransmitData16 (SPI1, Bcolor);
			mask>>=1;
			while (LL_SPI_IsActiveFlag_BSY(SPI1)){}
		}while(mask);
	}
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );
	LCD_CS_SET;
}

void TFT_Char48x64(char C,uint16_t x,uint16_t y,uint16_t Fcolor,uint16_t Bcolor)
{
	uint8_t mask,tmp;
	uint16_t Cptrfont;
	Cptrfont = (C-48)*384;

	LCD_SetWindows(x,y,x+63,y+47);

	LCD_CS_CLR;
	LCD_RS_SET;
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_16BIT );
	for(uint8_t i=0;i<lcddev.height;i++)
	{
		tmp = FONT_48x64[Cptrfont];
		Cptrfont++;
		mask=0x80;
		for(uint8_t si=0;si<8;si++)
		{
			while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
			if(tmp & mask) 		LL_SPI_TransmitData16 (SPI1, Fcolor);
			else				LL_SPI_TransmitData16 (SPI1, Bcolor);
			mask>>=1;
			while (LL_SPI_IsActiveFlag_BSY(SPI1)){}
		}
	}
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );
	LCD_CS_SET;
}

void TFT_Char24x30(char C,uint16_t y,uint16_t x,uint16_t Fcolor,uint16_t Bcolor)
{
	uint8_t mask,tmp;
	uint16_t Cptrfont;
	Cptrfont = (C-48)*90;

	LCD_CS_CLR;
	LCD_RS_SET;
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_16BIT );
	for(uint8_t i=0;i<lcddev.height;i++)
	{
		tmp = FONT_24x30[Cptrfont];
		Cptrfont++;
		mask=0x80;
		for(uint8_t si=0;si<8;si++)
		{
			while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
			if(tmp & mask) 		LL_SPI_TransmitData16 (SPI1, Fcolor);
			else				LL_SPI_TransmitData16 (SPI1, Bcolor);
			mask>>=1;
			while (LL_SPI_IsActiveFlag_BSY(SPI1)){}
		}
	}
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );
	LCD_CS_SET;
}
/*
void TFT_Char(char C,uint16_t y,uint16_t x,char DimFont,uint16_t Fcolor,uint16_t Bcolor)
{
const char *ptrFont;
uint16_t Cptrfont;
uint16_t font8x8[8];
uint16_t font16x16[16];
char k,i,print1,print2;
uint16_t print3,print4;

if(DimFont == 8)
{
     ptrFont = &FONT_8x8;
     Cptrfont = (C-32)*8;
     ptrFont = ptrFont + Cptrfont;

     for(k = 0; k <= 7; k++)
     {
      font8x8[k] = *ptrFont;
      ptrFont++;
     }

     TFT_CS = 0;
     TFT_Set_Address(x,y,x+7,y+7);
     for(i = 0; i <= 7; i++)
     {
       for(k = 0; k <= 7; k++)
       {
          print1 = (font8x8[i] & 0x80);
          print2 = print1 >>7;
          if(print2 == 1)
          {
             Write_Data(Fcolor);
          }
          else
          {
             Write_Data(Bcolor);
          }
          font8x8[i] = font8x8[i] << 1;
       }
     }
     TFT_CS = 1;
}

else if(DimFont == 16)
{
     ptrFont = &FONT_16x16;
     Cptrfont = (C-32)*32;
     ptrFont = ptrFont + Cptrfont;

     for(k = 0; k <= 15; k++)
     {
      font16x16[k] = *ptrFont;
      ptrFont++;
      font16x16[k] = (font16x16[k] << 8);
      font16x16[k] = font16x16[k] + *ptrFont;
      ptrFont++;
     }

     TFT_CS = 0;
     TFT_Set_Address(x,y,x+15,y+15);
     for(i = 0; i <= 15; i++)
     {
       for(k = 0; k <= 15; k++)
       {
        print3 = (font16x16[i] & 0x8000);
        print4 = print3 >>15;
        if(print4 == 1)
        {
           Write_Data(Fcolor);
        }
        else
        {
           Write_Data(Bcolor);
        }

        font16x16[i] = font16x16[i] << 1;
       }
     }
     TFT_CS = 1;
}
}
*/

void LCD_Text(char *S,uint16_t x,uint16_t y,char DimFont,uint16_t Fcolor,uint16_t Bcolor)
{
  int lenght,cnt;
  char buffer[24];
  lenght = strlen(S);

  for(cnt = 0; cnt <= (lenght - 1); cnt++)
  {
    buffer[cnt] = S[cnt];
  }

  if(DimFont == 8)
  {
      for(cnt = 0; cnt <= (lenght - 1); cnt++)
      {
        TFT_Char8x8(buffer[cnt],x,y,Fcolor,Bcolor);
        x = x + 8;
      }
  }
  else if(DimFont == 16)
  {
      for(cnt = 0; cnt <= (lenght - 1); cnt++)
      {
        TFT_Char16x16(buffer[cnt],x,y,Fcolor,Bcolor);
        x = x + 16;
      }
  }
  if(DimFont == 48)
  {
      for(cnt = 0; cnt <= (lenght - 1); cnt++)
      {

        TFT_Char48x64(buffer[cnt],x,y,Fcolor,Bcolor);
        x = x + 48;
      }
  }
  if(DimFont == 24)
  {
      for(cnt = 0; cnt <= (lenght - 1); cnt++)
      {

        TFT_Char24x30(buffer[cnt],x,y,Fcolor,Bcolor);
        x = x + 24;
      }
  }
}

uint16_t stringLen(uint8_t *str ,uint8_t *wt)
{
	uint16_t ret = 0;
	while(*str){
		ret += *(wt + (*str - 32));
		str++;
	}
	return ret;
}

uint8_t textWitdh(uint8_t c)
{
	return *(wTbl + c -32);
}

uint8_t drawChar(uint8_t c,uint8_t x, uint8_t y, uint16_t Fcolor, uint16_t Bcolor)
{
	uint8_t chrNo = c - 32;
	uint8_t w = *(wTbl + chrNo);
	uint8_t mask;

	uint32_t *dizi = *pointerTbl;
	dizi += chrNo;

	uint8_t *data = (uint8_t*) *dizi;

	LCD_SetWindows(x, y, x + w - 1, y + fontHeigh - 1);

	LCD_CS_CLR;
	LCD_RS_SET;
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_16BIT );
	for(uint8_t h = 3;h < fontHeigh + 3;h++)
	{
		mask = 0x80;
		for(uint8_t i = 0;i < w;i++)
		{
			if(mask == 0)
			{
				mask = 0x80;
				data++;
			}
			while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
			if(*data & mask) 	LL_SPI_TransmitData16 (SPI1, Fcolor);
			else				LL_SPI_TransmitData16 (SPI1, Bcolor);
			mask>>=1;
			while (LL_SPI_IsActiveFlag_BSY(SPI1)){}
		}
		data++;
	}
	LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );
	LCD_CS_SET;

	return x + w;
}

void drawText(uint8_t* string,uint8_t x,uint8_t y,uint8_t fontNo,uint16_t Fcolor,uint16_t Bcolor)
{
	wTbl = (uint8_t*) &widtbl_f16;
	*pointerTbl = &chrtbl_f16[0];

	while(*string)
	{
		x = drawChar(*string, x, y, Fcolor, Bcolor);
		string++;
	}
}

/*
void TFT_Image(uint16_t pos_y,uint16_t pos_x,uint16_t dim_y,uint16_t dim_x,uint16_t *picture){
    uint16_t x, y;

    TFT_CS  = 0;
    TFT_Set_Address(pos_x, pos_y, pos_x + dim_x - 1, pos_y + dim_y - 1);
    for(y = pos_y; y < (pos_y + dim_y); y++ ) {
        for(x = pos_x; x < (pos_x + dim_x); x++ ) {
            Write_Data(*picture++);
        }
    }
    TFT_CS = 1;
}

void Frame1(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t framecolor,uint16_t backcolor)
{
		TFT_Rectangle(x1,y1,x2,y2,framecolor);
		TFT_Rectangle(x1+1,y1+1,x2-1,y2-1,framecolor);
}

void Frame2(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t framecolor,uint16_t backcolor)
{
		TFT_Rectangle(x1,y1,x2,y2,framecolor);
		TFT_Rectangle(x1+1,y1+1,x2-1,y2-1,framecolor);
		TFT_fill_rect(x1+2,y1+2,x2-2,y2-2,backcolor);
}

void ProggressBar(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t FColor,uint16_t VColor,uint16_t BColor,uint16_t MaxVal,uint16_t Val)
{
	TFT_Line_H(x1+2,y1,x2-2,FColor);
	TFT_Line_H(x1+1,y1+1,x2-1,FColor);
	TFT_Line_H(x1+1,y2-1,x2-1,FColor);
	TFT_Line_H(x1+2,y2,x2-2,FColor);
	TFT_Line2_V(x1,y1+2,y2-2,FColor);
	TFT_Line2_V(x2-1,y1+2,y2-2,FColor);

	TFT_fill_rect(x1+2,y1+2,x2-2,y2-2,BColor);
	Val=(Val*(x2-x1-4))/MaxVal;
	TFT_fill_rect(x1+2,y1+2,x1+Val,y2-2,VColor);
}

void Cetvel(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t VColor,uint16_t BColor,uint16_t BolmeSay,uint16_t Val)
{
	uint16_t a=x1,b=(x2-x1)/BolmeSay;
	TFT_Line2_H(x1,y1,x2,BColor);
	TFT_Line_V(a,y1,y1+3,BColor);
	for(uint8_t l=0;l<BolmeSay;l++){
		a+=b;
		TFT_Line_V(a,y1,y1+3,BColor);
	}

	a=x1+b*Val;
	Pixel(a,y1+7,VColor);
	TFT_Line_H(a-1,y1+8,a+1,VColor);
	TFT_Line_H(a-2,y1+9,a+2,VColor);
}


void MenuBar(uint16_t x1,uint16_t x2,uint16_t FColor,uint16_t BolmeSay)
{
	uint16_t ky,a,b=(x2-x1)/BolmeSay;
	for(uint8_t k=0;k<9;k++){
		ky=k*4;
		TFT_fill_rect(x1,0,x2,ky,GRAY1);
		TFT_Line2_H(x1+1,ky+1,x2-1,GRAY1);
		TFT_Line_H(x1+2,ky+2,x2-2,GRAY1);
		TFT_Line_H(x1+4,ky+3,x2-4,GRAY2);

		a=x1;
		for(uint8_t l=0;l<BolmeSay-1;l++){
			a+=b;
			TFT_Line_V(a,0,ky,White);
			TFT_Line_V(a+1,0,ky,GRAY2);
		}
		for(uint8_t t=0;t<3;t++) __delay_ms(10);
	}
}


void Buton(int x1,int y1,int x2,int y2,char *str,uint8_t FontW,char Bas)
{
	uint16_t clr1,clr2,clr3,str_wide;


	if(Bas)
	{
		clr1=GRAY5;
		clr2=GRAY4;
		clr3=GRAY2;
	}
	else
	{
		clr1=BLACK;
		clr2=GRAY2;
		clr3=GRAY4;
	}

	TFT_fill_rect(x1+3,y1+3,x2-3,y2-3,GRAY3);
	TFT_Line_H(x1,y1,x2,clr2);
	TFT_Line_H(x1,y2,x2,clr3);
	TFT_Line_H(x1+1,y1+1,x2-1,clr2);
	TFT_Line_H(x1+1,y2-1,x2-1,clr3);
	TFT_Line_H(x1+2,y1+2,x2-2,clr2);
	TFT_Line_H(x1+2,y2-2,x2-2,clr3);

	TFT_Line_V(x1,y1,y2,clr2);
	TFT_Line_V(x2,y1+1,y2,clr3);
	TFT_Line_V(x1+1,y1,y2-1,clr2);
	TFT_Line_V(x2-1,y1+2,y2,clr3);
	TFT_Line_V(x1+2,y1,y2-2,clr2);
	TFT_Line_V(x2-2,y1+3,y2,clr3);

	Pixel(x1,y1,clr3);
	Pixel(x1+1,y1+1,clr3);
	Pixel(x1+2,y1+2,clr3);

	Pixel(x2,y2,clr2);
	Pixel(x2-1,y2-1,clr2);
	Pixel(x2-2,y2-2,clr2);

	str_wide=(x2-x1-strlen(str)*FontW)/2;
	TFT_Text(str,x1+str_wide,y1+(y2-y1-FontW)/2,FontW,clr1,GRAY3);
}

void Buton2Line(int x1,int y1,int x2,int y2,char *str1,char *str2,uint8_t FontW,char Bas)
{
	uint16_t clr1,clr2,clr3,str_wide,line;


	if(Bas)
	{
		clr1=GRAY5;
		clr2=GRAY4;
		clr3=GRAY2;
	}
	else
	{
		clr1=BLACK;
		clr2=GRAY2;
		clr3=GRAY4;
	}

	TFT_fill_rect(x1+3,y1+3,x2-3,y2-3,GRAY3);
	TFT_Line_H(x1,y1,x2,clr2);
	TFT_Line_H(x1,y2,x2,clr3);
	TFT_Line_H(x1+1,y1+1,x2-1,clr2);
	TFT_Line_H(x1+1,y2-1,x2-1,clr3);
	TFT_Line_H(x1+2,y1+2,x2-2,clr2);
	TFT_Line_H(x1+2,y2-2,x2-2,clr3);

	TFT_Line_V(x1,y1,y2,clr2);
	TFT_Line_V(x2,y1+1,y2,clr3);
	TFT_Line_V(x1+1,y1,y2-1,clr2);
	TFT_Line_V(x2-1,y1+2,y2,clr3);
	TFT_Line_V(x1+2,y1,y2-2,clr2);
	TFT_Line_V(x2-2,y1+3,y2,clr3);

	Pixel(x1,y1,clr3);
	Pixel(x1+1,y1+1,clr3);
	Pixel(x1+2,y1+2,clr3);

	Pixel(x2,y2,clr2);
	Pixel(x2-1,y2-1,clr2);
	Pixel(x2-2,y2-2,clr2);

	str_wide=(x2-x1-strlen(str1)*FontW)/2;
	line=y1+(y2-y1-FontW)/2;
	TFT_Text(str1,x1+str_wide,line,FontW,clr1,GRAY3);
	str_wide=(x2-x1-strlen(str2)*FontW)/2;
	line+=FontW;
	TFT_Text(str2,x1+str_wide,line,FontW,clr1,GRAY3);
}

void KeyPad3x4(int X,int Y,uint8_t KeyWide,uint8_t KeyHigh,unsigned KeyAralik,uint8_t FontW)
{
	uint8_t i,j,chr[2],tmp;
	uint16_t ex,ey;
	chr[1]=0;
	for(j=0;j<4;j++)
	{
		for(i=0;i<3;i++)
		{
			tmp=i+j*3;
			if(tmp<10)
				chr[0]=49+tmp;
				if(tmp==9) 	chr[0]=128;
			else{
				if(tmp==10) chr[0]=48;
				if(tmp==11) chr[0]=129;
			}
			ex=X+(KeyWide+KeyAralik)*i;
			ey=Y+(KeyHigh+KeyAralik)*j;
			Buton(ex,ey,ex+KeyWide,ey+KeyHigh,chr,FontW,BasiliDegil);
			TaskAdd(tmp,ex,ey,ex+KeyWide,ey+KeyHigh);
		}
	}
	//TaskIndex=12;
}

void TextBox(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,char *str,uint8_t Hizalama,uint8_t FontW,uint16_t T_bck_clr)
{
	uint16_t str_wide;
	TFT_fill_rect(x1+2,y1+2,x2-2,y2-2,T_bck_clr);
	TFT_Line2_H(x1+2,y1,x2,BLACK);
	TFT_Line2_H(x1,y2,x2-2,GRAY4);
	TFT_Line2_V(x2,y1,y2-2,BLACK);
	TFT_Line2_V(x1,y1,y2,GRAY4);

	switch(Hizalama){
		case Ortala:		str_wide=(x2-x1-strlen(str)*FontW)/2;
							TFT_Text(str,x1+str_wide,y1+(y2-y1-FontW)/2,FontW,BLACK,T_bck_clr);break;
		case SolaDayali:	TFT_Text(str,x1+4,y1+(y2-y1-FontW)/2,FontW,BLACK,T_bck_clr);break;
		case SagaDayali:	str_wide=(x2-x1-strlen(str)*FontW)/2;
							TFT_Text(str,x2-str_wide,y1+(y2-y1-FontW)/2,FontW,BLACK,T_bck_clr);break;
	}
}


void FanBar(uint16_t FColor,uint16_t VColor,uint16_t BColor,uint16_t Val){
	uint16_t by=239,bx=42;
	for(uint8_t k=0;k<15;k++){
		by-=6;
		TFT_Rectangle(bx,by,bx+12,239,FColor);
		if(Val>k) TFT_fill_rect(bx+1,by+1,bx+11,238,VColor);
		else 	  TFT_fill_rect(bx+1,by+1,bx+11,238,BColor);
		bx+=16;
	}
}
*/


#endif /* INC_GRAFIK_H_ */

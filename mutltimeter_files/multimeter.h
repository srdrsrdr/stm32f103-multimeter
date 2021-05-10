#include "grafik.h"
#include "multimeterDef.h"

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

uint32_t enTmp;
uint16_t encoderButBuf;
uint8_t encoderButFlag = 0;
uint32_t encoderButTime;

uint8_t screenStart = 0;
uint8_t scopeBuf[scopeBufLen];
uint8_t kanalIndex = 0;
uint16_t kanal1[scopeBufLen];
uint8_t verticalOffset = 60;

uint8_t menuEn = 0;
menu_t menu;
scopeIndexes_t scopeIndex;


timeDiv_t TimeDiv[] = {
		{	71,	999	},
		{	71,	499	},
		{	35,	99	},
		{	17,	99	},
		{	17,	19	},
		{	35,	1	},
};


void timeDiv(uint8_t kademe)
{
	kademe = kademe % 6;
	TIM3->PSC = TimeDiv[kademe].regPSC;
	TIM3->ARR = TimeDiv[kademe].regARR;
}

void ScopeScreenClear()
{
	for(uint8_t i = 0;i < 240;i++)
	{
		if(i == 120) LCD_Pixel(i, scopeBuf[i], scopeCenterLineColor);
		else if(i == 239) LCD_Pixel(i, scopeBuf[i], scopeLineColor);
		else
		{
			if(scopeBuf[i] == 74) LCD_Pixel(i, 74, scopeCenterLineColor);
			else
			{
				if(i % 20 == 0) LCD_Pixel(i, scopeBuf[i], scopeLineColor);
				else
				{
					if((scopeBuf[i] - screenTop) % 20) LCD_Pixel(i, scopeBuf[i], scopeBackColor);
					else LCD_Pixel(i, scopeBuf[i], scopeLineColor);
				}
			}
		}
	}
}

void bufAktar()
{
	for(uint8_t i = 0;i < 240;i++)
	{
		scopeBuf[i] = screenBottom - (uint8_t)(((kanal1[i] >> 4) * 120) / 255);
	}
}

void ScopeCiz()
{
	for(uint8_t i = 1;i < 240;i++) LCD_Pixel(i, scopeBuf[i], scopeSignalColor);
}


void ScopeScreenInit()
{
	LCD_Clear(scopeBackColor);

	for(uint8_t i = 14;i < 135;i += 20) Line_H(0, i, 239, scopeLineColor);
	for(uint8_t i = 0;i < 239;i += 20) Line_V(i,14, 134, scopeLineColor);
	Line_V(239,15, 133, scopeLineColor);

	Line_H(0, 74, 239, scopeCenterLineColor);
	Line_V(120,14, 134, scopeCenterLineColor);

	for(uint8_t i = 1;i < 240;i++) scopeBuf[i] = 74;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(menuEn) return;
	ScopeScreenClear();
	bufAktar();
	ScopeCiz();
	screenStart = 1;
}
void menuSelect()
{
	menu.pointer = &menuTimeDiv;
	menu.maxIndex = menuTimeDivLen - 1;
	menu.level = 2;
	menu.index = scopeIndex.timeDivIndex;
	menu.x = timDivMenuStart;
	menu.w = timDivMenuWidth;
}

void menuDisplay()
{
	menuEn = 1;
	ScopeScreenInit();

	for(uint8_t i = 0;i <= menu.maxIndex;i++)
	{
		if(i == menu.index)
		{
			LCD_fillRect(menu.x, 16 * i, menu.w, 16, menuActiveColor);
			drawText(*(menu.pointer + i), menu.x + 5, 16 * i, 16, scopeSignalColor, menuActiveColor);
		}
		else
		{
			LCD_fillRect(menu.x, 16 * i, menu.w, 16, menuColor);
			drawText(*(menu.pointer + i), menu.x + 5, 16 * i, 16, scopeSignalColor, menuColor);
		}
	}

	menuEn = 0;
}

void multimeterInit()
{
	enTmp = TIM1->CNT = 0;

	encoderButTime = HAL_GetTick();
	menuSelect();
}

void Run()
{
	if(enTmp != TIM1->CNT)
	{
		if(TIM1->CNT > menu.maxIndex) TIM1->CNT = menu.maxIndex;

		enTmp = TIM1->CNT;
		menu.index = enTmp;
		menuDisplay();
	}

	if(encoderButTime - HAL_GetTick() > encoderButPeriod)
	{
		encoderButBuf <<= 1;
		if(!(GPIOA->IDR & (0x1UL << (10U)))) encoderButBuf |= 1;
		if(encoderButBuf - 0xffff == 0) encoderButFlag = 1; else encoderButFlag = 0;
		encoderButTime = HAL_GetTick();
	}

	if(encoderButFlag)
	{
		menuDisplay();
		encoderButFlag = 0;
	}

}

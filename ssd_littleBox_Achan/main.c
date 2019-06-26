/* ------------------------------------------
 * Copyright (c) 2017, Synopsys, Inc. All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1) Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.

 * 3) Neither the name of the Synopsys, Inc., nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
--------------------------------------------- */

/**
 * \defgroup	EMBARC_APP_TMPL		embARC Template Example
 * \ingroup	EMBARC_APPS_TOTAL
 * \ingroup	EMBARC_APPS_BOARD_EMSK
 * \ingroup	EMBARC_APPS_BAREMETAL
 * \brief	embARC Example for template
 *
 * \details
 * ### Extra Required Tools
 *
 * ### Extra Required Peripherals
 *
 * ### Design Concept
 *
 * ### Usage Manual
 *
 * ### Extra Comments
 *
 */

/**
 * \file
 * \ingroup	EMBARC_APP_TMPL
 * \brief	main source of template example
 */

/**
 * \addtogroup	EMBARC_APP_TMPL
 * @{
 */
/* embARC HAL */
#include "embARC.h"
#include "embARC_debug.h"
#include "stdio.h"
#include "dev_uart.h"
#include "stdlib.h"
#include "math.h"
//#include "dev_gpio.h"

/* middleware level*/
#include "u8g.h"

#define IOTDK_WIND_ID DFSS_GPIO_8B3_ID

/* For ssd calculation */
float ssd;
float t = 26;
float f = 30;
float v = 2;
char* data;
char buf[32];
char buf_transform[32];
float tem;
float hum;
int32_t test;

/*For wind sensor */
const float zeroWindAdjustment =  0.2; 
uint32_t TMP_Therm_ADunits;  
uint32_t RV_Wind_ADunits;   
float RV_Wind_Volts;
unsigned long lastMillis;
float TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;

u8g_t u8g;
DEV_UART* uart2;

void u8g_prepare(void) {
	u8g_SetFont(&u8g, u8g_font_6x10);		/* set the current font and reset the font reference position to "Baseline" */
	u8g_SetFontRefHeightExtendedText(&u8g);		/* define the calculation method for the ascent and descent of the current font */
	u8g_SetDefaultForegroundColor(&u8g);		/* assign one of the default colors as current color index */
	u8g_SetFontPosTop(&u8g);			/* set the reference position for the character and string draw procedure */
}

/** first page in OLED */
void u8g_first_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Welcome!!");		/* draws a string at the specified x/y position */
}

/** second page in OLED */
void u8g_second_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is -4 degree.");
	u8g_DrawStr(&u8g, 0, 20, "Please add many coat,");
	u8g_DrawStr(&u8g, 0, 30, "to avoid freezing.");
}

/** third page in OLED */
void u8g_third_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is -3 degree.");
	u8g_DrawStr(&u8g, 0, 20, "Please add many");
	u8g_DrawStr(&u8g, 0, 30, "clothes.");
}

/** forth page in OLED */
void u8g_forth_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is -2 degree.");
	u8g_DrawStr(&u8g, 0, 20, "Please add a little ");
	u8g_DrawStr(&u8g, 0, 30, "bit clothes.");
}

/** fifth page in OLED */
void u8g_fifth_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is -1 degree.");
	u8g_DrawStr(&u8g, 0, 20, "May add some clothes,");
	u8g_DrawStr(&u8g, 0, 30, "cool outside");
}

/** sixth page in OLED */
void u8g_sixth_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is 0 degree.");
	u8g_DrawStr(&u8g, 0, 20, "Very comfortable now!");
}


/** seventh page in OLED */
void u8g_seventh_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is 1 degree.");
	u8g_DrawStr(&u8g, 0, 20, "May need to take off ");
	u8g_DrawStr(&u8g, 0, 30, "some clothes.");
}

/** eighth page in OLED */
void u8g_eighth_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is 2 degree.");
	u8g_DrawStr(&u8g, 0, 20, "Please turn on your ");
	u8g_DrawStr(&u8g, 0, 30, "fan.");
}

/** ninth page in OLED */
void u8g_ninth_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is 3 degree.");
	u8g_DrawStr(&u8g, 0, 20, "Please eat much ice ");
	u8g_DrawStr(&u8g, 0, 30, "cream as you can.");
}

/** tenth page in OLED */
void u8g_tenth_frame(void) {
	u8g_DrawStr(&u8g, 0, 0, "Ssd now is 4 degree.");
	u8g_DrawStr(&u8g, 0, 20, "Please turn on AC now");
	u8g_DrawStr(&u8g, 0, 30, "to avoid heatstroke.");
}

uint8_t draw_state = 0;

/** draw five pages in OLED */
void draw(void) {
	u8g_prepare();
	switch(draw_state) {
		case 0: u8g_first_frame(); break;	//Welcome page
		case 1: u8g_second_frame(); break;	//-4 page
		case 2: u8g_third_frame(); break;	//-3 page
		case 3: u8g_forth_frame(); break;	//-2 page
		case 4: u8g_fifth_frame(); break;	//-1 page
		case 5: u8g_sixth_frame(); break;	//0 page
		case 6: u8g_seventh_frame(); break;	//1 page
		case 7: u8g_eighth_frame(); break;	//2 page
		case 8: u8g_ninth_frame(); break;	//3 page
		case 9: u8g_tenth_frame(); break;	//4 page
	}
}


/** main entry for running ntshell */
int main(void)
{
	DEV_GPIO* RV_Pin;
	//DEV_GPIO_PTR TMP_Pin;

	io_arduino_config(ARDUINO_PIN_AD0, ARDUINO_ADC, IO_PINMUX_ENABLE);
	io_arduino_config(ARDUINO_PIN_AD1, ARDUINO_ADC, IO_PINMUX_ENABLE);
	
	RV_Pin = gpio_get_dev(IOTDK_WIND_ID);
	//TMP_Pin = gpio_get_dev(IOTDK_WIND_ID);
	if(RV_Pin->gpio_open((1 << 7)) == E_OK){
		RV_Pin->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)(1 << 7));
		EMBARC_PRINTF("OOpen,A0!\r\n");
	}
	if(RV_Pin->gpio_open((1 << 6)) == E_OPNED){
		RV_Pin->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)(1 << 6));
		EMBARC_PRINTF("Open,A1!\r\n");
	}
	
	
	uart2 = uart_get_dev(DFSS_UART_1_ID);	//Open UART1 transmition
	if(uart2->uart_open(9600) == E_OK){
		EMBARC_PRINTF("Open!\r\n");
	}

	u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_2x_i2c, U8G_COM_SSD_I2C); /* create a new interface to a graphics display */

	EMBARC_PRINTF("u8glib\r\n");

	EMBARC_PRINTF("Display Width: %u, Display Height: %u\r\n" , u8g_GetWidth(&u8g), u8g_GetHeight(&u8g));

	u8g_Begin(&u8g); /* reset display and put it into default state */
	
	if(RV_Pin->gpio_read(&TMP_Therm_ADunits, 1<<7) == E_OK){
			
		printf("Success1\n");
	}
	if(RV_Pin->gpio_read(&RV_Wind_ADunits, 1<<6) == E_OK){
		printf("Success2\n");
	}
   	RV_Wind_Volts = ((float)RV_Wind_ADunits *  0.0048828125);

	TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

	zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

	zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

	WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /0.2300) , 2.7265); 

	while(1) {
		//printf("%f\n",WindSpeed_MPH);
		//printf("%d\n",RV_Wind_ADunits );
		for(int i = 0;i < 6;i++){						//Read temperature from ESP8266
			test = uart2->uart_read(&data,1);
			buf[i] = data;
		}
		
		/*======================== Recognize whether tem or hum ============================ */
		if(buf[0] == '|'){
			for(int i = 0;i < 2; i++)
				buf_transform[i] = buf[i+1];
			tem = atof(buf_transform);
		}
		else if(buf[0] == '#'){
			for(int i = 0;i < 2; i++)
				buf_transform[i] = buf[i+1];
			hum = atof(buf_transform);
		}

		if(buf[3] == '|'){
			for(int i = 0;i < 2; i++)
				buf_transform[i] = buf[i+4];
			tem = atof(buf_transform);
		}
		else if(buf[3] == '#'){
			for(int i = 0;i < 2; i++)
				buf_transform[i] = buf[i+4];
			hum = atof(buf_transform);
		}
		/*================================================================================= */

		//printf("Current temperature : %f, Current humidity : %f, Current wind speed : %f\n",tem,hum,WindSpeed_MPH);

		ssd = (1.818 * tem + 18.18) * (0.88 + 0.002 * hum) + (tem - 32)/(45 - tem) - 3.2 * WindSpeed_MPH + 18.2;
		//printf("Current ssd : %f\n\n",ssd);

		if(ssd < 25){
			draw_state = 1;
		}
		else if(ssd >= 25 && ssd < 38){
			draw_state = 2;
		}
		else if(ssd >= 38 && ssd < 50){
			draw_state = 3;
		}
		else if(ssd >= 50 && ssd < 58){
			draw_state = 4;
		}
		else if(ssd >= 58 && ssd < 70){
			draw_state = 5;
		}
		else if(ssd >= 70 && ssd < 75){
			draw_state = 6;
		}
		else if(ssd >= 75 && ssd < 79){
			draw_state = 7;
		}
		else if(ssd >= 79 && ssd < 85){
			draw_state = 8;
		}
		else if(ssd >= 85){
			draw_state = 9;
		}

		/* picture loop */
		u8g_FirstPage(&u8g); /* marks the beginning of the picture loop; it cannot be used inside the picture loop */
		do {
			draw();
		} while (u8g_NextPage(&u8g)); /* marks the end of the body of the picture loop */
		//board_delay_ms(1000, 1);
	}

	return E_SYS;
}

/** @} */
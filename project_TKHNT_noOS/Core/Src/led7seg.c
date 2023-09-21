/*
 * The led 7 segments library
 * Nguyen Dinh Khanh
 * To use: call display_float(float) function
 * The dot in second section, if you want to change, see on function
 */



#include "main.h"
#include <led7seg.h>


//if not define pins name, define its:
/*
#define A_pin_Pin GPIO_PIN_0
#define A_pin_GPIO_Port GPIOD
#define B_pin_Pin GPIO_PIN_1
#define B_pin_GPIO_Port GPIOD
#define C_pin_Pin GPIO_PIN_2
#define C_pin_GPIO_Port GPIOD
#define D_pin_Pin GPIO_PIN_3
#define D_pin_GPIO_Port GPIOD
#define E_pin_Pin GPIO_PIN_4
#define E_pin_GPIO_Port GPIOD
#define F_pin_Pin GPIO_PIN_5
#define F_pin_GPIO_Port GPIOD
#define G_pin_Pin GPIO_PIN_6
#define G_pin_GPIO_Port GPIOD
#define Dot_pin_Pin GPIO_PIN_7
#define Dot_pin_GPIO_Port GPIOD
#define Digit1_pin_Pin GPIO_PIN_3
#define Digit1_pin_GPIO_Port GPIOB
#define Digit2_pin_Pin GPIO_PIN_4
#define Digit2_pin_GPIO_Port GPIOB
#define Digit3_pin_Pin GPIO_PIN_5
#define Digit3_pin_GPIO_Port GPIOB
#define Digit4_pin_Pin GPIO_PIN_6
#define Digit4_pin_GPIO_Port GPIOB
#define ds18b20_Pin GPIO_PIN_7
#define ds18b20_GPIO_Port GPIOB
*/

uint8_t digit1, digit2, digit3, digit4;

uint8_t numberOfSegment[10]={
		0xc0,
		0xf9,
		0xa4,
		0xb0,
		0x99,
		0x92,
		0x82,
		0xf8,
		0x80,
		0x90
};

void display_digit(uint8_t number)
{
	HAL_GPIO_WritePin(A_pin_GPIO_Port, A_pin_Pin, ((number>>0)&0x01));
	HAL_GPIO_WritePin(B_pin_GPIO_Port, B_pin_Pin, ((number>>1)&0x01));
	HAL_GPIO_WritePin(C_pin_GPIO_Port, C_pin_Pin, ((number>>2)&0x01));
	HAL_GPIO_WritePin(D_pin_GPIO_Port, D_pin_Pin, ((number>>3)&0x01));
	HAL_GPIO_WritePin(E_pin_GPIO_Port, E_pin_Pin, ((number>>4)&0x01));
	HAL_GPIO_WritePin(F_pin_GPIO_Port, F_pin_Pin, ((number>>5)&0x01));
	HAL_GPIO_WritePin(G_pin_GPIO_Port, G_pin_Pin, ((number>>6)&0x01));
	HAL_GPIO_WritePin(Dot_pin_GPIO_Port, Dot_pin_Pin, ((number>>7)&0x01));
}

void display_float(float temperature)
{
	int hundred_temp = (int)(temperature * 100);

	digit1 = hundred_temp/1000;
	digit2 = ((hundred_temp/100)%10);
	digit3 = ((hundred_temp/10)%10);
	digit4 = (hundred_temp%10);
	
	display_digit(numberOfSegment[digit1]);
	HAL_GPIO_WritePin(Digit1_pin_GPIO_Port, Digit1_pin_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Digit1_pin_GPIO_Port, Digit1_pin_Pin, GPIO_PIN_RESET);

	display_digit(numberOfSegment[digit2]);
	HAL_GPIO_WritePin(Dot_pin_GPIO_Port, Dot_pin_Pin, GPIO_PIN_RESET);  //dot in 2nd section, move it if you want
	HAL_GPIO_WritePin(Digit2_pin_GPIO_Port, Digit2_pin_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Digit2_pin_GPIO_Port, Digit2_pin_Pin, GPIO_PIN_RESET);

	display_digit(numberOfSegment[digit3]);
	HAL_GPIO_WritePin(Digit3_pin_GPIO_Port, Digit3_pin_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Digit3_pin_GPIO_Port, Digit3_pin_Pin, GPIO_PIN_RESET);

	display_digit(numberOfSegment[digit4]);
	HAL_GPIO_WritePin(Digit4_pin_GPIO_Port, Digit4_pin_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Digit4_pin_GPIO_Port, Digit4_pin_Pin, GPIO_PIN_RESET);
}

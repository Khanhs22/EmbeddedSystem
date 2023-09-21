#include"main.h"
#include"keypad.h"

#define COLUMNS 3
#define ROWS 4

//define for keypad 4x3
GPIO_TypeDef* rowPort_array[ROWS] = {ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port};
uint16_t rowPin_array[ROWS] = {ROW1_Pin, ROW2_Pin, ROW3_Pin,  ROW4_Pin};


GPIO_TypeDef* columnPort_array[COLUMNS] = {COLUMN1_GPIO_Port, COLUMN2_GPIO_Port, COLUMN3_GPIO_Port};
uint16_t columnPin_array[COLUMNS] = {COLUMN1_Pin, COLUMN2_Pin, COLUMN3_Pin};

char keyPad[ROWS][COLUMNS] = {
									{'1', '2', '3',},
							  		{'4', '5', '6',},
							 		{'7', '8', '9',},
							 		{'*', '0', '#'}};

/*uncomment if don't define pull-up mode for column pin
void initKeyPad(void)
{
	HAL_GPIO_WritePin(COLUMN1_GPIO_Port, COLUMN1_Pin, 1);
	HAL_GPIO_WritePin(COLUMN2_GPIO_Port, COLUMN2_Pin, 1);
	HAL_GPIO_WritePin(COLUMN3_GPIO_Port, COLUMN3_Pin, 1);
//	HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, 1);
//	HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, 1);
//	HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, 1);
//	HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, 1);
}

*/
char getKey(void)
{
	char key = 0;
	//initKeyPad();
	for(int i = 0; i < ROWS; i++)
	{
		HAL_GPIO_WritePin(rowPort_array[i], rowPin_array[i], 0);
		for(int m = 0; m < COLUMNS; m++)
		{
			if(HAL_GPIO_ReadPin(columnPort_array[m], columnPin_array[m]) == 0)
			{
				HAL_Delay(10);
				if(HAL_GPIO_ReadPin(columnPort_array[m], columnPin_array[m]) == 0)
				{
					key = keyPad[i][m];
					while(!HAL_GPIO_ReadPin(columnPort_array[m], columnPin_array[m])){};
					return key;
				}
			}
		}
		HAL_GPIO_WritePin(rowPort_array[i], rowPin_array[i], 1);
	}
	return key;
}

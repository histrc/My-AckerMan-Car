#include "key.h"
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*����Ƿ��а������� */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) ==GPIO_PIN_RESET )  
	{	
		HAL_Delay(10);
		/*�ȴ������ͷ� */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) ==GPIO_PIN_RESET);
		HAL_Delay(10);	
		return 	GPIO_PIN_RESET;	 
	}
	else
		return GPIO_PIN_SET;
}

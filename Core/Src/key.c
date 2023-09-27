#include "key.h"
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/*检测是否有按键按下 */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) ==GPIO_PIN_RESET )  
	{	
		HAL_Delay(10);
		/*等待按键释放 */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) ==GPIO_PIN_RESET);
		HAL_Delay(10);	
		return 	GPIO_PIN_RESET;	 
	}
	else
		return GPIO_PIN_SET;
}
